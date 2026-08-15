import random
import statistics

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer, with_timeout
from cocotb.utils import get_sim_time


CLK_NS = 10
ERROR_INVALID_COMMAND = 1
ERROR_ADDRESS_NACK = 2
ERROR_DATA_NACK = 3
ERROR_STRETCH_TIMEOUT = 4
ERROR_TRANSACTION_TIMEOUT = 5
ERROR_RECOVERY_FAILED = 6


class OpenDrainBus:
    def __init__(self, dut):
        self.dut = dut
        self.external_scl_low = False
        self.external_sda_low = False

    async def resolve(self):
        while True:
            scl_low = bool(self.dut.scl_drive_low.value) or self.external_scl_low
            sda_low = bool(self.dut.sda_drive_low.value) or self.external_sda_low
            self.dut.scl_i.value = not scl_low
            self.dut.sda_i.value = not sda_low
            await Timer(1, unit="ns")


class ScriptedSlave:
    """Edge-driven I2C target; no DUT state or internal signal is observed."""

    def __init__(self, dut, bus, rng=None, random_delays=False):
        self.dut = dut
        self.bus = bus
        self.rng = rng
        self.random_delays = random_delays
        self.starts = []
        self.stops = 0
        self.addresses = []
        self.received = []
        self.master_acks = []

    async def _wait_line_edge(self, trigger, level):
        while True:
            await trigger(self.dut.sda_i)
            # A START/STOP must persist with SCL high, not be a resolver delta
            # from simultaneous SCL/SDA changes.
            await ClockCycles(self.dut.clk, 2)
            if int(self.dut.scl_i.value) == 1 and int(self.dut.sda_i.value) == level:
                return

    async def wait_start(self, repeated=False):
        await self._wait_line_edge(FallingEdge, 0)
        self.starts.append("restart" if repeated else "start")

    async def wait_stop(self):
        await self._wait_line_edge(RisingEdge, 1)
        self.stops += 1

    async def _stretch_this_bit(self):
        if not self.random_delays or self.rng.randrange(4) != 0:
            return
        self.bus.external_scl_low = True
        await FallingEdge(self.dut.scl_drive_low)
        await ClockCycles(self.dut.clk, self.rng.randint(1, 3))
        self.bus.external_scl_low = False

    async def _clock_and_sample(self):
        await self._stretch_this_bit()
        await RisingEdge(self.dut.scl_i)
        await Timer(1, unit="ps")
        bit = int(self.dut.sda_i.value)
        await FallingEdge(self.dut.scl_i)
        return bit

    async def receive_byte(self, wait_for_low=True):
        if wait_for_low:
            await FallingEdge(self.dut.scl_i)
        value = 0
        for _ in range(8):
            value = (value << 1) | await self._clock_and_sample()
        return value

    async def drive_ack(self, ack=True):
        delay = self.rng.randint(0, 1) if self.random_delays else 0
        if delay:
            await ClockCycles(self.dut.clk, delay)
        self.bus.external_sda_low = ack
        await self._stretch_this_bit()
        await RisingEdge(self.dut.scl_i)
        await FallingEdge(self.dut.scl_i)
        self.bus.external_sda_low = False

    async def send_byte(self, value):
        for bit_index in range(7, -1, -1):
            self.bus.external_sda_low = not bool((value >> bit_index) & 1)
            await self._stretch_this_bit()
            await RisingEdge(self.dut.scl_i)
            await FallingEdge(self.dut.scl_i)
        self.bus.external_sda_low = False
        await self._stretch_this_bit()
        await RisingEdge(self.dut.scl_i)
        await Timer(1, unit="ps")
        ack = int(self.dut.sda_i.value) == 0
        self.master_acks.append(ack)
        await FallingEdge(self.dut.scl_i)

    async def receive_address(self, expected_address, read, ack=True, repeated=False):
        await self.wait_start(repeated=repeated)
        address_byte = await self.receive_byte()
        self.addresses.append(address_byte)
        assert address_byte == (expected_address << 1) | read
        await self.drive_ack(ack)

    async def write(self, address, count, address_ack=True, nack_index=None):
        await self.receive_address(address, read=0, ack=address_ack)
        if not address_ack:
            await self.wait_stop()
            return
        for index in range(count):
            value = await self.receive_byte(wait_for_low=False)
            self.received.append(value)
            await self.drive_ack(index != nack_index)
            if index == nack_index:
                await self.wait_stop()
                return
        await self.wait_stop()

    async def read(self, address, values, address_ack=True):
        await self.receive_address(address, read=1, ack=address_ack)
        if not address_ack:
            await self.wait_stop()
            return
        for value in values:
            await self.send_byte(value)
        await self.wait_stop()

    async def combined(self, address, writes, reads):
        await self.receive_address(address, read=0, ack=True)
        for value in writes:
            observed = await self.receive_byte(wait_for_low=False)
            self.received.append(observed)
            await self.drive_ack(True)
        await self.receive_address(address, read=1, ack=True, repeated=True)
        for value in reads:
            await self.send_byte(value)
        await self.wait_stop()


async def start_environment(dut):
    clock = Clock(dut.clk, CLK_NS, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    bus = OpenDrainBus(dut)
    resolver_task = cocotb.start_soon(bus.resolve())
    dut.cmd_valid.value = 0
    dut.cmd_address.value = 0
    dut.cmd_write_count.value = 0
    dut.cmd_read_count.value = 0
    dut.cmd_fast_mode.value = 1
    dut.tx_valid.value = 0
    dut.tx_data.value = 0
    dut.rx_ready.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    await Timer(1, unit="ps")
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    return bus, [clock_task, resolver_task]


def stop_tasks(tasks):
    for task in tasks:
        task.cancel()


async def issue_command(dut, address, writes=0, reads=0, fast=True):
    while not int(dut.cmd_ready.value):
        await RisingEdge(dut.clk)
    dut.cmd_address.value = address
    dut.cmd_write_count.value = writes
    dut.cmd_read_count.value = reads
    dut.cmd_fast_mode.value = fast
    dut.cmd_valid.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_valid.value = 0


async def feed_tx(dut, values):
    for value in values:
        dut.tx_data.value = value
        dut.tx_valid.value = 1
        while not int(dut.tx_ready.value):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        dut.tx_valid.value = 0


async def collect_rx(dut, count, delays=None):
    values = []
    for index in range(count):
        while not int(dut.rx_valid.value):
            await RisingEdge(dut.clk)
        delay = delays[index] if delays else 0
        dut.rx_ready.value = 0
        await ClockCycles(dut.clk, delay)
        values.append(int(dut.rx_data.value))
        dut.rx_ready.value = 1
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        dut.rx_ready.value = 0
    return values


async def wait_completion(dut):
    while True:
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        if int(dut.done.value):
            return (
                int(dut.error.value),
                int(dut.error_code.value),
                int(dut.error_byte_index.value),
            )


async def completed(dut, timeout_us=100):
    return await with_timeout(wait_completion(dut), timeout_us, "us")


async def reset_now(dut):
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await Timer(1, unit="ps")
    assert int(dut.busy.value) == 0
    assert int(dut.cmd_ready.value) == 1
    assert int(dut.scl_drive_low.value) == 0
    assert int(dut.sda_drive_low.value) == 0
    assert int(dut.done.value) == 0
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


@cocotb.test()
async def reset_idle_open_drain_and_zero_length(dut):
    bus, tasks = await start_environment(dut)
    try:
        assert int(dut.cmd_ready.value) == 1
        assert int(dut.busy.value) == 0
        assert int(dut.scl_i.value) == 1 and int(dut.sda_i.value) == 1
        for _ in range(10):
            await RisingEdge(dut.clk)
            assert int(dut.scl_drive_low.value) in (0, 1)
            assert int(dut.sda_drive_low.value) in (0, 1)
            assert int(dut.scl_drive_low.value) == 0
            assert int(dut.sda_drive_low.value) == 0
        await issue_command(dut, 0x2A)
        await Timer(1, unit="ps")
        assert (
            int(dut.done.value),
            int(dut.error.value),
            int(dut.error_code.value),
        ) == (1, 1, ERROR_INVALID_COMMAND)
        assert int(dut.busy.value) == 0
        assert not bus.external_scl_low and not bus.external_sda_low
    finally:
        stop_tasks(tasks)


async def capture_scl_widths(dut, lows, highs):
    previous = int(dut.scl_i.value)
    last_edge = None
    while True:
        await dut.scl_i.value_change
        now = int(get_sim_time(unit="ns"))
        current = int(dut.scl_i.value)
        if last_edge is not None:
            (lows if previous == 0 else highs).append(now - last_edge)
        previous = current
        last_edge = now
@cocotb.test()
async def standard_and_fast_bus_timing(dut):
    bus, tasks = await start_environment(dut)
    try:
        measurements = {}
        for fast in (True, False):
            slave = ScriptedSlave(dut, bus)
            slave_task = cocotb.start_soon(slave.write(0x31, 0, address_ack=False))
            lows = []
            highs = []
            edge_task = cocotb.start_soon(capture_scl_widths(dut, lows, highs))
            await issue_command(dut, 0x31, writes=1, fast=fast)
            assert await completed(dut) == (1, ERROR_ADDRESS_NACK, 0)
            await with_timeout(slave_task, 20, "us")
            edge_task.cancel()
            measurements[fast] = (statistics.median(lows), statistics.median(highs))
        fast_low, fast_high = measurements[True]
        standard_low, standard_high = measurements[False]
        assert 35 <= fast_low <= 50
        assert 55 <= fast_high <= 75
        assert 95 <= standard_low <= 110
        assert 115 <= standard_high <= 135
        assert 2.2 <= standard_low / fast_low <= 2.8
        assert 1.7 <= standard_high / fast_high <= 2.3
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def multi_byte_write(dut):
    bus, tasks = await start_environment(dut)
    try:
        values = [0x00, 0xA5, 0xFF, 0x3C]
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.write(0x52, len(values)))
        tx_task = cocotb.start_soon(feed_tx(dut, values))
        await issue_command(dut, 0x52, writes=len(values))
        assert await completed(dut) == (0, 0, 0)
        await with_timeout(slave_task, 20, "us")
        await with_timeout(tx_task, 20, "us")
        assert slave.received == values
        assert slave.addresses == [0xA4]
        assert slave.starts == ["start"] and slave.stops == 1
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def read_backpressure_and_final_nack(dut):
    bus, tasks = await start_environment(dut)
    try:
        values = [0x12, 0x80, 0xFE]
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.read(0x19, values))
        await issue_command(dut, 0x19, reads=len(values))
        rx_task = cocotb.start_soon(collect_rx(dut, len(values), delays=[8, 1, 12]))
        assert await completed(dut) == (0, 0, 0)
        assert await with_timeout(rx_task, 20, "us") == values
        await with_timeout(slave_task, 20, "us")
        assert slave.master_acks == [True, True, False]
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def combined_write_repeated_start_read(dut):
    bus, tasks = await start_environment(dut)
    try:
        writes = [0x0F, 0x20]
        reads = [0xBE, 0xEF]
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.combined(0x44, writes, reads))
        tx_task = cocotb.start_soon(feed_tx(dut, writes))
        await issue_command(dut, 0x44, writes=len(writes), reads=len(reads))
        rx_task = cocotb.start_soon(collect_rx(dut, len(reads)))
        assert await completed(dut) == (0, 0, 0)
        assert await with_timeout(rx_task, 20, "us") == reads
        await with_timeout(slave_task, 20, "us")
        await with_timeout(tx_task, 20, "us")
        assert slave.received == writes
        assert slave.addresses == [0x88, 0x89]
        assert slave.starts == ["start", "restart"] and slave.stops == 1
        assert slave.master_acks == [True, False]
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def deterministic_random_response_delays_and_stretch(dut):
    bus, tasks = await start_environment(dut)
    try:
        rng = random.Random(0x1C2C)
        writes = [rng.randrange(256) for _ in range(5)]
        reads = [rng.randrange(256) for _ in range(4)]
        slave = ScriptedSlave(dut, bus, rng=rng, random_delays=True)
        slave_task = cocotb.start_soon(slave.combined(0x2D, writes, reads))
        tx_task = cocotb.start_soon(feed_tx(dut, writes))
        await issue_command(dut, 0x2D, writes=len(writes), reads=len(reads))
        rx_task = cocotb.start_soon(collect_rx(dut, len(reads), delays=[2, 5, 1, 7]))
        assert await completed(dut) == (0, 0, 0)
        assert await with_timeout(rx_task, 30, "us") == reads
        await with_timeout(slave_task, 30, "us")
        await with_timeout(tx_task, 30, "us")
        assert slave.received == writes
        assert slave.master_acks == [True, True, True, False]
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def address_nack(dut):
    bus, tasks = await start_environment(dut)
    try:
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.read(0x63, [], address_ack=False))
        await issue_command(dut, 0x63, reads=1)
        assert await completed(dut) == (1, ERROR_ADDRESS_NACK, 0)
        await with_timeout(slave_task, 20, "us")
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def data_nack_at_every_byte_position(dut):
    bus, tasks = await start_environment(dut)
    try:
        values = [0x11, 0x22, 0x33, 0x44]
        for nack_index in range(len(values)):
            slave = ScriptedSlave(dut, bus)
            slave_task = cocotb.start_soon(
                slave.write(0x36, len(values), nack_index=nack_index)
            )
            tx_task = cocotb.start_soon(feed_tx(dut, values))
            await issue_command(dut, 0x36, writes=len(values))
            assert await completed(dut) == (1, ERROR_DATA_NACK, nack_index)
            await with_timeout(slave_task, 20, "us")
            tx_task.cancel()
            dut.tx_valid.value = 0
            assert slave.received == values[: nack_index + 1]
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def stuck_sda_recovery_is_exactly_nine_clocks_then_continues(dut):
    bus, tasks = await start_environment(dut)
    try:
        bus.external_sda_low = True
        pulse_count = 0

        async def release_after_nine():
            nonlocal pulse_count
            for _ in range(9):
                await RisingEdge(dut.scl_i)
                pulse_count += 1
            bus.external_sda_low = False

        release_task = cocotb.start_soon(release_after_nine())
        tx_task = cocotb.start_soon(feed_tx(dut, [0x5A]))
        await issue_command(dut, 0x28, writes=1)
        await with_timeout(release_task, 20, "us")
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.write(0x28, 1))
        assert await completed(dut) == (0, 0, 0)
        await with_timeout(slave_task, 30, "us")
        await with_timeout(tx_task, 20, "us")
        assert pulse_count == 9
        assert slave.received == [0x5A]
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def stuck_sda_recovery_failure(dut):
    bus, tasks = await start_environment(dut)
    try:
        bus.external_sda_low = True
        rises = 0

        async def count_rises():
            nonlocal rises
            while True:
                await RisingEdge(dut.scl_i)
                await Timer(1, unit="ps")
                if not int(dut.sda_drive_low.value):
                    rises += 1

        counter = cocotb.start_soon(count_rises())
        await issue_command(dut, 0x20, writes=1)
        assert await completed(dut) == (1, ERROR_RECOVERY_FAILED, 0)
        counter.cancel()
        assert rises == 9
        bus.external_sda_low = False
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def stuck_scl_has_bounded_timeout(dut):
    bus, tasks = await start_environment(dut)
    try:
        bus.external_scl_low = True
        await issue_command(dut, 0x20, writes=1)
        assert await completed(dut, timeout_us=20) == (1, ERROR_STRETCH_TIMEOUT, 0)
        assert int(dut.busy.value) == 0
        bus.external_scl_low = False
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def transaction_timeout_on_tx_and_rx_stream_stalls(dut):
    bus, tasks = await start_environment(dut)
    try:
        tx_slave = ScriptedSlave(dut, bus)
        tx_slave_task = cocotb.start_soon(tx_slave.receive_address(0x11, 0, True))
        await issue_command(dut, 0x11, writes=1)
        assert await completed(dut) == (1, ERROR_TRANSACTION_TIMEOUT, 0)
        await with_timeout(tx_slave_task, 20, "us")

        rx_slave = ScriptedSlave(dut, bus)

        async def supply_one_byte():
            await rx_slave.receive_address(0x12, 1, True)
            await rx_slave.send_byte(0xC7)

        rx_slave_task = cocotb.start_soon(supply_one_byte())
        await issue_command(dut, 0x12, reads=1)
        while not int(dut.rx_valid.value):
            await RisingEdge(dut.clk)
        assert int(dut.rx_data.value) == 0xC7
        assert await completed(dut) == (1, ERROR_TRANSACTION_TIMEOUT, 0)
        rx_slave_task.cancel()
    finally:
        stop_tasks(tasks)


@cocotb.test()
async def reset_during_transfer_and_recovery_releases_bus(dut):
    bus, tasks = await start_environment(dut)
    try:
        slave = ScriptedSlave(dut, bus)
        slave_task = cocotb.start_soon(slave.write(0x55, 2))
        tx_task = cocotb.start_soon(feed_tx(dut, [0xAA, 0xBB]))
        await issue_command(dut, 0x55, writes=2)
        for _ in range(4):
            await RisingEdge(dut.scl_i)
        await reset_now(dut)
        slave_task.cancel()
        tx_task.cancel()
        bus.external_sda_low = False
        dut.tx_valid.value = 0

        bus.external_sda_low = True
        await issue_command(dut, 0x21, writes=1)
        await RisingEdge(dut.scl_i)
        await RisingEdge(dut.scl_i)
        await reset_now(dut)
        bus.external_sda_low = False
        await ClockCycles(dut.clk, 4)
        assert int(dut.scl_i.value) == 1 and int(dut.sda_i.value) == 1
        assert int(dut.error.value) == 0 and int(dut.error_code.value) == 0
    finally:
        stop_tasks(tasks)
