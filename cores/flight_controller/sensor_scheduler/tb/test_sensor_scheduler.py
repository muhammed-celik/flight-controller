from collections import defaultdict, deque

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer


CLK_NS = 10
MPU_DEFAULT = bytes([0x01, 0x10, 0x11, 0x20, 0x21, 0x30, 0x31,
                     0x40, 0x41, 0x50, 0x51, 0x60, 0x61, 0x70, 0x71])
AK_DEFAULT = bytes([0x01, 0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A, 0x10])
BMP_DEFAULT = bytes([0x64, 0x32, 0x10, 0x54, 0x32, 0x10])


class OpenDrainBus:
    def __init__(self, dut):
        self.dut = dut
        self.target_sda_low = False
        self.target_scl_low = False

    async def resolve(self):
        while True:
            self.dut.scl_i.value = not (
                bool(self.dut.scl_drive_low.value) or self.target_scl_low
            )
            self.dut.sda_i.value = not (
                bool(self.dut.sda_drive_low.value) or self.target_sda_low
            )
            await Timer(5, unit="ns")


class RuntimeGy91:
    """Runtime targets driven from resolved bus edges, not adapter data ports."""

    def __init__(self, dut, bus):
        self.dut = dut
        self.bus = bus
        self.responses = defaultdict(deque)
        self.defaults = {0x68: MPU_DEFAULT, 0x0C: AK_DEFAULT, 0x76: BMP_DEFAULT}
        self.nack_next = defaultdict(int)
        self.stretch_next = False
        self.log = []
        self.accepted = []
        self.protocol_checks = 0
        self.bmp_mutation = None

    def queue(self, address, *responses):
        self.responses[address].extend(bytes(value) for value in responses)

    def response(self, address, count):
        value = (self.responses[address].popleft() if self.responses[address]
                 else self.defaults[address])
        assert len(value) == count
        return value

    async def _wait_start(self):
        while True:
            await FallingEdge(self.dut.sda_i)
            await Timer(1, unit="ps")
            if int(self.dut.scl_i.value):
                return

    async def _receive_byte(self, wait_for_low=True):
        if wait_for_low:
            await FallingEdge(self.dut.scl_i)
        value = 0
        for _ in range(8):
            await RisingEdge(self.dut.scl_i)
            await Timer(1, unit="ps")
            value = (value << 1) | int(self.dut.sda_i.value)
            await FallingEdge(self.dut.scl_i)
        return value

    async def _ack(self, ack=True):
        self.bus.target_sda_low = ack
        await RisingEdge(self.dut.scl_i)
        await FallingEdge(self.dut.scl_i)
        self.bus.target_sda_low = False

    async def _send_byte(self, value):
        for bit in range(7, -1, -1):
            self.bus.target_sda_low = not bool((value >> bit) & 1)
            await RisingEdge(self.dut.scl_i)
            await FallingEdge(self.dut.scl_i)
        self.bus.target_sda_low = False
        await RisingEdge(self.dut.scl_i)
        await Timer(1, unit="ps")
        master_ack = not bool(self.dut.sda_i.value)
        await FallingEdge(self.dut.scl_i)
        return master_ack

    async def _wait_stop(self):
        while True:
            await RisingEdge(self.dut.sda_i)
            await Timer(1, unit="ps")
            if int(self.dut.scl_i.value):
                return

    async def serve(self):
        while True:
            await RisingEdge(self.dut.clk)
            await Timer(1, unit="ps")
            if not int(self.dut.monitor_cmd_accepted.value):
                continue
            address = int(self.dut.monitor_cmd_address.value)
            writes = int(self.dut.monitor_cmd_write_count.value)
            reads = int(self.dut.monitor_cmd_read_count.value)
            fast = int(self.dut.monitor_cmd_fast_mode.value)
            shape = (address, writes, reads, fast)
            self.accepted.append(shape)
            assert writes == 1 and reads in (6, 8, 15) and fast == 1
            if self.stretch_next:
                self.stretch_next = False
                self.bus.target_scl_low = True
                await ClockCycles(self.dut.clk, 2100)
                self.bus.target_scl_low = False
                continue
            await self._wait_start()
            assert await self._receive_byte() == address << 1
            nack = self.nack_next[address] > 0
            if nack:
                self.nack_next[address] -= 1
            await self._ack(not nack)
            if nack:
                continue
            register = await self._receive_byte(wait_for_low=False)
            await self._ack(True)
            await self._wait_start()
            assert await self._receive_byte() == (address << 1) | 1
            await self._ack(True)
            payload = self.response(address, reads)
            acks = []
            for index, value in enumerate(payload):
                acks.append(await self._send_byte(value))
                if address == 0x76 and index == 2 and self.bmp_mutation is not None:
                    self.defaults[0x76] = self.bmp_mutation
            assert acks == [True] * (reads - 1) + [False]
            await self._wait_stop()
            expected_register = {0x68: 0x3A, 0x0C: 0x02, 0x76: 0xF7}[address]
            assert register == expected_register
            self.protocol_checks += 1
            self.log.append((address, register, payload, tuple(acks)))


async def timestamp_counter(dut):
    while True:
        await ClockCycles(dut.clk, 10)
        dut.timestamp_cycles.value = (int(dut.timestamp_cycles.value) + 10) & ((1 << 64) - 1)


async def start_environment(dut):
    dut.rst_n.value = 0
    dut.init_ready.value = 0
    dut.mpu_address.value = 0x68
    dut.bmp_address.value = 0x76
    dut.timestamp_cycles.value = 0
    for name in ("enable_1khz", "enable_100hz", "enable_50hz", "enable_10hz",
                 "snapshot_capture", "fifo_pop"):
        getattr(dut, name).value = 0
    clock = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    bus = OpenDrainBus(dut)
    resolver = cocotb.start_soon(bus.resolve())
    model = RuntimeGy91(dut, bus)
    server = cocotb.start_soon(model.serve())
    ticker = cocotb.start_soon(timestamp_counter(dut))
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    dut.init_ready.value = 1
    await ClockCycles(dut.clk, 2)
    return model, bus, [clock, resolver, server, ticker]


def stop(tasks):
    for task in tasks:
        task.cancel()


async def pulse(dut, *names):
    for name in names:
        getattr(dut, name).value = 1
    await RisingEdge(dut.clk)
    for name in names:
        getattr(dut, name).value = 0


async def wait_count(dut, signal_name, value, cycles=30000):
    for _ in range(cycles):
        if int(getattr(dut, signal_name).value) >= value:
            return
        await RisingEdge(dut.clk)
    raise AssertionError(f"timeout waiting for {signal_name} >= {value}")


def record(value):
    return {
        "type": value & 3,
        "flags": (value >> 2) & 0x3FFF,
        "sequence": (value >> 16) & 0xFFFFFFFF,
        "timestamp": (value >> 48) & 0xFFFFFFFFFFFFFFFF,
        "payload": (value >> 112) & ((1 << 112) - 1),
        "release_sequence": (value >> 224) & 0xFFFFFFFF,
    }


async def pop_all(dut):
    result = []
    while not int(dut.fifo_empty.value):
        result.append(int(dut.fifo_head_data.value))
        await pulse(dut, "fifo_pop")
        await RisingEdge(dut.clk)
    return result


@cocotb.test()
async def common_phase_order_protocol_and_record_packing(dut):
    model, _, tasks = await start_environment(dut)
    try:
        await pulse(dut, "enable_1khz", "enable_100hz", "enable_50hz")
        await wait_count(dut, "bmp_accepted_sample_count", 1)
        assert [item[:3] for item in model.accepted] == [
            (0x68, 1, 15), (0x0C, 1, 8), (0x76, 1, 6)]
        assert model.protocol_checks == 3
        records = await pop_all(dut)
        assert [record(item)["type"] for item in records] == [0, 1, 2]
        assert [record(item)["sequence"] for item in records] == [1, 1, 1]
        assert [record(item)["release_sequence"] for item in records] == [1, 1, 1]
        assert record(records[0])["payload"] == int.from_bytes(MPU_DEFAULT[1:], "little")
        assert record(records[0])["flags"] & 0x02
        assert record(records[1])["payload"] & ((1 << 48) - 1) == int.from_bytes(AK_DEFAULT[1:7], "little")
        assert (record(records[1])["flags"] & 0x1F) == 0x12
        assert record(records[0])["timestamp"] <= record(records[1])["timestamp"] <= record(records[2])["timestamp"]
    finally:
        stop(tasks)


@cocotb.test()
async def long_run_exact_rates_bandwidth_and_deadlines(dut):
    model, _, tasks = await start_environment(dut)
    async def releases():
        for period in range(101):
            names = ["enable_1khz"]
            if period % 10 == 0:
                names.append("enable_100hz")
            if period % 20 == 0:
                names.append("enable_50hz")
            await pulse(dut, *names)
            await ClockCycles(dut.clk, 9999)
            await pop_all(dut)
    try:
        await releases()
        await wait_count(dut, "mpu_accepted_sample_count", 101)
        await wait_count(dut, "bmp_accepted_sample_count", 6)
        assert int(dut.mpu_release_count.value) == 101
        assert int(dut.ak_release_count.value) == 11
        assert int(dut.bmp_release_count.value) == 6
        assert int(dut.mpu_accepted_sample_count.value) == 101
        assert int(dut.ak_accepted_sample_count.value) == 11
        assert int(dut.bmp_accepted_sample_count.value) == 6
        assert int(dut.deadline_miss_count.value) == 0
        assert int(dut.mpu_missed_release_count.value) == 0
        assert int(dut.mpu_max_latency_cycles.value) < 10000
        assert int(dut.bus_transaction_count.value) == len(model.accepted) == 118
        busy = int(dut.bus_busy_cycles.value)
        assert 300000 < busy < 600000
    finally:
        stop(tasks)


@cocotb.test()
async def mpu_retry_priority_and_double_clear_accounting(dut):
    model, _, tasks = await start_environment(dut)
    try:
        clear = bytes(15)
        model.queue(0x68, clear, MPU_DEFAULT)
        await pulse(dut, "enable_1khz", "enable_100hz", "enable_50hz")
        await wait_count(dut, "mpu_accepted_sample_count", 1)
        assert [x[0] for x in model.accepted[:2]] == [0x68, 0x68]
        assert int(dut.mpu_retry_count.value) == 1
        assert int(dut.mpu_duplicate_poll_count.value) == 1
        model.queue(0x68, clear, clear)
        await pulse(dut, "enable_1khz")
        await wait_count(dut, "mpu_invalid_sample_count", 1)
        assert int(dut.mpu_accepted_sample_count.value) == 1
        assert int(dut.mpu_retry_count.value) == 2
        assert int(dut.mpu_duplicate_poll_count.value) == 3
        assert int(dut.mpu_missed_sample_count.value) == 1
    finally:
        stop(tasks)


@cocotb.test()
async def ak_validity_flags_and_bmp_atomic_sentinels(dut):
    model, _, tasks = await start_environment(dut)
    try:
        model.queue(0x0C, bytes(8), bytes([3, 1, 0, 2, 0, 3, 0, 0x10]),
                    bytes([1, 1, 0, 2, 0, 3, 0, 0x18]),
                    bytes([1, 1, 0, 2, 0, 3, 0, 0x00]))
        for _ in range(4):
            await pulse(dut, "enable_100hz")
            await wait_count(dut, "ak_release_count", _ + 1)
            while int(dut.bus_transaction_count.value) < _ + 1:
                await RisingEdge(dut.clk)
            await ClockCycles(dut.clk, 3500)
        assert int(dut.ak_accepted_sample_count.value) == 1
        assert int(dut.ak_duplicate_poll_count.value) == 1
        assert int(dut.ak_overrun_count.value) == 1
        assert int(dut.ak_invalid_sample_count.value) == 3
        sentinel = bytes([0x80, 0, 0, 0x80, 0, 0])
        changed = bytes([0x12, 0x34, 0x50, 0x67, 0x89, 0xA0])
        model.queue(0x76, sentinel, BMP_DEFAULT)
        model.bmp_mutation = changed
        await pulse(dut, "enable_50hz")
        await wait_count(dut, "bmp_invalid_sample_count", 1)
        await pulse(dut, "enable_50hz")
        await wait_count(dut, "bmp_accepted_sample_count", 1)
        payload = record(int(dut.bmp_latest_record.value))["payload"] & ((1 << 48) - 1)
        assert payload == int.from_bytes(bytes([0x64, 0x32, 0x10, 0x54, 0x32, 0x10]), "little")
        await pulse(dut, "enable_50hz")
        await wait_count(dut, "bmp_accepted_sample_count", 2)
        assert (record(int(dut.bmp_latest_record.value))["payload"] & ((1 << 48) - 1)) == int.from_bytes(changed, "little")
    finally:
        stop(tasks)


@cocotb.test()
async def transport_nack_timeout_and_sticky_runtime_fault(dut):
    model, bus, tasks = await start_environment(dut)
    try:
        model.nack_next[0x68] = 3
        for failure in range(3):
            await pulse(dut, "enable_1khz")
            await wait_count(dut, "mpu_i2c_error_count", failure + 1)
        assert int(dut.runtime_fault.value) == 1
        assert int(dut.mpu_valid.value) == 0 and int(dut.fifo_level.value) == 0
        model.stretch_next = True
        await pulse(dut, "enable_100hz")
        await wait_count(dut, "ak_i2c_error_count", 1, 40000)
        assert int(dut.last_i2c_error.value) in (4, 5)
        assert int(dut.bus_error_count.value) == 4
        tasks[2].cancel()
        bus.target_scl_low = False
        bus.target_sda_low = False
        model = RuntimeGy91(dut, bus)
        tasks[2] = cocotb.start_soon(model.serve())
        await pulse(dut, "enable_1khz")
        await wait_count(dut, "mpu_accepted_sample_count", 1)
        assert int(dut.runtime_fault.value) == 1
    finally:
        stop(tasks)


@cocotb.test()
async def pending_coalescing_and_priority_after_lower_work(dut):
    model, _, tasks = await start_environment(dut)
    try:
        await pulse(dut, "enable_50hz")
        await wait_count(dut, "bus_transaction_count", 1)
        await pulse(dut, "enable_50hz", "enable_1khz")
        await pulse(dut, "enable_50hz", "enable_1khz")
        await wait_count(dut, "mpu_accepted_sample_count", 1)
        assert [item[0] for item in model.accepted[:2]] == [0x76, 0x68]
        assert int(dut.bmp_missed_release_count.value) == 2
        assert int(dut.mpu_missed_release_count.value) == 1
        assert int(dut.bmp_missed_sample_count.value) == 2
    finally:
        stop(tasks)


@cocotb.test()
async def fifo_wrap_drop_newest_underflow_and_latest_progress(dut):
    _, _, tasks = await start_environment(dut)
    try:
        for index in range(6):
            await pulse(dut, "enable_1khz")
            await wait_count(dut, "mpu_accepted_sample_count", index + 1)
        await ClockCycles(dut.clk, 3)
        assert int(dut.fifo_level.value) == 4 and int(dut.fifo_full.value)
        assert int(dut.fifo_overflow_count.value) == 2
        assert int(dut.fifo_overflow_sticky.value)
        assert record(int(dut.mpu_latest_record.value))["sequence"] == 6
        records = await pop_all(dut)
        assert [record(item)["sequence"] for item in records] == [1, 2, 3, 4]
        await pulse(dut, "fifo_pop")
        await ClockCycles(dut.clk, 2)
        assert int(dut.fifo_underflow_count.value) == 1
        for index in range(4):
            await pulse(dut, "enable_1khz")
            await wait_count(dut, "mpu_accepted_sample_count", 7 + index)
            await ClockCycles(dut.clk, 2)
            await pulse(dut, "fifo_pop")
        await RisingEdge(dut.clk)
        assert int(dut.fifo_level.value) == 0
    finally:
        stop(tasks)


@cocotb.test()
async def snapshot_atomicity_with_live_fifo_activity(dut):
    _, _, tasks = await start_environment(dut)
    try:
        await pulse(dut, "enable_1khz", "enable_100hz", "enable_50hz")
        await wait_count(dut, "bmp_accepted_sample_count", 1)
        await pulse(dut, "snapshot_capture")
        await RisingEdge(dut.clk)
        captured = (int(dut.snapshot_records.value), int(dut.snapshot_status.value),
                    int(dut.snapshot_sequence.value))
        await pulse(dut, "fifo_pop", "enable_1khz")
        await wait_count(dut, "mpu_accepted_sample_count", 2)
        assert (int(dut.snapshot_records.value), int(dut.snapshot_status.value),
                int(dut.snapshot_sequence.value)) == captured
        assert record(int(dut.mpu_latest_record.value))["sequence"] == 2
        await pulse(dut, "snapshot_capture")
        await RisingEdge(dut.clk)
        assert int(dut.snapshot_sequence.value) == captured[2] + 1
        assert int(dut.snapshot_records.value) != captured[0]
    finally:
        stop(tasks)


@cocotb.test()
async def freshness_hard_stale_wrap_and_init_drop(dut):
    _, _, tasks = await start_environment(dut)
    try:
        tasks[3].cancel()
        dut.timestamp_cycles.value = 50001
        await ClockCycles(dut.clk, 2)
        assert int(dut.mpu_hard_stale.value) and int(dut.runtime_fault.value)
        dut.init_ready.value = 0
        await ClockCycles(dut.clk, 2)
        assert int(dut.fifo_empty.value) and not int(dut.mpu_valid.value)
        assert not int(dut.runtime_fault.value) and not int(dut.scl_drive_low.value)
        dut.timestamp_cycles.value = (1 << 64) - 100
        dut.init_ready.value = 1
        await pulse(dut, "enable_1khz")
        await wait_count(dut, "mpu_accepted_sample_count", 1)
        assert int(dut.mpu_fresh.value)
        completion = (int(dut.mpu_latest_record.value) >> 48) & ((1 << 64) - 1)
        dut.timestamp_cycles.value = (completion + 20001) & ((1 << 64) - 1)
        await ClockCycles(dut.clk, 2)
        assert not int(dut.mpu_fresh.value) and not int(dut.mpu_hard_stale.value)
        dut.timestamp_cycles.value = (completion + 50001) & ((1 << 64) - 1)
        await ClockCycles(dut.clk, 2)
        assert int(dut.mpu_hard_stale.value)
    finally:
        stop(tasks)


@cocotb.test()
async def reset_during_bus_retry_and_fifo_activity_recovers(dut):
    model, bus, tasks = await start_environment(dut)
    try:
        model.queue(0x68, bytes(15))
        await pulse(dut, "enable_1khz")
        await wait_count(dut, "mpu_retry_count", 1)
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 2)
        assert not int(dut.scl_drive_low.value) and not int(dut.sda_drive_low.value)
        assert int(dut.fifo_empty.value) and int(dut.bus_transaction_count.value) == 0
        tasks[2].cancel()
        bus.target_sda_low = False
        bus.target_scl_low = False
        model = RuntimeGy91(dut, bus)
        tasks[2] = cocotb.start_soon(model.serve())
        dut.rst_n.value = 1
        await pulse(dut, "enable_1khz")
        await wait_count(dut, "mpu_accepted_sample_count", 1)
        await ClockCycles(dut.clk, 3)
        assert int(dut.mpu_valid.value) and int(dut.fifo_level.value) == 1
    finally:
        stop(tasks)
