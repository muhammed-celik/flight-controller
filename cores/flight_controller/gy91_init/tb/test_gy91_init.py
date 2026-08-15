import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer, with_timeout
from cocotb.triggers import SimTimeoutError


CLK_NS = 10
CALIBRATION = bytes.fromhex(
    "706b436718fc7d8e43d6d00b270b8c00f9ff8c3c70c61700"
)
ASA = bytes([0x82, 0x7D, 0x91])


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


class Gy91Devices:
    """Register-accurate targets driven exclusively from resolved bus edges."""

    def __init__(self, dut, bus, mpu_address=0x68, bmp_address=0x76):
        self.dut = dut
        self.bus = bus
        self.mpu_address = mpu_address
        self.bmp_address = bmp_address
        self.mpu_addresses = {mpu_address}
        self.bmp_addresses = {bmp_address}
        self.mpu_id = 0x71
        self.ak_id = 0x48
        self.bmp_id = 0x58
        self.bmp_post_id = None
        self.asa = ASA
        self.calibration = CALIBRATION
        self.bmp_busy_reads = 2
        self.bad_mpu_readback = False
        self.bad_ak_readback = False
        self.bad_bmp_config_readback = False
        self.bad_bmp_ctrl_readback = False
        self.nack_once_at = None
        self.nack_fired = False
        self.log = []
        self.nacks = []
        self.fast_modes = []
        self.mpu = {0x75: self.mpu_id}
        self.ak = {0x00: self.ak_id, 0x10: self.asa[0], 0x11: self.asa[1], 0x12: self.asa[2]}
        self.bmp = {0xD0: self.bmp_id}
        self._bmp_status_reads = 0

    def repair(self):
        self.mpu_id = 0x71
        self.ak_id = 0x48
        self.bmp_id = 0x58
        self.bmp_post_id = None
        self.asa = ASA
        self.calibration = CALIBRATION
        self.bmp_busy_reads = 2
        self.bad_mpu_readback = False
        self.bad_ak_readback = False
        self.bad_bmp_config_readback = False
        self.bad_bmp_ctrl_readback = False
        self.nack_once_at = None
        self.nack_fired = False
        self.mpu = {0x75: self.mpu_id}
        self.ak = {0x00: self.ak_id, 0x10: self.asa[0], 0x11: self.asa[1], 0x12: self.asa[2]}
        self.bmp = {0xD0: self.bmp_id}
        self._bmp_status_reads = 0

    async def _wait_start(self):
        while True:
            await FallingEdge(self.dut.sda_i)
            await Timer(1, unit="ns")
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
        await FallingEdge(self.dut.scl_i)

    async def _wait_stop(self):
        while True:
            await RisingEdge(self.dut.sda_i)
            await Timer(1, unit="ns")
            if int(self.dut.scl_i.value):
                return

    def _present(self, address):
        if address in self.mpu_addresses:
            return self.mpu_id is not None
        if address == 0x0C:
            return self.ak_id is not None and bool(self.mpu.get(0x37, 0) & 0x02)
        if address in self.bmp_addresses:
            return self.bmp_id is not None
        return False

    def _read(self, address, register, count):
        if address in self.mpu_addresses:
            values = [self.mpu_id if register + i == 0x75 else self.mpu.get(register + i, 0) for i in range(count)]
            if self.bad_mpu_readback and register == 0x1A:
                values[0] ^= 0x02
            return values
        if address == 0x0C:
            if register == 0x00:
                return [self.ak_id]
            if register == 0x10:
                return list(self.asa[:count])
            value = self.ak.get(register, 0)
            if self.bad_ak_readback and register == 0x0A:
                value ^= 0x10
            return [value]
        if register == 0xD0:
            return [self.bmp_id]
        if register == 0xF3:
            busy = self._bmp_status_reads < self.bmp_busy_reads
            self._bmp_status_reads += 1
            return [0x09 if busy else 0x00]
        if register == 0x88:
            return list(self.calibration[:count])
        value = self.bmp.get(register, 0)
        if register == 0xF5:
            value |= 0x02  # Reserved readback bit is intentionally nonzero.
            if self.bad_bmp_config_readback:
                value ^= 0x08
        if register == 0xF4 and self.bad_bmp_ctrl_readback:
            value ^= 0x01
        return [value]

    def _write(self, address, register, value):
        if address in self.mpu_addresses:
            if register == 0x6B and value == 0x80:
                self.mpu = {0x75: self.mpu_id}
            else:
                self.mpu[register] = value
            return
        if address == 0x0C:
            if register == 0x0B and value == 0x01:
                self.ak = {0x00: self.ak_id, 0x10: self.asa[0], 0x11: self.asa[1], 0x12: self.asa[2]}
            else:
                self.ak[register] = value
            return
        if register == 0xE0 and value == 0xB6:
            if self.bmp_post_id is not None:
                self.bmp_id = self.bmp_post_id
            self.bmp = {0xD0: self.bmp_id}
            self._bmp_status_reads = 0
        else:
            self.bmp[register] = value

    async def serve(self):
        while True:
            await RisingEdge(self.dut.clk)
            await Timer(1, unit="ps")
            if not int(self.dut.monitor_cmd_accepted.value):
                continue
            address = int(self.dut.monitor_cmd_address.value)
            write_count = int(self.dut.monitor_cmd_write_count.value)
            read_count = int(self.dut.monitor_cmd_read_count.value)
            self.fast_modes.append(int(self.dut.monitor_cmd_fast_mode.value))
            await self._wait_start()
            observed_address = await self._receive_byte()
            assert observed_address == address << 1
            inject = self.nack_once_at == len(self.log) and not self.nack_fired
            present = self._present(address) and not inject
            await self._ack(present)
            if not present:
                if inject:
                    self.nack_fired = True
                self.nacks.append((address, write_count, read_count))
                continue
            writes = []
            for _ in range(write_count):
                writes.append(await self._receive_byte(wait_for_low=False))
                await self._ack(True)
            assert writes, "register transactions always contain a register byte"
            register = writes[0]
            assert len(writes) in (1, 2)
            if len(writes) == 2:
                self._write(address, register, writes[1])
            reads = []
            if read_count:
                await self._wait_start()
                read_address = await self._receive_byte()
                assert read_address == (address << 1) | 1
                await self._ack(True)
                reads = self._read(address, register, read_count)
                assert len(reads) == read_count
                for value in reads:
                    await self._send_byte(value)
            await self._wait_stop()
            self.log.append((address, tuple(writes), tuple(reads)))


def expected_transactions(mpu=0x68, bmp=0x76, busy_reads=2):
    result = [
        (mpu, (0x75,), (0x71,)), (mpu, (0x6B, 0x80), ()),
        (mpu, (0x75,), (0x71,)), (mpu, (0x6B, 0x01), ()),
        (mpu, (0x6C, 0x00), ()),
    ]
    configs = [(0x19, 0x00), (0x1A, 0x02), (0x1B, 0x18), (0x1C, 0x10),
               (0x1D, 0x02), (0x6A, 0x00), (0x37, 0x02)]
    result += [(mpu, pair, ()) for pair in configs]
    verify = [(0x75, 0x71), (0x6B, 0x01), (0x6C, 0x00)] + configs
    result += [(mpu, (register,), (value,)) for register, value in verify]
    result += [
        (0x0C, (0x0B, 0x01), ()), (0x0C, (0x00,), (0x48,)),
        (0x0C, (0x0A, 0x00), ()), (0x0C, (0x0A, 0x0F), ()),
        (0x0C, (0x10,), tuple(ASA)), (0x0C, (0x0A, 0x00), ()),
        (0x0C, (0x0A, 0x16), ()), (0x0C, (0x0A,), (0x16,)),
        (bmp, (0xD0,), (0x58,)), (bmp, (0xE0, 0xB6), ()),
    ]
    result += [(bmp, (0xF3,), (0x09,)) for _ in range(busy_reads)]
    result += [
        (bmp, (0xF3,), (0x00,)), (bmp, (0xD0,), (0x58,)),
        (bmp, (0x88,), tuple(CALIBRATION)), (bmp, (0xF4, 0x2C), ()),
        (bmp, (0xF5, 0x08), ()), (bmp, (0xF5,), (0x0A,)),
        (bmp, (0xF4, 0x2F), ()), (bmp, (0xF4,), (0x2F,)),
    ]
    return result


async def start_environment(dut, **kwargs):
    dut.reinitialize.value = 0
    dut.rst_n.value = 0
    clock = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    bus = OpenDrainBus(dut)
    resolver = cocotb.start_soon(bus.resolve())
    devices = Gy91Devices(dut, bus, **kwargs)
    server = cocotb.start_soon(devices.serve())
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value = 1
    return devices, [clock, resolver, server]


async def wait_complete(dut, timeout_us=1000):
    async def waiter():
        while int(dut.initializing.value):
            assert not int(dut.ready.value)
            await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
    try:
        await with_timeout(waiter(), timeout_us, "us")
    except SimTimeoutError:
        dut._log.error(
            "timeout: init=%s failures=%s/%s/%s errors=%s/%s/%s lines=%s/%s drive=%s/%s",
            dut.initializing.value, dut.mpu_failed.value, dut.ak_failed.value,
            dut.bmp_failed.value, dut.mpu_error_count.value,
            dut.ak_error_count.value, dut.bmp_error_count.value,
            dut.scl_i.value, dut.sda_i.value, dut.scl_drive_low.value,
            dut.sda_drive_low.value,
        )
        raise


async def reinitialize(dut):
    assert int(dut.reinitialize_ready.value)
    dut.reinitialize.value = 1
    await RisingEdge(dut.clk)
    await Timer(1, unit="ps")
    dut.reinitialize.value = 0
    assert int(dut.initializing.value) and not int(dut.ready.value)


def stop(tasks):
    for task in tasks:
        task.cancel()


@cocotb.test()
async def nominal_exact_sequence_status_and_packing(dut):
    devices, tasks = await start_environment(dut)
    try:
        await wait_complete(dut)
        assert int(dut.ready.value) and not int(dut.failed.value)
        assert all(int(getattr(dut, name).value) for name in ("mpu_present", "mpu_ready", "ak_present", "ak_ready", "bmp_present", "bmp_ready"))
        assert devices.log == expected_transactions()
        assert not any(devices.fast_modes)
        assert int(dut.mpu_address.value) == 0x68 and int(dut.bmp_address.value) == 0x76
        assert int(dut.ak_asa.value) == int.from_bytes(ASA, "little")
        assert int(dut.bmp_calibration.value) == int.from_bytes(CALIBRATION, "little")
        assert int(dut.init_sequence.value) == 1
    finally:
        stop(tasks)


@cocotb.test()
async def alternate_addresses(dut):
    devices, tasks = await start_environment(dut, mpu_address=0x69, bmp_address=0x77)
    try:
        await wait_complete(dut)
        assert int(dut.ready.value)
        assert int(dut.mpu_address.value) == 0x69 and int(dut.bmp_address.value) == 0x77
        assert devices.log == expected_transactions(0x69, 0x77)
        assert devices.nacks[:6] == [(0x68, 1, 1)] * 3 + [(0x76, 1, 1)] * 3
    finally:
        stop(tasks)


@cocotb.test()
async def identities_and_mpu_dependency(dut):
    devices, tasks = await start_environment(dut)
    try:
        devices.mpu_id = None
        devices.mpu = {}
        await wait_complete(dut)
        assert int(dut.mpu_failed.value) and int(dut.ak_failed.value)
        assert int(dut.bmp_ready.value) and int(dut.failed.value)
        assert not any(entry[0] == 0x0C for entry in devices.log)
        assert any(entry[0] == 0x76 for entry in devices.log)
        assert int(dut.last_failed_device.value) == 1
    finally:
        stop(tasks)


@cocotb.test()
async def wrong_probe_identities_try_both_candidates(dut):
    devices, tasks = await start_environment(dut)
    try:
        devices.mpu_id = 0x70
        devices.mpu[0x75] = 0x70
        devices.mpu_addresses = {0x68, 0x69}
        devices.bmp_id = 0x57
        devices.bmp[0xD0] = 0x57
        devices.bmp_addresses = {0x76, 0x77}
        await wait_complete(dut)
        assert int(dut.mpu_failed.value) and int(dut.ak_failed.value) and int(dut.bmp_failed.value)
        probes = [(address, writes[0]) for address, writes, _ in devices.log]
        assert probes == [(0x68, 0x75), (0x69, 0x75), (0x76, 0xD0), (0x77, 0xD0)]
        assert int(dut.mpu_error_count.value) == 2
        assert int(dut.bmp_error_count.value) == 2
    finally:
        stop(tasks)


@cocotb.test()
async def ak_identity_asa_and_readback_failures(dut):
    devices, tasks = await start_environment(dut)
    try:
        cases = [("ak_id", None), ("ak_id", 0x47), ("asa", bytes(3)),
                 ("asa", bytes([0xFF] * 3)), ("bad_ak_readback", True)]
        expected_errors = 0
        for index, (field, value) in enumerate(cases, 1):
            setattr(devices, field, value)
            if field == "ak_id":
                devices.ak[0x00] = value or 0
            await wait_complete(dut)
            assert int(dut.mpu_ready.value) and int(dut.ak_failed.value) and int(dut.bmp_ready.value)
            expected_errors += 3 if value is None else 1
            assert int(dut.failed.value) and int(dut.ak_error_count.value) == expected_errors
            devices.repair()
            devices.log.clear()
            await reinitialize(dut)
        await wait_complete(dut)
        assert int(dut.ready.value)
    finally:
        stop(tasks)


@cocotb.test()
async def bmp_poll_identity_calibration_and_readback_failures(dut):
    devices, tasks = await start_environment(dut)
    try:
        cases = [
            ("bmp_busy_reads", 4), ("bmp_post_id", 0x57),
            ("calibration", bytes(24)), ("calibration", bytes([0xFF] * 24)),
            ("calibration", bytes(2) + CALIBRATION[2:]),
            ("calibration", CALIBRATION[:6] + bytes(2) + CALIBRATION[8:]),
            ("bad_bmp_config_readback", True), ("bad_bmp_ctrl_readback", True),
        ]
        for index, (field, value) in enumerate(cases, 1):
            setattr(devices, field, value)
            await wait_complete(dut)
            assert int(dut.mpu_ready.value) and int(dut.ak_ready.value)
            assert int(dut.bmp_failed.value) and int(dut.failed.value)
            assert int(dut.bmp_error_count.value) == index
            devices.repair()
            devices.log.clear()
            await reinitialize(dut)
        await wait_complete(dut)
        assert int(dut.ready.value)
    finally:
        stop(tasks)


@cocotb.test()
async def transient_nack_retries_every_nominal_transaction(dut):
    devices, tasks = await start_environment(dut)
    try:
        nominal = expected_transactions()
        for position in range(len(nominal)):
            devices.nack_once_at = position
            devices.nack_fired = False
            await wait_complete(dut)
            assert int(dut.ready.value), f"failed after transient NACK at {position}"
            assert devices.nack_fired
            expected_shape = (nominal[position][0], len(nominal[position][1]), len(nominal[position][2]))
            assert devices.nacks[-1] == expected_shape
            assert devices.log == nominal
            devices.repair()
            devices.log.clear()
            await reinitialize(dut)
        await wait_complete(dut)
        assert int(dut.ready.value)
    finally:
        stop(tasks)


@cocotb.test()
async def retry_exhaustion_per_device_continues_or_terminates(dut):
    devices, tasks = await start_environment(dut)
    try:
        devices.mpu_id = None
        devices.mpu = {}
        await wait_complete(dut)
        assert int(dut.mpu_error_count.value) == 6 and int(dut.bmp_ready.value)
        devices.repair(); devices.ak_id = None; devices.ak[0x00] = 0
        await reinitialize(dut); await wait_complete(dut)
        assert int(dut.ak_error_count.value) == 3 and int(dut.bmp_ready.value)
        devices.repair(); devices.bmp_id = None; devices.bmp = {}
        await reinitialize(dut); await wait_complete(dut)
        assert int(dut.bmp_error_count.value) == 6 and int(dut.bmp_failed.value)
    finally:
        stop(tasks)


@cocotb.test()
async def failed_run_reinitialize_repairs_and_preserves_counters(dut):
    devices, tasks = await start_environment(dut)
    try:
        devices.bad_ak_readback = True
        await wait_complete(dut)
        assert int(dut.failed.value) and int(dut.init_sequence.value) == 1
        errors = int(dut.ak_error_count.value)
        devices.repair(); devices.log.clear()
        await reinitialize(dut)
        assert not int(dut.ak_ready.value) and int(dut.ak_error_count.value) == errors
        await wait_complete(dut)
        assert int(dut.ready.value) and int(dut.init_sequence.value) == 2
        assert int(dut.ak_error_count.value) == errors
    finally:
        stop(tasks)


@cocotb.test()
async def reset_during_delay_and_active_transaction_restarts_safely(dut):
    devices, tasks = await start_environment(dut)
    try:
        await ClockCycles(dut.clk, 2)
        dut.rst_n.value = 0
        await RisingEdge(dut.clk)
        assert not int(dut.scl_drive_low.value) and not int(dut.sda_drive_low.value)
        dut.rst_n.value = 1
        while not int(dut.monitor_cmd_accepted.value):
            await RisingEdge(dut.clk)
        await FallingEdge(dut.scl_i)
        dut.rst_n.value = 0
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert not int(dut.scl_drive_low.value) and not int(dut.sda_drive_low.value)
        tasks[-1].cancel()
        bus = devices.bus
        bus.target_sda_low = False
        devices = Gy91Devices(dut, bus)
        tasks[-1] = cocotb.start_soon(devices.serve())
        dut.rst_n.value = 1
        await wait_complete(dut)
        assert int(dut.ready.value) and int(dut.init_sequence.value) == 1
    finally:
        stop(tasks)
