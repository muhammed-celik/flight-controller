import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout
from cocotb.utils import get_sim_time


SYS_CLK_NS = 10
TRANSACTION_TIMEOUT_MS = 3
DEVICE_ADDRESS = 0x68


async def reset_dut(dut):
    dut.i_rstn.value = 0
    dut.i_en.value = 0
    dut.i_dev_addr.value = 0
    dut.i_rw.value = 0
    dut.i_data.value = 0
    dut.slave_scl_low.value = 0
    dut.slave_sda_low.value = 0
    await Timer(5 * SYS_CLK_NS, unit="ns")
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)
    assert int(dut.scl.value) == 1
    assert int(dut.sda.value) == 1


async def wait_for_stop(dut):
    while True:
        await with_timeout(RisingEdge(dut.sda), TRANSACTION_TIMEOUT_MS, "ms")
        await ReadOnly()
        if int(dut.scl.value) == 1:
            return


class I2cSlave:
    """Minimal open-drain I2C slave used to verify the master bus behavior."""

    def __init__(self, dut, address=DEVICE_ADDRESS):
        self.dut = dut
        self.address = address

    async def wait_for_start(self):
        while True:
            await FallingEdge(self.dut.sda)
            await ReadOnly()
            if int(self.dut.scl.value) == 1:
                return

    async def receive_byte(self):
        value = 0
        for _ in range(8):
            await RisingEdge(self.dut.scl)
            await ReadOnly()
            value = (value << 1) | int(self.dut.sda.value)
        return value

    async def acknowledge(self, ack=True):
        await FallingEdge(self.dut.scl)
        self.dut.slave_sda_low.value = int(ack)
        await RisingEdge(self.dut.scl)
        await FallingEdge(self.dut.scl)
        self.dut.slave_sda_low.value = 0

    async def receive_address(self, ack=True):
        await self.wait_for_start()
        address_rw = await self.receive_byte()
        await self.acknowledge(ack)
        return address_rw >> 1, address_rw & 1

    async def receive_write(self, byte_count, nack_byte=None, address_ack=True):
        address, rw = await self.receive_address(address_ack)
        assert address == self.address
        assert rw == 0
        if not address_ack:
            await wait_for_stop(self.dut)
            return []

        received = []
        for index in range(byte_count):
            received.append(await self.receive_byte())
            await self.acknowledge(index != nack_byte)
            if index == nack_byte:
                break

        await wait_for_stop(self.dut)
        return received

    async def send_byte(self, value):
        for bit_index in range(7, -1, -1):
            self.dut.slave_sda_low.value = int(((value >> bit_index) & 1) == 0)
            await RisingEdge(self.dut.scl)
            await FallingEdge(self.dut.scl)

        self.dut.slave_sda_low.value = 0
        await RisingEdge(self.dut.scl)
        await ReadOnly()
        master_ack = int(self.dut.sda.value) == 0
        await FallingEdge(self.dut.scl)
        return master_ack

    async def provide_read(self, values, address_ack=True):
        address, rw = await self.receive_address(address_ack)
        assert address == self.address
        assert rw == 1
        if not address_ack:
            await wait_for_stop(self.dut)
            return []

        master_acks = []
        for value in values:
            master_acks.append(await self.send_byte(value))

        await wait_for_stop(self.dut)
        return master_acks


async def start_operation(dut, rw, data=0, address=DEVICE_ADDRESS):
    dut.i_dev_addr.value = address
    dut.i_rw.value = rw
    dut.i_data.value = data
    dut.i_en.value = 1
    await RisingEdge(dut.i_clk)


async def wait_done(dut):
    await with_timeout(RisingEdge(dut.o_done), TRANSACTION_TIMEOUT_MS, "ms")
    await ReadOnly()
    value = int(dut.o_data.value)
    error = int(dut.o_error.value)
    await Timer(1, unit="ns")
    return value, error


async def stretch_next_low_period(dut, duration_us):
    await FallingEdge(dut.scl)
    started = get_sim_time(unit="us")
    dut.slave_scl_low.value = 1
    await Timer(duration_us, unit="us")
    dut.slave_scl_low.value = 0
    return get_sim_time(unit="us") - started


@cocotb.test()
async def single_write(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    slave_task = cocotb.start_soon(I2cSlave(dut).receive_write(1))
    await start_operation(dut, rw=0, data=0x75)
    _, error = await wait_done(dut)
    dut.i_en.value = 0

    assert error == 0
    assert await with_timeout(slave_task, TRANSACTION_TIMEOUT_MS, "ms") == [0x75]


@cocotb.test()
async def single_read(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    slave_task = cocotb.start_soon(I2cSlave(dut).provide_read([0xA6]))
    await start_operation(dut, rw=1)
    value, error = await wait_done(dut)
    dut.i_en.value = 0

    master_acks = await with_timeout(slave_task, TRANSACTION_TIMEOUT_MS, "ms")
    assert value == 0xA6
    assert error == 0
    assert master_acks == [False], "master must NACK the final read byte"


@cocotb.test()
async def burst_write(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    values = [0x10, 0xDE, 0xAD, 0xBE]
    slave_task = cocotb.start_soon(I2cSlave(dut).receive_write(len(values)))
    await start_operation(dut, rw=0, data=values[0])

    for index in range(len(values)):
        _, error = await wait_done(dut)
        assert error == 0
        if index + 1 < len(values):
            dut.i_data.value = values[index + 1]
        else:
            dut.i_en.value = 0

    assert await with_timeout(slave_task, TRANSACTION_TIMEOUT_MS, "ms") == values


@cocotb.test()
async def burst_read(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    values = [0x12, 0x34, 0x56, 0x78]
    slave_task = cocotb.start_soon(I2cSlave(dut).provide_read(values))
    await start_operation(dut, rw=1)
    received = []

    for index in range(len(values)):
        value, error = await wait_done(dut)
        received.append(value)
        assert error == 0
        if index + 1 == len(values):
            dut.i_en.value = 0

    master_acks = await with_timeout(slave_task, TRANSACTION_TIMEOUT_MS, "ms")
    assert received == values
    assert master_acks == [True, True, True, False]


@cocotb.test()
async def nack_address_and_write_data(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    address_nack = cocotb.start_soon(
        I2cSlave(dut).receive_write(0, address_ack=False)
    )
    await start_operation(dut, rw=0, data=0x11)
    _, error = await wait_done(dut)
    dut.i_en.value = 0
    assert error == 1, "address NACK must raise o_error"
    assert await with_timeout(address_nack, TRANSACTION_TIMEOUT_MS, "ms") == []
    await Timer(1, unit="ns")

    data_nack = cocotb.start_soon(I2cSlave(dut).receive_write(1, nack_byte=0))
    await start_operation(dut, rw=0, data=0x5A)
    _, error = await wait_done(dut)
    dut.i_en.value = 0
    assert error == 1, "data NACK must raise o_error"
    assert await with_timeout(data_nack, TRANSACTION_TIMEOUT_MS, "ms") == [0x5A]


@cocotb.test()
async def clock_stretching(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    slave_task = cocotb.start_soon(I2cSlave(dut).provide_read([0xC3]))
    stretch_task = cocotb.start_soon(stretch_next_low_period(dut, duration_us=30))
    await start_operation(dut, rw=1)
    value, error = await wait_done(dut)
    dut.i_en.value = 0

    stretched_for = await with_timeout(stretch_task, TRANSACTION_TIMEOUT_MS, "ms")
    master_acks = await with_timeout(slave_task, TRANSACTION_TIMEOUT_MS, "ms")
    assert stretched_for >= 30
    assert value == 0xC3
    assert error == 0
    assert master_acks == [False]
