import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout


SYS_CLK_NS = 10
TRANSACTION_TIMEOUT_MS = 12
DEVICE_ADDRESS = 0x68


async def reset_dut(dut):
    dut.i_rstn.value = 0
    dut.i_cmd_valid.value = 0
    dut.i_cmd_type.value = 0
    dut.i_cmd_nbytes.value = 0
    dut.i_cmd_dev_addr.value = 0
    dut.i_cmd_reg_addr.value = 0
    dut.i_data_valid.value = 0
    dut.i_data.value = 0
    dut.slave_scl_low.value = 0
    dut.slave_sda_low.value = 0
    await Timer(5 * SYS_CLK_NS, unit="ns")
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)


async def issue_command(dut, read, register, byte_count, address=DEVICE_ADDRESS):
    assert 1 <= byte_count <= 32
    if not int(dut.o_cmd_ready.value):
        await with_timeout(RisingEdge(dut.o_cmd_ready), TRANSACTION_TIMEOUT_MS, "ms")
    await FallingEdge(dut.i_clk)
    dut.i_cmd_type.value = read
    dut.i_cmd_nbytes.value = byte_count - 1
    dut.i_cmd_dev_addr.value = address
    dut.i_cmd_reg_addr.value = register
    dut.i_cmd_valid.value = 1
    await RisingEdge(dut.i_clk)
    await FallingEdge(dut.i_clk)
    dut.i_cmd_valid.value = 0


async def wait_for_stop(dut):
    while True:
        await with_timeout(RisingEdge(dut.sda), TRANSACTION_TIMEOUT_MS, "ms")
        await ReadOnly()
        if int(dut.scl.value) == 1:
            return


class I2cSlave:
    def __init__(self, dut, address=DEVICE_ADDRESS):
        self.dut = dut
        self.address = address

    async def wait_for_start(self):
        while True:
            await with_timeout(FallingEdge(self.dut.sda), TRANSACTION_TIMEOUT_MS, "ms")
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

    async def receive_address(self, ack=True, expected_rw=None):
        await self.wait_for_start()
        address_rw = await self.receive_byte()
        await self.acknowledge(ack)
        address = address_rw >> 1
        rw = address_rw & 1
        assert address == self.address
        if expected_rw is not None:
            assert rw == expected_rw
        return rw

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

    async def register_read(self, values):
        await self.receive_address(expected_rw=0)
        register = await self.receive_byte()
        await self.acknowledge()
        await self.receive_address(expected_rw=1)

        master_acks = []
        for value in values:
            master_acks.append(await self.send_byte(value))
        await wait_for_stop(self.dut)
        return register, master_acks

    async def register_write(self, byte_count):
        await self.receive_address(expected_rw=0)
        register = await self.receive_byte()
        await self.acknowledge()

        values = []
        for _ in range(byte_count):
            values.append(await self.receive_byte())
            await self.acknowledge()
        await wait_for_stop(self.dut)
        return register, values


async def provide_write_data(dut, values):
    """Supply each byte immediately when the controller enters its wait state."""
    for value in values:
        await RisingEdge(dut.o_i2c_done)
        await RisingEdge(dut.i_clk)
        await FallingEdge(dut.i_clk)
        dut.i_data.value = value
        dut.i_data_valid.value = 1
        await RisingEdge(dut.i_clk)
        await FallingEdge(dut.i_clk)
        dut.i_data_valid.value = 0


async def collect_read_data(dut, byte_count):
    values = []
    for _ in range(byte_count):
        await with_timeout(RisingEdge(dut.o_data_valid), TRANSACTION_TIMEOUT_MS, "ms")
        await ReadOnly()
        values.append(int(dut.o_data.value))
    return values


async def wait_until_ready(dut):
    if not int(dut.o_cmd_ready.value):
        await with_timeout(RisingEdge(dut.o_cmd_ready), TRANSACTION_TIMEOUT_MS, "ms")
    await Timer(1, unit="ns")
    await ReadOnly()


async def wait_for_command_error(dut):
    await RisingEdge(dut.o_cmd_error)


@cocotb.test()
async def reset_sets_idle_interface(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)
    await ReadOnly()

    assert int(dut.o_cmd_ready.value) == 1
    assert int(dut.o_cmd_error.value) == 0
    assert int(dut.o_data_valid.value) == 0
    assert int(dut.o_data.value) == 0
    assert int(dut.scl.value) == 1
    assert int(dut.sda.value) == 1


@cocotb.test()
async def reads_single_and_multiple_bytes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for register, expected in ((0x75, [0xA6]), (0x20, [0x12, 0x34, 0x56])):
        slave_task = cocotb.start_soon(I2cSlave(dut).register_read(expected))
        data_task = cocotb.start_soon(collect_read_data(dut, len(expected)))
        await issue_command(dut, read=1, register=register, byte_count=len(expected))

        observed_register, master_acks = await with_timeout(
            slave_task, TRANSACTION_TIMEOUT_MS, "ms"
        )
        observed_data = await with_timeout(data_task, TRANSACTION_TIMEOUT_MS, "ms")
        await wait_until_ready(dut)

        assert observed_register == register
        assert observed_data == expected
        assert master_acks == ([True] * (len(expected) - 1)) + [False]
        assert int(dut.o_cmd_error.value) == 0


@cocotb.test()
async def writes_single_and_multiple_bytes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for register, expected in ((0x10, [0xC3]), (0x30, [0xDE, 0xAD, 0xBE])):
        slave_task = cocotb.start_soon(I2cSlave(dut).register_write(len(expected)))
        data_task = cocotb.start_soon(provide_write_data(dut, expected))
        await issue_command(dut, read=0, register=register, byte_count=len(expected))

        observed_register, observed_data = await with_timeout(
            slave_task, TRANSACTION_TIMEOUT_MS, "ms"
        )
        await with_timeout(data_task, TRANSACTION_TIMEOUT_MS, "ms")
        await wait_until_ready(dut)

        assert observed_register == register
        assert observed_data == expected
        assert int(dut.o_cmd_error.value) == 0


@cocotb.test()
async def transfers_maximum_length(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    read_values = [((index * 29) + 7) & 0xFF for index in range(32)]
    read_slave = cocotb.start_soon(I2cSlave(dut).register_read(read_values))
    read_data = cocotb.start_soon(collect_read_data(dut, len(read_values)))
    await issue_command(dut, read=1, register=0x40, byte_count=len(read_values))
    read_register, master_acks = await with_timeout(
        read_slave, TRANSACTION_TIMEOUT_MS, "ms"
    )
    assert await with_timeout(read_data, TRANSACTION_TIMEOUT_MS, "ms") == read_values
    await wait_until_ready(dut)
    assert read_register == 0x40
    assert master_acks == ([True] * 31) + [False]

    write_values = [((index * 17) + 3) & 0xFF for index in range(32)]
    write_slave = cocotb.start_soon(I2cSlave(dut).register_write(len(write_values)))
    write_data = cocotb.start_soon(provide_write_data(dut, write_values))
    await issue_command(dut, read=0, register=0x41, byte_count=len(write_values))
    write_register, observed = await with_timeout(
        write_slave, TRANSACTION_TIMEOUT_MS, "ms"
    )
    await with_timeout(write_data, TRANSACTION_TIMEOUT_MS, "ms")
    await wait_until_ready(dut)
    assert write_register == 0x41
    assert observed == write_values


@cocotb.test()
async def reports_device_address_nack_and_recovers(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    slave = I2cSlave(dut)
    nack_task = cocotb.start_soon(slave.receive_address(ack=False, expected_rw=0))
    error_task = cocotb.start_soon(wait_for_command_error(dut))
    await issue_command(dut, read=1, register=0x75, byte_count=1)
    await with_timeout(nack_task, TRANSACTION_TIMEOUT_MS, "ms")
    await with_timeout(error_task, TRANSACTION_TIMEOUT_MS, "ms")
    await wait_for_stop(dut)
    await wait_until_ready(dut)

    expected = [0x5A]
    read_slave = cocotb.start_soon(slave.register_read(expected))
    read_data = cocotb.start_soon(collect_read_data(dut, 1))
    await issue_command(dut, read=1, register=0x01, byte_count=1)
    register, master_acks = await with_timeout(
        read_slave, TRANSACTION_TIMEOUT_MS, "ms"
    )
    assert await with_timeout(read_data, TRANSACTION_TIMEOUT_MS, "ms") == expected
    await wait_until_ready(dut)
    assert register == 0x01
    assert master_acks == [False]
