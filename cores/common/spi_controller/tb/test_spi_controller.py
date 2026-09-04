import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout


SYS_CLK_NS = 10
TRANSFER_TIMEOUT_MS = 5


async def reset_dut(dut):
    dut.i_rstn.value = 0
    dut.i_cmd_valid.value = 0
    dut.i_cmd_type.value = 0
    dut.i_cmd_nbytes.value = 0
    dut.i_cmd_addr.value = 0
    dut.i_data_valid.value = 0
    dut.i_data.value = 0
    dut.i_miso.value = 0
    await Timer(5 * SYS_CLK_NS, unit="ns")
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)


async def issue_command(dut, read, address, byte_count):
    assert 1 <= byte_count <= 32
    if not int(dut.o_cmd_ready.value):
        await with_timeout(RisingEdge(dut.o_cmd_ready), TRANSFER_TIMEOUT_MS, "ms")
    await FallingEdge(dut.i_clk)
    dut.i_cmd_type.value = read
    dut.i_cmd_addr.value = address
    dut.i_cmd_nbytes.value = byte_count - 1
    dut.i_cmd_valid.value = 1
    await RisingEdge(dut.i_clk)
    await FallingEdge(dut.i_clk)
    dut.i_cmd_valid.value = 0


async def mode3_slave(dut, response_bytes, write_bytes=None):
    """Exchange MSB-first bytes as an SPI mode 3 slave."""
    captured = []
    await FallingEdge(dut.o_cs)

    for byte_index, response in enumerate(response_bytes):
        master_byte = 0
        for bit_index in range(7, -1, -1):
            await FallingEdge(dut.o_sclk)
            assert int(dut.o_cs.value) == 0, "CS rose during an SPI byte"
            dut.i_miso.value = (response >> bit_index) & 1
            await RisingEdge(dut.o_sclk)
            master_byte = (master_byte << 1) | int(dut.o_mosi.value)

        captured.append(master_byte)
        if write_bytes is not None and 0 < byte_index < len(write_bytes):
            await Timer(1, unit="ns")
            dut.i_data.value = write_bytes[byte_index]

    await FallingEdge(dut.o_sclk)
    dut.i_miso.value = 0
    return captured


async def collect_read_data(dut, byte_count):
    received = []
    for _ in range(byte_count):
        await with_timeout(RisingEdge(dut.o_data_valid), TRANSFER_TIMEOUT_MS, "ms")
        await ReadOnly()
        received.append(int(dut.o_data.value))
    return received


async def wait_for_transaction_end(dut):
    await with_timeout(RisingEdge(dut.o_cs), TRANSFER_TIMEOUT_MS, "ms")
    await ReadOnly()
    assert int(dut.o_cmd_ready.value) == 1
    assert int(dut.o_sclk.value) == 1
    await Timer(1, unit="ns")


async def pulse_write_data(dut, write_data):
    """Present each write byte for one clock after the preceding SPI byte."""
    await FallingEdge(dut.o_cs)
    for data in write_data:
        for _ in range(8):
            await RisingEdge(dut.o_sclk)

        # The controller detects spi_done one clock after the final sample edge.
        for _ in range(3):
            await RisingEdge(dut.i_clk)
        await FallingEdge(dut.i_clk)
        dut.i_data.value = data
        dut.i_data_valid.value = 1
        await RisingEdge(dut.i_clk)
        await FallingEdge(dut.i_clk)
        dut.i_data_valid.value = 0


@cocotb.test()
async def reset_sets_idle_interface(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)
    await ReadOnly()

    assert int(dut.o_cmd_ready.value) == 1
    assert int(dut.o_data_valid.value) == 0
    assert int(dut.o_data.value) == 0
    assert int(dut.o_cs.value) == 1
    assert int(dut.o_sclk.value) == 1
    assert int(dut.o_mosi.value) == 0


@cocotb.test()
async def reads_single_and_multiple_bytes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for address, response_data in ((0x25, [0xA5]), (0x42, [0x12, 0x34, 0x56])):
        command = 0x80 | (address & 0x7F)
        slave_task = cocotb.start_soon(
            mode3_slave(dut, [0x00, *response_data])
        )
        read_task = cocotb.start_soon(collect_read_data(dut, len(response_data)))

        await issue_command(dut, read=1, address=address, byte_count=len(response_data))
        captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
        received = await with_timeout(read_task, TRANSFER_TIMEOUT_MS, "ms")
        await wait_for_transaction_end(dut)

        assert captured == [command, *([0x00] * len(response_data))]
        assert received == response_data


@cocotb.test()
async def writes_single_and_multiple_bytes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for address, write_data in ((0x11, [0xC3]), (0x36, [0xDE, 0xAD, 0xBE])):
        command = address & 0x7F
        dut.i_data.value = write_data[0]
        dut.i_data_valid.value = 1
        slave_task = cocotb.start_soon(
            mode3_slave(dut, [0x00] * (len(write_data) + 1), write_data)
        )

        await issue_command(dut, read=0, address=address, byte_count=len(write_data))
        captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
        dut.i_data_valid.value = 0
        await wait_for_transaction_end(dut)

        assert captured == [command, *write_data]
        assert int(dut.o_data_valid.value) == 0


@cocotb.test()
async def encodes_boundary_addresses(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for address in (0x00, 0x7F, 0x80, 0xFF):
        command = 0x80 | (address & 0x7F)
        response = address ^ 0x5A
        slave_task = cocotb.start_soon(mode3_slave(dut, [0x00, response]))
        read_task = cocotb.start_soon(collect_read_data(dut, 1))

        await issue_command(dut, read=1, address=address, byte_count=1)
        captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
        received = await with_timeout(read_task, TRANSFER_TIMEOUT_MS, "ms")
        await wait_for_transaction_end(dut)

        assert captured == [command, 0x00]
        assert received == [response]


@cocotb.test()
async def reads_maximum_length_transfer(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    response_data = [((index * 29) + 7) & 0xFF for index in range(32)]
    slave_task = cocotb.start_soon(mode3_slave(dut, [0x00, *response_data]))
    read_task = cocotb.start_soon(collect_read_data(dut, len(response_data)))

    await issue_command(dut, read=1, address=0x7E, byte_count=len(response_data))
    captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
    received = await with_timeout(read_task, TRANSFER_TIMEOUT_MS, "ms")
    await wait_for_transaction_end(dut)

    assert captured == [0xFE, *([0x00] * 32)]
    assert received == response_data


@cocotb.test()
async def writes_maximum_length_transfer(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    write_data = [((index * 17) + 3) & 0xFF for index in range(32)]
    dut.i_data.value = write_data[0]
    dut.i_data_valid.value = 1
    slave_task = cocotb.start_soon(
        mode3_slave(dut, [0x00] * 33, write_data)
    )

    await issue_command(dut, read=0, address=0x7D, byte_count=len(write_data))
    captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
    dut.i_data_valid.value = 0
    await wait_for_transaction_end(dut)

    assert captured == [0x7D, *write_data]


@cocotb.test()
async def accepts_single_cycle_write_valid_pulses(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    write_data = [0x10, 0x32, 0x54, 0x76]
    slave_task = cocotb.start_soon(mode3_slave(dut, [0x00] * 5))
    data_task = cocotb.start_soon(pulse_write_data(dut, write_data))

    await issue_command(dut, read=0, address=0x2A, byte_count=len(write_data))
    captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
    await with_timeout(data_task, TRANSFER_TIMEOUT_MS, "ms")
    await wait_for_transaction_end(dut)

    assert captured == [0x2A, *write_data]


@cocotb.test()
async def reset_aborts_transfer_and_recovers(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    await issue_command(dut, read=1, address=0x33, byte_count=3)
    await FallingEdge(dut.o_cs)
    for _ in range(3):
        await RisingEdge(dut.o_sclk)

    dut.i_rstn.value = 0
    await Timer(1, unit="ns")
    assert int(dut.o_cs.value) == 1
    assert int(dut.o_sclk.value) == 1
    assert int(dut.o_cmd_ready.value) == 1
    assert int(dut.o_data_valid.value) == 0

    await Timer(5 * SYS_CLK_NS, unit="ns")
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)

    slave_task = cocotb.start_soon(mode3_slave(dut, [0x00, 0xA6]))
    read_task = cocotb.start_soon(collect_read_data(dut, 1))
    await issue_command(dut, read=1, address=0x19, byte_count=1)
    captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
    received = await with_timeout(read_task, TRANSFER_TIMEOUT_MS, "ms")
    await wait_for_transaction_end(dut)

    assert captured == [0x99, 0x00]
    assert received == [0xA6]
