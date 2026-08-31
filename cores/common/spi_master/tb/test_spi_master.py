import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout


SYS_CLK_NS = 10
BYTE_TIMEOUT_US = 20


async def reset_dut(dut):
    dut.i_rstn.value = 0
    dut.i_en.value = 0
    dut.i_data.value = 0
    dut.i_miso.value = 0
    await Timer(5 * SYS_CLK_NS, unit="ns")
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)


async def spi_mode3_slave(dut, response_bytes):
    """Exchange MSB-first bytes as a mode-3 slave and return captured MOSI."""
    captured = []
    await FallingEdge(dut.o_cs)

    for response in response_bytes:
        tx_from_master = 0
        for bit_index in range(7, -1, -1):
            await FallingEdge(dut.o_sclk)
            assert dut.o_cs.value == 0, "chip select rose during a byte"
            dut.i_miso.value = (response >> bit_index) & 1

            await RisingEdge(dut.o_sclk)
            tx_from_master = (tx_from_master << 1) | int(dut.o_mosi.value)
        captured.append(tx_from_master)

    return captured


async def exchange(dut, tx_bytes, response_bytes):
    assert len(tx_bytes) == len(response_bytes)
    slave_task = cocotb.start_soon(spi_mode3_slave(dut, response_bytes))

    dut.i_data.value = tx_bytes[0]
    dut.i_en.value = 1
    received = []

    for index in range(len(tx_bytes)):
        await with_timeout(RisingEdge(dut.o_done), BYTE_TIMEOUT_US, "us")
        await ReadOnly()
        received.append(int(dut.o_data.value))
        await Timer(1, unit="ns")

        if index + 1 < len(tx_bytes):
            dut.i_data.value = tx_bytes[index + 1]
        else:
            dut.i_en.value = 0

    captured = await with_timeout(slave_task, BYTE_TIMEOUT_US, "us")
    await with_timeout(RisingEdge(dut.o_cs), BYTE_TIMEOUT_US, "us")
    return captured, received


@cocotb.test()
async def single_byte_full_duplex_transfer(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    captured, received = await exchange(dut, [0xA5], [0x3C])

    assert captured == [0xA5]
    assert received == [0x3C]
    assert dut.o_sclk.value == 1, "mode-3 clock must idle high"
    assert dut.o_done.value == 0, "done must be a one-cycle pulse"


@cocotb.test()
async def burst_transfer_keeps_chip_select_asserted(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    tx_bytes = [0x80, 0x12, 0x34, 0x56]
    response_bytes = [0xDE, 0xAD, 0xBE, 0xEF]
    captured, received = await exchange(dut, tx_bytes, response_bytes)

    assert captured == tx_bytes
    assert received == response_bytes
    assert dut.o_cs.value == 1
