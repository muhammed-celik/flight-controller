import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout


SYS_CLK_NS = 10
TRANSFER_TIMEOUT_MS = 2


class SpiPort:
    def __init__(self, dut, mode):
        prefix = f"m{mode}"
        self.mode = mode
        self.cpol = (mode >> 1) & 1
        self.cpha = mode & 1
        self.en = getattr(dut, f"{prefix}_i_en")
        self.tx_data = getattr(dut, f"{prefix}_i_data")
        self.miso = getattr(dut, f"{prefix}_i_miso")
        self.done = getattr(dut, f"{prefix}_o_done")
        self.rx_data = getattr(dut, f"{prefix}_o_data")
        self.cs = getattr(dut, f"{prefix}_o_cs")
        self.sclk = getattr(dut, f"{prefix}_o_sclk")
        self.mosi = getattr(dut, f"{prefix}_o_mosi")

    async def leading_edge(self):
        if self.cpol:
            await FallingEdge(self.sclk)
        else:
            await RisingEdge(self.sclk)

    async def trailing_edge(self):
        if self.cpol:
            await RisingEdge(self.sclk)
        else:
            await FallingEdge(self.sclk)

    async def sample_edge(self):
        if self.cpha:
            await self.trailing_edge()
        else:
            await self.leading_edge()

    async def shift_edge(self):
        if self.cpha:
            await self.leading_edge()
        else:
            await self.trailing_edge()


async def reset_dut(dut):
    dut.i_rstn.value = 0
    for mode in range(4):
        port = SpiPort(dut, mode)
        port.en.value = 0
        port.tx_data.value = 0
        port.miso.value = 0
    await Timer(5 * SYS_CLK_NS, unit="ns")
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)


async def spi_slave(port, response_bytes):
    """Exchange MSB-first bytes using the selected SPI mode."""
    captured = []
    await FallingEdge(port.cs)

    for response in response_bytes:
        master_byte = 0
        if not port.cpha:
            port.miso.value = (response >> 7) & 1

        for bit_index in range(7, -1, -1):
            if port.cpha:
                await port.shift_edge()
                assert int(port.cs.value) == 0, f"mode {port.mode}: CS rose during byte"
                port.miso.value = (response >> bit_index) & 1
                await port.sample_edge()
                master_byte = (master_byte << 1) | int(port.mosi.value)
            else:
                await port.sample_edge()
                assert int(port.cs.value) == 0, f"mode {port.mode}: CS rose during byte"
                master_byte = (master_byte << 1) | int(port.mosi.value)
                await port.shift_edge()
                if bit_index:
                    port.miso.value = (response >> (bit_index - 1)) & 1

        captured.append(master_byte)

    if port.cpha:
        await port.shift_edge()
    port.miso.value = 0
    return captured


async def exchange(port, tx_bytes, response_bytes):
    assert len(tx_bytes) == len(response_bytes)
    slave_task = cocotb.start_soon(spi_slave(port, response_bytes))

    port.tx_data.value = tx_bytes[0]
    port.en.value = 1
    received = []

    for index in range(len(tx_bytes)):
        await with_timeout(RisingEdge(port.done), TRANSFER_TIMEOUT_MS, "ms")
        await ReadOnly()
        received.append(int(port.rx_data.value))
        assert int(port.cs.value) == 0, f"mode {port.mode}: CS inactive at done"
        await Timer(1, unit="ns")

        if index + 1 < len(tx_bytes):
            port.tx_data.value = tx_bytes[index + 1]
        else:
            port.en.value = 0

    captured = await with_timeout(slave_task, TRANSFER_TIMEOUT_MS, "ms")
    await with_timeout(RisingEdge(port.cs), TRANSFER_TIMEOUT_MS, "ms")
    await ReadOnly()
    await Timer(1, unit="ns")
    return captured, received


@cocotb.test()
async def reset_sets_idle_bus_levels(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    for mode in range(4):
        port = SpiPort(dut, mode)
        assert int(port.cs.value) == 1, f"mode {mode}: CS must idle high"
        assert int(port.sclk.value) == port.cpol, f"mode {mode}: incorrect SCLK idle level"
        assert int(port.done.value) == 0, f"mode {mode}: done asserted after reset"


@cocotb.test()
async def single_byte_all_modes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    patterns = [(0xA5, 0x3C), (0x00, 0xFF), (0xFF, 0x00), (0x96, 0x69)]
    for mode, (tx_byte, response_byte) in enumerate(patterns):
        port = SpiPort(dut, mode)
        captured, received = await exchange(port, [tx_byte], [response_byte])
        assert captured == [tx_byte], f"mode {mode}: slave captured {captured}"
        assert received == [response_byte], f"mode {mode}: master received {received}"
        assert int(port.sclk.value) == port.cpol, f"mode {mode}: SCLK did not return idle"


@cocotb.test()
async def burst_transfer_all_modes(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)

    tx_bytes = [0x80, 0x12, 0x34, 0x56]
    response_bytes = [0xDE, 0xAD, 0xBE, 0xEF]
    for mode in range(4):
        port = SpiPort(dut, mode)
        captured, received = await exchange(port, tx_bytes, response_bytes)
        assert captured == tx_bytes, f"mode {mode}: slave captured {captured}"
        assert received == response_bytes, f"mode {mode}: master received {received}"


@cocotb.test()
async def reset_aborts_active_transfer(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset_dut(dut)
    port = SpiPort(dut, 3)

    port.tx_data.value = 0xA5
    port.en.value = 1
    await FallingEdge(port.cs)
    await port.leading_edge()
    dut.i_rstn.value = 0
    await Timer(1, unit="ns")

    assert int(port.cs.value) == 1
    assert int(port.sclk.value) == port.cpol
    assert int(port.done.value) == 0
