import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer


SYS_CLK_NS = 10
SLOW_CLK_NS = 14


class SdpRam:
    """Simple driver for one sdp_ram instance in the testbench."""

    def __init__(self, dut, prefix):
        self.en_w = getattr(dut, f"{prefix}_i_en_w")
        self.we_w = getattr(dut, f"{prefix}_i_we_w")
        self.addr_w = getattr(dut, f"{prefix}_i_addr_w")
        self.wdata = getattr(dut, f"{prefix}_i_wdata")
        self.be = getattr(dut, f"{prefix}_i_be")
        self.en_r = getattr(dut, f"{prefix}_i_en_r")
        self.addr_r = getattr(dut, f"{prefix}_i_addr_r")
        self.rdata = getattr(dut, f"{prefix}_o_rdata")

    async def write(self, clk, addr, data, be=None):
        await FallingEdge(clk)
        self.en_w.value = 1
        self.we_w.value = 1
        self.addr_w.value = addr
        self.wdata.value = data
        if be is not None:
            self.be.value = be
        self.en_r.value = 0
        await RisingEdge(clk)

    async def read(self, clk, addr, latency=1):
        await FallingEdge(clk)
        self.en_w.value = 0
        self.en_r.value = 1
        self.addr_r.value = addr
        for _ in range(latency):
            await RisingEdge(clk)
        await ReadOnly()
        return int(self.rdata.value)


def start_clocks(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.b_i_clk_w, SYS_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.b_i_clk_r, SLOW_CLK_NS, unit="ns").start())


@cocotb.test()
async def single_clock_write_read(dut):
    start_clocks(dut)
    a = SdpRam(dut, "a")

    expected = [(addr * 0x01020304) & 0xFFFFFFFF for addr in range(16)]
    for addr, data in enumerate(expected):
        await a.write(dut.i_clk, addr, data)

    for addr, data in enumerate(expected):
        value = await a.read(dut.i_clk, addr, latency=1)
        assert value == data, f"addr {addr}: got {value:#010x}, want {data:#010x}"


@cocotb.test()
async def read_during_write_returns_old_data(dut):
    """With a shared clock a read of the written address returns old contents."""
    start_clocks(dut)
    a = SdpRam(dut, "a")

    await a.write(dut.i_clk, 7, 0xDEADBEEF)

    await FallingEdge(dut.i_clk)
    a.en_w.value = 1
    a.we_w.value = 1
    a.addr_w.value = 7
    a.wdata.value = 0x11223344
    a.en_r.value = 1
    a.addr_r.value = 7
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(a.rdata.value) == 0xDEADBEEF, "read must return pre-write contents"

    assert await a.read(dut.i_clk, 7, latency=1) == 0x11223344


@cocotb.test()
async def disabled_read_holds_output(dut):
    start_clocks(dut)
    a = SdpRam(dut, "a")

    await a.write(dut.i_clk, 3, 0xA5A5A5A5)
    assert await a.read(dut.i_clk, 3, latency=1) == 0xA5A5A5A5

    await FallingEdge(dut.i_clk)
    a.en_w.value = 0
    a.en_r.value = 0
    a.addr_r.value = 0
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(a.rdata.value) == 0xA5A5A5A5, "disabled read must hold output"


@cocotb.test()
async def dual_clock_write_then_read(dut):
    start_clocks(dut)
    b = SdpRam(dut, "b")

    await b.write(dut.b_i_clk_w, 5, 0xBEEF, be=0b11)
    await Timer(3 * SYS_CLK_NS, unit="ns")
    assert await b.read(dut.b_i_clk_r, 5, latency=2) == 0xBEEF


@cocotb.test()
async def byte_enable_with_output_register(dut):
    start_clocks(dut)
    b = SdpRam(dut, "b")

    await b.write(dut.b_i_clk_w, 0, 0xABCD, be=0b11)
    assert await b.read(dut.b_i_clk_r, 0, latency=2) == 0xABCD

    await b.write(dut.b_i_clk_w, 0, 0x0012, be=0b01)
    assert await b.read(dut.b_i_clk_r, 0, latency=2) == 0xAB12

    await b.write(dut.b_i_clk_w, 0, 0x3400, be=0b10)
    assert await b.read(dut.b_i_clk_r, 0, latency=2) == 0x3412

    await b.write(dut.b_i_clk_w, 0, 0xFFFF, be=0b00)
    assert await b.read(dut.b_i_clk_r, 0, latency=2) == 0x3412
