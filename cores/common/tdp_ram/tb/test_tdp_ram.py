import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer


SYS_CLK_NS = 10
SLOW_CLK_NS = 14


class TdpPort:
    """Driver for one read/write port of a tdp_ram instance."""

    def __init__(self, dut, prefix, port):
        self.en = getattr(dut, f"{prefix}_i_en_{port}")
        self.we = getattr(dut, f"{prefix}_i_we_{port}")
        self.addr = getattr(dut, f"{prefix}_i_addr_{port}")
        self.wdata = getattr(dut, f"{prefix}_i_wdata_{port}")
        self.be = getattr(dut, f"{prefix}_i_be_{port}")
        self.rdata = getattr(dut, f"{prefix}_o_rdata_{port}")

    async def write(self, clk, addr, data, be=None):
        await FallingEdge(clk)
        self.en.value = 1
        self.we.value = 1
        self.addr.value = addr
        self.wdata.value = data
        if be is not None:
            self.be.value = be
        await RisingEdge(clk)

    async def read(self, clk, addr, latency=1):
        await FallingEdge(clk)
        self.en.value = 1
        self.we.value = 0
        self.addr.value = addr
        for _ in range(latency):
            await RisingEdge(clk)
        await ReadOnly()
        return int(self.rdata.value)

    async def idle(self, clk):
        await FallingEdge(clk)
        self.en.value = 0
        self.we.value = 0


def start_clocks(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.p1_i_clk_a, SYS_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.p1_i_clk_b, SLOW_CLK_NS, unit="ns").start())


@cocotb.test()
async def write_port_a_read_port_b(dut):
    start_clocks(dut)
    a = TdpPort(dut, "p0", "a")
    b = TdpPort(dut, "p0", "b")

    expected = [(addr * 0x10203040) & 0xFFFFFFFF for addr in range(16)]
    for addr, data in enumerate(expected):
        await a.write(dut.i_clk, addr, data)
    await a.idle(dut.i_clk)

    for addr, data in enumerate(expected):
        value = await b.read(dut.i_clk, addr, latency=1)
        assert value == data, f"addr {addr}: got {value:#010x}, want {data:#010x}"


@cocotb.test()
async def write_port_b_read_port_a(dut):
    start_clocks(dut)
    a = TdpPort(dut, "p0", "a")
    b = TdpPort(dut, "p0", "b")

    await b.write(dut.i_clk, 9, 0xC0FFEE00)
    await b.idle(dut.i_clk)
    assert await a.read(dut.i_clk, 9, latency=1) == 0xC0FFEE00


@cocotb.test()
async def simultaneous_access_different_addresses(dut):
    start_clocks(dut)
    a = TdpPort(dut, "p0", "a")
    b = TdpPort(dut, "p0", "b")

    await a.write(dut.i_clk, 5, 0x11111111)
    await a.write(dut.i_clk, 6, 0x22222222)

    await FallingEdge(dut.i_clk)
    a.en.value = 1
    a.we.value = 1
    a.addr.value = 5
    a.wdata.value = 0x33333333
    b.en.value = 1
    b.we.value = 0
    b.addr.value = 6
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(b.rdata.value) == 0x22222222

    await a.idle(dut.i_clk)
    assert await b.read(dut.i_clk, 5, latency=1) == 0x33333333


@cocotb.test()
async def write_first_bypasses_to_output(dut):
    """Instance p1 uses WRITE_FIRST with an output register (2 cycle latency)."""
    start_clocks(dut)
    a = TdpPort(dut, "p1", "a")

    await FallingEdge(dut.p1_i_clk_a)
    a.en.value = 1
    a.we.value = 1
    a.addr.value = 1
    a.wdata.value = 0x1234
    a.be.value = 0b11
    await RisingEdge(dut.p1_i_clk_a)
    await RisingEdge(dut.p1_i_clk_a)
    await ReadOnly()
    assert int(a.rdata.value) == 0x1234, "WRITE_FIRST must bypass the written word"


@cocotb.test()
async def byte_enable_on_port_b(dut):
    start_clocks(dut)
    b = TdpPort(dut, "p1", "b")

    await b.write(dut.p1_i_clk_b, 0, 0xABCD, be=0b11)
    assert await b.read(dut.p1_i_clk_b, 0, latency=2) == 0xABCD

    await b.write(dut.p1_i_clk_b, 0, 0x0012, be=0b01)
    assert await b.read(dut.p1_i_clk_b, 0, latency=2) == 0xAB12

    await b.write(dut.p1_i_clk_b, 0, 0x3400, be=0b10)
    assert await b.read(dut.p1_i_clk_b, 0, latency=2) == 0x3412

    await b.write(dut.p1_i_clk_b, 0, 0xFFFF, be=0b00)
    assert await b.read(dut.p1_i_clk_b, 0, latency=2) == 0x3412


@cocotb.test()
async def dual_clock_write_a_read_b(dut):
    start_clocks(dut)
    a = TdpPort(dut, "p1", "a")
    b = TdpPort(dut, "p1", "b")

    await a.write(dut.p1_i_clk_a, 4, 0xBEEF, be=0b11)
    await Timer(3 * SYS_CLK_NS, unit="ns")
    assert await b.read(dut.p1_i_clk_b, 4, latency=2) == 0xBEEF
