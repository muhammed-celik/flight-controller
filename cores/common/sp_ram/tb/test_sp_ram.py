import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge


SYS_CLK_NS = 10


class SpRam:
    """Simple driver for one sp_ram instance in the testbench."""

    def __init__(self, dut, prefix):
        self.en = getattr(dut, f"{prefix}_i_en")
        self.we = getattr(dut, f"{prefix}_i_we")
        self.addr = getattr(dut, f"{prefix}_i_addr")
        self.wdata = getattr(dut, f"{prefix}_i_wdata")
        self.be = getattr(dut, f"{prefix}_i_be")
        self.rdata = getattr(dut, f"{prefix}_o_rdata")

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


def start_clock(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())


@cocotb.test()
async def read_write_all_addresses(dut):
    """Instance a (READ_FIRST, no output register) has one cycle read latency."""
    start_clock(dut)
    a = SpRam(dut, "a")

    expected = [(addr * 0x11111111) & 0xFFFFFFFF for addr in range(16)]
    for addr, data in enumerate(expected):
        await a.write(dut.i_clk, addr, data)

    for addr, data in enumerate(expected):
        value = await a.read(dut.i_clk, addr, latency=1)
        assert value == data, f"addr {addr}: got {value:#010x}, want {data:#010x}"


@cocotb.test()
async def read_first_returns_old_data(dut):
    """A write with READ_FIRST drives the pre-write contents to o_rdata."""
    start_clock(dut)
    a = SpRam(dut, "a")

    await a.write(dut.i_clk, 5, 0xDEADBEEF)

    await FallingEdge(dut.i_clk)
    a.en.value = 1
    a.we.value = 1
    a.addr.value = 5
    a.wdata.value = 0x11223344
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(a.rdata.value) == 0xDEADBEEF, "READ_FIRST must present the old word"

    assert await a.read(dut.i_clk, 5, latency=1) == 0x11223344


@cocotb.test()
async def disabled_operations_are_ignored(dut):
    start_clock(dut)
    a = SpRam(dut, "a")

    await a.write(dut.i_clk, 2, 0xCAFEF00D)

    await FallingEdge(dut.i_clk)
    a.en.value = 0
    a.we.value = 1
    a.addr.value = 2
    a.wdata.value = 0x00000000
    await RisingEdge(dut.i_clk)

    assert await a.read(dut.i_clk, 2, latency=1) == 0xCAFEF00D


@cocotb.test()
async def byte_enable_and_write_first(dut):
    """Instance b has byte enables, an output register and WRITE_FIRST."""
    start_clock(dut)
    b = SpRam(dut, "b")

    await b.write(dut.i_clk, 0, 0xABCD, be=0b11)
    assert await b.read(dut.i_clk, 0, latency=2) == 0xABCD

    await b.write(dut.i_clk, 0, 0x0012, be=0b01)
    assert await b.read(dut.i_clk, 0, latency=2) == 0xAB12

    await b.write(dut.i_clk, 0, 0x3400, be=0b10)
    assert await b.read(dut.i_clk, 0, latency=2) == 0x3412

    await b.write(dut.i_clk, 0, 0xFFFF, be=0b00)
    assert await b.read(dut.i_clk, 0, latency=2) == 0x3412


@cocotb.test()
async def write_first_bypasses_to_output(dut):
    start_clock(dut)
    b = SpRam(dut, "b")

    await FallingEdge(dut.i_clk)
    b.en.value = 1
    b.we.value = 1
    b.addr.value = 1
    b.wdata.value = 0x1234
    b.be.value = 0b11
    await RisingEdge(dut.i_clk)
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(b.rdata.value) == 0x1234, "WRITE_FIRST must bypass the written word"


@cocotb.test()
async def no_change_holds_output(dut):
    """Instance c uses NO_CHANGE, so o_rdata holds during a write."""
    start_clock(dut)
    c = SpRam(dut, "c")

    await c.write(dut.i_clk, 1, 0x5A)
    assert await c.read(dut.i_clk, 1, latency=1) == 0x5A

    await FallingEdge(dut.i_clk)
    c.en.value = 1
    c.we.value = 1
    c.addr.value = 1
    c.wdata.value = 0xA5
    await RisingEdge(dut.i_clk)
    await ReadOnly()
    assert int(c.rdata.value) == 0x5A, "NO_CHANGE must hold the previous output"

    assert await c.read(dut.i_clk, 1, latency=1) == 0xA5
