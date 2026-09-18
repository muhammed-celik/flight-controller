import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer


SYS_CLK_NS = 10


class Fifo:
    def __init__(self, dut, prefix):
        self.wr_en = getattr(dut, f"{prefix}_i_wr_en")
        self.wr_data = getattr(dut, f"{prefix}_i_wr_data")
        self.wr_user = getattr(dut, f"{prefix}_i_wr_user")
        self.full = getattr(dut, f"{prefix}_o_full")
        self.afull = getattr(dut, f"{prefix}_o_almost_full")
        self.empty = getattr(dut, f"{prefix}_o_empty")
        self.aempty = getattr(dut, f"{prefix}_o_almost_empty")
        self.level = getattr(dut, f"{prefix}_o_level")
        self.rd_en = getattr(dut, f"{prefix}_i_rd_en")
        self.rd_data = getattr(dut, f"{prefix}_o_rd_data")
        self.rd_user = getattr(dut, f"{prefix}_o_rd_user")


async def reset(dut):
    for prefix in ("a", "b"):
        fifo = Fifo(dut, prefix)
        fifo.wr_en.value = 0
        fifo.rd_en.value = 0
        fifo.wr_data.value = 0
        fifo.wr_user.value = 0
    dut.i_rstn.value = 0
    await RisingEdge(dut.i_clk)
    await RisingEdge(dut.i_clk)
    dut.i_rstn.value = 1
    await RisingEdge(dut.i_clk)


async def write(fifo, clk, data, user=0):
    await FallingEdge(clk)
    fifo.rd_en.value = 0
    fifo.wr_en.value = 1
    fifo.wr_data.value = data
    fifo.wr_user.value = user
    await RisingEdge(clk)


async def read(fifo, clk):
    """FWFT read: data is valid before i_rd_en is asserted."""
    await Timer(1, unit="ns")
    assert int(fifo.empty.value) == 0, "read from empty FIFO"
    value = int(fifo.rd_data.value)
    user = int(fifo.rd_user.value)
    await FallingEdge(clk)
    fifo.wr_en.value = 0
    fifo.rd_en.value = 1
    await RisingEdge(clk)
    fifo.rd_en.value = 0
    return value, user


@cocotb.test()
async def write_then_read_in_order(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset(dut)
    a = Fifo(dut, "a")

    for i in range(10):
        await write(a, dut.i_clk, 0xA0000000 + i, user=i & 0xF)

    for i in range(10):
        value, user = await read(a, dut.i_clk)
        assert value == 0xA0000000 + i, f"word {i}: got {value:#010x}"
        assert user == (i & 0xF), f"user {i}: got {user}"

    await Timer(1, unit="ns")
    assert int(a.empty.value) == 1
    assert int(a.level.value) == 0


@cocotb.test()
async def full_and_empty_flags(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset(dut)
    b = Fifo(dut, "b")

    for i in range(8):
        await write(b, dut.i_clk, i, user=i & 1)

    await Timer(1, unit="ns")
    assert int(b.full.value) == 1, "FIFO must be full at depth"
    assert int(b.level.value) == 8
    assert int(b.empty.value) == 0

    await write(b, dut.i_clk, 0xEE, user=1)
    await Timer(1, unit="ns")
    assert int(b.level.value) == 8, "write while full must be ignored"

    for i in range(8):
        value, user = await read(b, dut.i_clk)
        assert value == i
        assert user == (i & 1)

    await Timer(1, unit="ns")
    assert int(b.empty.value) == 1
    assert int(b.full.value) == 0


@cocotb.test()
async def almost_flags(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset(dut)
    a = Fifo(dut, "a")

    for i in range(14):
        await write(a, dut.i_clk, i)

    await Timer(1, unit="ns")
    assert int(a.afull.value) == 1, "almost_full must assert at threshold 14"

    for _ in range(13):
        await read(a, dut.i_clk)

    await Timer(1, unit="ns")
    assert int(a.aempty.value) == 1, "almost_empty must assert at level 1"
    assert int(a.level.value) == 1


@cocotb.test()
async def first_word_fall_through(dut):
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset(dut)
    a = Fifo(dut, "a")

    await write(a, dut.i_clk, 0xDEADBEEF, user=0x5)
    for _ in range(3):
        await RisingEdge(dut.i_clk)
    await Timer(1, unit="ns")
    assert int(a.empty.value) == 0, "empty must deassert without i_rd_en"
    assert int(a.rd_data.value) == 0xDEADBEEF, "head word must be visible"
    assert int(a.rd_user.value) == 0x5


@cocotb.test()
async def lockstep_full_throughput(dut):
    """One write and one read per cycle while the FIFO stays full."""
    cocotb.start_soon(Clock(dut.i_clk, SYS_CLK_NS, unit="ns").start())
    await reset(dut)
    b = Fifo(dut, "b")

    for i in range(8):
        await write(b, dut.i_clk, i)

    bursts = 20
    for i in range(bursts):
        await Timer(1, unit="ns")
        assert int(b.empty.value) == 0
        assert int(b.rd_data.value) == i, f"pop order: got {int(b.rd_data.value)}"
        await FallingEdge(dut.i_clk)
        b.wr_en.value = 1
        b.wr_data.value = 8 + i
        b.wr_user.value = 0
        b.rd_en.value = 1
        await RisingEdge(dut.i_clk)
        b.wr_en.value = 0
        b.rd_en.value = 0

    for i in range(bursts, bursts + 8):
        value, _ = await read(b, dut.i_clk)
        assert value == i, f"drain order: got {value}, want {i}"

    await Timer(1, unit="ns")
    assert int(b.empty.value) == 1