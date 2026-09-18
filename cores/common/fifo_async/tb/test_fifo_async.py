import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer


WR_CLK_NS = 10
RD_CLK_NS = 14


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


def start_clocks(dut):
    cocotb.start_soon(Clock(dut.i_clk_w, WR_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.i_clk_r, RD_CLK_NS, unit="ns").start())


async def reset(dut):
    for prefix in ("a", "b"):
        fifo = Fifo(dut, prefix)
        fifo.wr_en.value = 0
        fifo.rd_en.value = 0
        fifo.wr_data.value = 0
        fifo.wr_user.value = 0
    dut.i_rstn_w.value = 0
    dut.i_rstn_r.value = 0
    for _ in range(3):
        await RisingEdge(dut.i_clk_w)
        await RisingEdge(dut.i_clk_r)
    dut.i_rstn_w.value = 1
    dut.i_rstn_r.value = 1
    await RisingEdge(dut.i_clk_w)
    await RisingEdge(dut.i_clk_r)


async def producer(fifo, clk, words, users):
    for data, user in zip(words, users):
        await FallingEdge(clk)
        fifo.wr_en.value = 0
        while int(fifo.full.value) == 1:
            await FallingEdge(clk)
        fifo.wr_en.value = 1
        fifo.wr_data.value = data
        fifo.wr_user.value = user
        await RisingEdge(clk)
    await FallingEdge(clk)
    fifo.wr_en.value = 0


async def consumer(fifo, clk, count):
    received = []
    for _ in range(count):
        await FallingEdge(clk)
        fifo.rd_en.value = 0
        guard = 0
        while int(fifo.empty.value) == 1:
            await FallingEdge(clk)
            guard += 1
            assert guard < 1000, (
                f"timeout waiting for FIFO data: empty={int(fifo.empty.value)} "
                f"level={int(fifo.level.value)} data={int(fifo.rd_data.value)}"
            )
        await Timer(1, unit="ns")
        received.append((int(fifo.rd_data.value), int(fifo.rd_user.value)))
        fifo.rd_en.value = 1
        await RisingEdge(clk)
    await FallingEdge(clk)
    fifo.rd_en.value = 0
    return received


async def wait_cycles(clk, count):
    for _ in range(count):
        await RisingEdge(clk)


@cocotb.test()
async def cross_domain_order(dut):
    start_clocks(dut)
    await reset(dut)
    a = Fifo(dut, "a")

    count = 40
    words = [0x10000000 + i for i in range(count)]
    users = [i & 0xF for i in range(count)]

    prod = cocotb.start_soon(producer(a, dut.i_clk_w, words, users))
    cons = cocotb.start_soon(consumer(a, dut.i_clk_r, count))
    await prod
    received = await cons

    assert [data for data, _ in received] == words, "data order across domains"
    assert [user for _, user in received] == users, "user order across domains"


@cocotb.test()
async def fill_to_full_and_drain(dut):
    start_clocks(dut)
    await reset(dut)
    b = Fifo(dut, "b")

    written = 0
    while written < 24:
        await FallingEdge(dut.i_clk_w)
        if int(b.full.value) == 1:
            break
        b.wr_en.value = 0
        b.wr_en.value = 1
        b.wr_data.value = written
        b.wr_user.value = written & 1
        await RisingEdge(dut.i_clk_w)
        written += 1
    await FallingEdge(dut.i_clk_w)
    b.wr_en.value = 0

    await wait_cycles(dut.i_clk_w, 3)
    await Timer(1, unit="ns")
    assert int(b.full.value) == 1, "FIFO must report full when filled"
    assert written >= 8, f"FIFO capacity too small: {written}"

    received = await consumer(b, dut.i_clk_r, written)
    assert [data for data, _ in received] == list(range(written))
    assert [user for _, user in received] == [i & 1 for i in range(written)]

    await wait_cycles(dut.i_clk_r, 4)
    await Timer(1, unit="ns")
    assert int(b.empty.value) == 1, "FIFO must be empty after drain"


@cocotb.test()
async def almost_flags(dut):
    start_clocks(dut)
    await reset(dut)
    a = Fifo(dut, "a")

    written = 0
    while written < 24:
        await FallingEdge(dut.i_clk_w)
        if int(a.afull.value) == 1:
            break
        a.wr_en.value = 0
        a.wr_en.value = 1
        a.wr_data.value = written
        a.wr_user.value = 0
        await RisingEdge(dut.i_clk_w)
        written += 1
    await FallingEdge(dut.i_clk_w)
    a.wr_en.value = 0
    await Timer(1, unit="ns")
    assert int(a.afull.value) == 1, "almost_full must assert before full"
    assert int(a.full.value) == 0, "almost_full must precede full"

    # Let the write pointer propagate, then drain until only one word remains.
    await wait_cycles(dut.i_clk_r, 12)

    reads = 0
    while reads < written:
        await FallingEdge(dut.i_clk_r)
        a.rd_en.value = 1
        await RisingEdge(dut.i_clk_r)
        a.rd_en.value = 0
        await Timer(1, unit="ns")
        if int(a.aempty.value) == 1 or int(a.empty.value) == 1:
            break
        reads += 1

    assert int(a.aempty.value) == 1, (
        f"almost_empty must assert when one word remains: "
        f"level={int(a.level.value)} empty={int(a.empty.value)} reads={reads}"
    )
