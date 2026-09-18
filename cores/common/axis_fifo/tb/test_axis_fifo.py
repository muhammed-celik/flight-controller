import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, RisingEdge, Timer


WR_CLK_NS = 10
RD_CLK_NS = 14


class Axis:
    def __init__(self, dut, prefix):
        self.s_tvalid = getattr(dut, f"{prefix}_i_s_tvalid")
        self.s_tready = getattr(dut, f"{prefix}_o_s_tready")
        self.s_tdata = getattr(dut, f"{prefix}_i_s_tdata")
        self.s_tuser = getattr(dut, f"{prefix}_i_s_tuser")
        self.s_tlast = getattr(dut, f"{prefix}_i_s_tlast")
        self.m_tvalid = getattr(dut, f"{prefix}_o_m_tvalid")
        self.m_tready = getattr(dut, f"{prefix}_i_m_tready")
        self.m_tdata = getattr(dut, f"{prefix}_o_m_tdata")
        self.m_tuser = getattr(dut, f"{prefix}_o_m_tuser")
        self.m_tlast = getattr(dut, f"{prefix}_o_m_tlast")


def start_clocks(dut):
    cocotb.start_soon(Clock(dut.i_clk_w, WR_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.i_clk_r, RD_CLK_NS, unit="ns").start())


async def reset(dut):
    for prefix in ("s", "a"):
        ax = Axis(dut, prefix)
        ax.s_tvalid.value = 0
        ax.s_tdata.value = 0
        ax.s_tuser.value = 0
        ax.s_tlast.value = 0
        ax.m_tready.value = 0
    dut.i_rstn_w.value = 0
    dut.i_rstn_r.value = 0
    for _ in range(3):
        await RisingEdge(dut.i_clk_w)
    for _ in range(3):
        await RisingEdge(dut.i_clk_r)
    dut.i_rstn_w.value = 1
    dut.i_rstn_r.value = 1
    await RisingEdge(dut.i_clk_w)
    await RisingEdge(dut.i_clk_r)


async def axis_send(ax, clk, beats):
    for data, user, last in beats:
        await FallingEdge(clk)
        ax.s_tvalid.value = 1
        ax.s_tdata.value = data
        ax.s_tuser.value = user
        ax.s_tlast.value = last
        await ReadOnly()
        while int(ax.s_tready.value) == 0:
            await RisingEdge(clk)
            await ReadOnly()
        await RisingEdge(clk)
    await FallingEdge(clk)
    ax.s_tvalid.value = 0


async def axis_recv(ax, clk, count, ready_delay=0):
    received = []
    ax.m_tready.value = 0
    if ready_delay:
        for _ in range(ready_delay):
            await RisingEdge(clk)
    ax.m_tready.value = 1
    while len(received) < count:
        await ReadOnly()
        if int(ax.m_tvalid.value) == 1:
            received.append(
                (int(ax.m_tdata.value), int(ax.m_tuser.value), int(ax.m_tlast.value))
            )
            await RisingEdge(clk)
        else:
            await RisingEdge(clk)
    await FallingEdge(clk)
    ax.m_tready.value = 0
    return received


BEATS = [
    (0x1000, 0x1, 0),
    (0x1001, 0x1, 0),
    (0x1002, 0x1, 1),
    (0x2000, 0x2, 0),
    (0x2001, 0x2, 1),
]


@cocotb.test()
async def sync_stream(dut):
    start_clocks(dut)
    await reset(dut)
    s = Axis(dut, "s")

    await axis_send(s, dut.i_clk_w, BEATS)
    received = await axis_recv(s, dut.i_clk_w, len(BEATS))
    assert received == BEATS, f"stream mismatch: {received}"


@cocotb.test()
async def sync_ready_backpressure(dut):
    start_clocks(dut)
    await reset(dut)
    s = Axis(dut, "s")

    await axis_send(s, dut.i_clk_w, BEATS)
    # Hold tready low for a while; data must be retained.
    received = await axis_recv(s, dut.i_clk_w, len(BEATS), ready_delay=3)
    assert received == BEATS, f"backpressure mismatch: {received}"


@cocotb.test()
async def async_stream(dut):
    start_clocks(dut)
    await reset(dut)
    a = Axis(dut, "a")

    count = 20
    beats = [(0x3000 + i, i & 0xF, int((i % 4) == 3)) for i in range(count)]

    send = cocotb.start_soon(axis_send(a, dut.i_clk_w, beats))
    recv = cocotb.start_soon(axis_recv(a, dut.i_clk_r, count))
    await send
    received = await recv
    assert received == beats, f"async stream mismatch: {received}"
