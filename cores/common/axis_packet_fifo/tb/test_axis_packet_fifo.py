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


async def send_beat(ax, clk, data, user, last):
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


async def send_idle(ax, clk):
    await FallingEdge(clk)
    ax.s_tvalid.value = 0


async def axis_recv(ax, clk, count):
    received = []
    ax.m_tready.value = 1
    while len(received) < count:
        await ReadOnly()
        if int(ax.m_tvalid.value) == 1:
            received.append(
                (int(ax.m_tdata.value), int(ax.m_tuser.value), int(ax.m_tlast.value))
            )
            await RisingEdge(clk)
            await Timer(1, unit="ns")
        else:
            await RisingEdge(clk)
    await FallingEdge(clk)
    ax.m_tready.value = 0
    return received


PACKET = [(0x10, 0x1, 0), (0x11, 0x1, 0), (0x12, 0x1, 1), (0x13, 0x1, 0)]
PACKET_B = [(0x20, 0x2, 0), (0x21, 0x2, 1)]


@cocotb.test()
async def sync_store_and_forward(dut):
    start_clocks(dut)
    await reset(dut)
    s = Axis(dut, "s")

    # Partial packet: master must stay idle.
    await send_beat(s, dut.i_clk_w, *PACKET[0])
    await send_beat(s, dut.i_clk_w, *PACKET[1])
    await send_idle(s, dut.i_clk_w)
    for _ in range(3):
        await RisingEdge(dut.i_clk_w)
    await Timer(1, unit="ns")
    assert int(s.m_tvalid.value) == 0, "master must not see an incomplete packet"

    # Complete the packet; now the whole packet becomes visible.
    await send_beat(s, dut.i_clk_w, *PACKET[2])
    await send_idle(s, dut.i_clk_w)
    for _ in range(2):
        await RisingEdge(dut.i_clk_w)
    await Timer(1, unit="ns")
    assert int(s.m_tvalid.value) == 1, "master must see the completed packet"

    received = await axis_recv(s, dut.i_clk_w, 3)
    assert received == PACKET[:3], f"packet mismatch: {received}"


@cocotb.test()
async def async_packet_boundaries(dut):
    start_clocks(dut)
    await reset(dut)
    a = Axis(dut, "a")

    all_beats = PACKET + PACKET_B
    for beat in all_beats:
        await send_beat(a, dut.i_clk_w, *beat)
    await send_idle(a, dut.i_clk_w)

    received = await axis_recv(a, dut.i_clk_r, len(all_beats))
    assert received == all_beats, f"packet boundaries mismatch: {received}"
