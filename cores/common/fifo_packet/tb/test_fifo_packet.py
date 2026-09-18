import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer


WR_CLK_NS = 10
RD_CLK_NS = 14


class PacketFifo:
    def __init__(self, dut, prefix):
        self.wr_en = getattr(dut, f"{prefix}_i_wr_en")
        self.wr_data = getattr(dut, f"{prefix}_i_wr_data")
        self.wr_user = getattr(dut, f"{prefix}_i_wr_user")
        self.wr_sop = getattr(dut, f"{prefix}_i_wr_sop")
        self.wr_eop = getattr(dut, f"{prefix}_i_wr_eop")
        self.full = getattr(dut, f"{prefix}_o_full")
        self.empty = getattr(dut, f"{prefix}_o_empty")
        self.rd_en = getattr(dut, f"{prefix}_i_rd_en")
        self.rd_data = getattr(dut, f"{prefix}_o_rd_data")
        self.rd_user = getattr(dut, f"{prefix}_o_rd_user")
        self.rd_sop = getattr(dut, f"{prefix}_o_rd_sop")
        self.rd_eop = getattr(dut, f"{prefix}_o_rd_eop")


def start_clocks(dut):
    cocotb.start_soon(Clock(dut.i_clk_w, WR_CLK_NS, unit="ns").start())
    cocotb.start_soon(Clock(dut.i_clk_r, RD_CLK_NS, unit="ns").start())


async def reset(dut):
    for prefix in ("s", "a"):
        fifo = PacketFifo(dut, prefix)
        fifo.wr_en.value = 0
        fifo.rd_en.value = 0
        fifo.wr_data.value = 0
        fifo.wr_user.value = 0
        fifo.wr_sop.value = 0
        fifo.wr_eop.value = 0
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


async def send_word(fifo, clk, data, user=0, sop=0, eop=0):
    await FallingEdge(clk)
    fifo.rd_en.value = 0
    fifo.wr_en.value = 1
    fifo.wr_data.value = data
    fifo.wr_user.value = user
    fifo.wr_sop.value = sop
    fifo.wr_eop.value = eop
    await RisingEdge(clk)


async def recv_word(fifo, clk):
    await Timer(1, unit="ns")
    assert int(fifo.empty.value) == 0, "read from empty packet FIFO"
    word = (
        int(fifo.rd_data.value),
        int(fifo.rd_user.value),
        int(fifo.rd_sop.value),
        int(fifo.rd_eop.value),
    )
    await FallingEdge(clk)
    fifo.wr_en.value = 0
    fifo.wr_sop.value = 0
    fifo.wr_eop.value = 0
    fifo.rd_en.value = 1
    await RisingEdge(clk)
    fifo.rd_en.value = 0
    return word


PACKETS = [
    [(0x10, 0x1), (0x11, 0x1), (0x12, 0x1)],
    [(0x20, 0x2)],
    [(0x30, 0x3), (0x31, 0x3), (0x32, 0x3), (0x33, 0x3)],
]


def expected_words():
    words = []
    for pkt in PACKETS:
        for index, (data, user) in enumerate(pkt):
            words.append(
                (data, user, int(index == 0), int(index == len(pkt) - 1))
            )
    return words


async def drive_packets(fifo, clk):
    for pkt in PACKETS:
        for index, (data, user) in enumerate(pkt):
            await send_word(
                fifo, clk, data, user,
                sop=int(index == 0), eop=int(index == len(pkt) - 1),
            )


async def drain_words(fifo, clk, count):
    words = []
    for _ in range(count):
        words.append(await recv_word(fifo, clk))
    return words


@cocotb.test()
async def sync_packet_boundaries(dut):
    start_clocks(dut)
    await reset(dut)
    s = PacketFifo(dut, "s")

    await drive_packets(s, dut.i_clk_w)
    words = await drain_words(s, dut.i_clk_w, len(expected_words()))
    assert words == expected_words(), f"packet words mismatch: {words}"


@cocotb.test()
async def async_packet_boundaries(dut):
    start_clocks(dut)
    await reset(dut)
    a = PacketFifo(dut, "a")

    await drive_packets(a, dut.i_clk_w)
    # give the write pointer time to cross into the read domain
    for _ in range(4):
        await RisingEdge(dut.i_clk_r)
    words = await drain_words(a, dut.i_clk_r, len(expected_words()))
    assert words == expected_words(), f"packet words mismatch: {words}"
