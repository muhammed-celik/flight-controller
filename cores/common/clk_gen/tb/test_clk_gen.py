import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, with_timeout
from cocotb.utils import get_sim_time


INPUT_PERIOD_PS = 83_334
OUTPUT_PERIOD_NS = 10.0


async def start_input_clock(dut):
    clock = Clock(dut.clk_in, INPUT_PERIOD_PS, unit="ps")
    return cocotb.start_soon(clock.start())


@cocotb.test()
async def locks_and_generates_100_mhz(dut):
    clock_task = await start_input_clock(dut)
    try:
        dut.rst.value = 1
        await Timer(500, unit="ns")
        assert dut.locked.value == 0

        dut.rst.value = 0
        await with_timeout(RisingEdge(dut.locked), 2, "us")

        edge_times = []
        for _ in range(11):
            await RisingEdge(dut.clk_out)
            edge_times.append(get_sim_time(unit="ns"))

        measured = [b - a for a, b in zip(edge_times, edge_times[1:])]
        assert all(abs(period - OUTPUT_PERIOD_NS) < 0.01 for period in measured)
    finally:
        clock_task.cancel()


@cocotb.test()
async def reset_drops_lock_and_relocks(dut):
    clock_task = await start_input_clock(dut)
    try:
        dut.rst.value = 1
        await Timer(1, unit="ns")
        dut.rst.value = 0
        await with_timeout(RisingEdge(dut.locked), 2, "us")

        dut.rst.value = 1
        await Timer(1, unit="ns")
        assert dut.locked.value == 0

        dut.rst.value = 0
        await with_timeout(RisingEdge(dut.locked), 2, "us")
        assert dut.locked.value == 1
    finally:
        clock_task.cancel()
