import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer, with_timeout
from cocotb.utils import get_sim_time


INPUT_PERIOD_PS = 83_334


async def start_input_clock(dut):
    clock = Clock(dut.clk_12mhz, INPUT_PERIOD_PS, unit="ps")
    return cocotb.start_soon(clock.start())


@cocotb.test()
async def qualifies_lock_and_releases_reset(dut):
    clock_task = await start_input_clock(dut)
    try:
        dut.ext_reset.value = 1
        await Timer(500, unit="ns")
        assert dut.clock_locked.value == 0
        assert dut.rst_n.value == 0

        dut.ext_reset.value = 0
        await with_timeout(RisingEdge(dut.clock_locked), 3, "us")
        assert dut.rst_n.value == 0
        await with_timeout(RisingEdge(dut.rst_n), 100, "ns")
        assert dut.clock_locked.value == 1
    finally:
        clock_task.cancel()


@cocotb.test()
async def output_clock_is_100_mhz(dut):
    clock_task = await start_input_clock(dut)
    try:
        dut.ext_reset.value = 1
        await Timer(100, unit="ns")
        dut.ext_reset.value = 0
        await with_timeout(RisingEdge(dut.rst_n), 3, "us")

        edge_times = []
        for _ in range(11):
            await RisingEdge(dut.clk_100mhz)
            edge_times.append(get_sim_time(unit="ns"))
        periods = [b - a for a, b in zip(edge_times, edge_times[1:])]
        assert all(abs(period - 10.0) < 0.01 for period in periods)
    finally:
        clock_task.cancel()


@cocotb.test()
async def reset_asserts_between_output_clock_edges(dut):
    clock_task = await start_input_clock(dut)
    try:
        dut.ext_reset.value = 1
        await Timer(100, unit="ns")
        dut.ext_reset.value = 0
        await with_timeout(RisingEdge(dut.rst_n), 3, "us")

        await FallingEdge(dut.clk_100mhz)
        await Timer(2, unit="ns")
        dut.ext_reset.value = 1
        await Timer(1, unit="ps")
        assert dut.rst_n.value == 0
        assert dut.clock_locked.value == 0

        dut.ext_reset.value = 0
        await with_timeout(RisingEdge(dut.rst_n), 3, "us")
        assert dut.rst_n.value == 1
    finally:
        clock_task.cancel()
