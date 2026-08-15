import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


EXPECTED_INTERVALS = {
    "enable_1khz": 1,
    "enable_250hz": 4,
    "enable_100hz": 10,
    "enable_50hz": 20,
    "enable_10hz": 100,
}


async def reset_dut(dut):
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await Timer(1, unit="ps")
    assert dut.timestamp_cycles.value == 0
    for signal_name in EXPECTED_INTERVALS:
        assert getattr(dut, signal_name).value == 0
    dut.rst_n.value = 1


@cocotb.test()
async def timestamp_increments_and_wraps(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    try:
        await reset_dut(dut)
        for expected in range(1, 256):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
            assert dut.timestamp_cycles.value == expected

        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert dut.timestamp_cycles.value == 0
    finally:
        clock_task.cancel()


@cocotb.test()
async def rate_enables_have_exact_intervals(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    try:
        await reset_dut(dut)
        pulse_cycles = {name: [] for name in EXPECTED_INTERVALS}

        for cycle in range(1, 301):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
            for signal_name in EXPECTED_INTERVALS:
                if getattr(dut, signal_name).value == 1:
                    pulse_cycles[signal_name].append(cycle)

        for signal_name, interval in EXPECTED_INTERVALS.items():
            expected = list(range(interval, 301, interval))
            assert pulse_cycles[signal_name] == expected
    finally:
        clock_task.cancel()


@cocotb.test()
async def reset_clears_timestamp_and_enable_phase(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    try:
        await reset_dut(dut)
        for _ in range(37):
            await RisingEdge(dut.clk)

        dut.rst_n.value = 0
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert dut.timestamp_cycles.value == 0
        for signal_name in EXPECTED_INTERVALS:
            assert getattr(dut, signal_name).value == 0

        dut.rst_n.value = 1
        for cycle in range(1, 101):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
            expected_10hz = 1 if cycle == 100 else 0
            assert dut.enable_10hz.value == expected_10hz
    finally:
        clock_task.cancel()
