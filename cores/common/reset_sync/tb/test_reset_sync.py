import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


SYNC_STAGES = 3


@cocotb.test()
async def asserts_asynchronously_and_releases_synchronously(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    try:
        dut.arst_n.value = 0
        await Timer(1, unit="ns")
        assert dut.srst_n.value == 0

        dut.arst_n.value = 1
        for _ in range(SYNC_STAGES - 1):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
            assert dut.srst_n.value == 0

        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert dut.srst_n.value == 1

        await Timer(3, unit="ns")
        dut.arst_n.value = 0
        await Timer(1, unit="ps")
        assert dut.srst_n.value == 0
    finally:
        clock_task.cancel()


@cocotb.test()
async def short_async_reset_pulse_is_captured(dut):
    clock = Clock(dut.clk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    try:
        dut.arst_n.value = 1
        for _ in range(SYNC_STAGES):
            await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert dut.srst_n.value == 1

        await Timer(2, unit="ns")
        dut.arst_n.value = 0
        await Timer(1, unit="ns")
        assert dut.srst_n.value == 0
        dut.arst_n.value = 1

        for _ in range(SYNC_STAGES - 1):
            await RisingEdge(dut.clk)
            await Timer(1, unit="ps")
            assert dut.srst_n.value == 0
        await RisingEdge(dut.clk)
        await Timer(1, unit="ps")
        assert dut.srst_n.value == 1
    finally:
        clock_task.cancel()
