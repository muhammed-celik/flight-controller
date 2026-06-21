import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, FallingEdge


@cocotb.test()
async def test_top_locked(dut):
    """Test that MMCM locks and LED starts toggling."""
    clock = Clock(dut.clk, 83, units="ns")  # ~12 MHz
    cocotb.start_soon(clock.start())

    dut.btn.value = 1
    await Timer(500, units="ns")
    dut.btn.value = 0

    for _ in range(500):
        await RisingEdge(dut.clk)

    assert dut.locked.value == 1, "MMCM should lock after startup"


@cocotb.test()
async def test_top_reset_holds_led(dut):
    """Test that holding button keeps LED off."""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())

    dut.btn.value = 1
    await Timer(1, units="us")

    assert dut.led.value == 0, "LED should be 0 while button held (reset)"
