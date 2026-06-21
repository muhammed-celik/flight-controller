import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


@cocotb.test()
async def test_blinky_toggles(dut):
    """Test that LED toggles after the expected count."""
    clk_freq = 12_000_000
    blink_hz = 1
    half_period_cycles = clk_freq // (2 * blink_hz)

    clock = Clock(dut.clk, 83, units="ns")  # ~12 MHz
    cocotb.start_soon(clock.start())

    dut.rst_n.value = 0
    await Timer(200, units="ns")
    dut.rst_n.value = 1

    initial_led = dut.led.value

    for _ in range(half_period_cycles):
        await RisingEdge(dut.clk)

    assert dut.led.value != initial_led, "LED should have toggled"


@cocotb.test()
async def test_blinky_reset(dut):
    """Test that reset clears the LED and counter."""
    clock = Clock(dut.clk, 83, units="ns")
    cocotb.start_soon(clock.start())

    dut.rst_n.value = 0
    await Timer(200, units="ns")

    assert dut.led.value == 0, "LED should be 0 during reset"

    dut.rst_n.value = 1
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    dut.rst_n.value = 0
    await Timer(200, units="ns")

    assert dut.led.value == 0, "LED should be 0 after re-assert reset"
