import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer, with_timeout


OKAY = 0b00
SLVERR = 0b10

ADDR_ID = 0x0000
ADDR_VERSION = 0x0004
ADDR_CAPABILITIES = 0x0008
ADDR_SCRATCH = 0x000C
ADDR_STATUS = 0x0010
ADDR_IRQ_STATUS = 0x0014
ADDR_IRQ_ENABLE = 0x0018
ADDR_IRQ_CLEAR = 0x001C
ADDR_TIME_CAPTURE = 0x0020
ADDR_TIME_LO = 0x0024
ADDR_TIME_HI = 0x0028
ADDR_TIME_SEQUENCE = 0x002C
ADDR_CFG_SHADOW_LO = 0x0030
ADDR_CFG_SHADOW_HI = 0x0034
ADDR_CFG_COMMIT = 0x0038
ADDR_CFG_ACTIVE_LO = 0x003C
ADDR_CFG_ACTIVE_HI = 0x0040
ADDR_CFG_SEQUENCE = 0x0044


def apply_strobes(current, update, strobes):
    result = current
    for byte_index in range(4):
        if strobes & (1 << byte_index):
            mask = 0xFF << (byte_index * 8)
            result = (result & ~mask) | (update & mask)
    return result & 0xFFFFFFFF


async def reset_dut(dut):
    dut.s_axi_awaddr.value = 0
    dut.s_axi_awprot.value = 0
    dut.s_axi_awvalid.value = 0
    dut.s_axi_wdata.value = 0
    dut.s_axi_wstrb.value = 0
    dut.s_axi_wvalid.value = 0
    dut.s_axi_bready.value = 0
    dut.s_axi_araddr.value = 0
    dut.s_axi_arprot.value = 0
    dut.s_axi_arvalid.value = 0
    dut.s_axi_rready.value = 0
    dut.time_cycles.value = 0
    dut.status_in.value = 0
    dut.irq_sources.value = 0
    dut.aresetn.value = 0
    await RisingEdge(dut.aclk)
    await RisingEdge(dut.aclk)
    dut.aresetn.value = 1
    await RisingEdge(dut.aclk)


async def drive_aw(dut, address):
    dut.s_axi_awaddr.value = address
    dut.s_axi_awvalid.value = 1
    while True:
        await RisingEdge(dut.aclk)
        if dut.s_axi_awready.value == 1:
            break
    dut.s_axi_awvalid.value = 0


async def drive_w(dut, data, strobes):
    dut.s_axi_wdata.value = data
    dut.s_axi_wstrb.value = strobes
    dut.s_axi_wvalid.value = 1
    while True:
        await RisingEdge(dut.aclk)
        if dut.s_axi_wready.value == 1:
            break
    dut.s_axi_wvalid.value = 0


async def axi_write(
    dut, address, data, strobes=0xF, order="simultaneous", response_stall=0
):
    dut.s_axi_bready.value = 0
    if order == "aw_first":
        await drive_aw(dut, address)
        await RisingEdge(dut.aclk)
        await drive_w(dut, data, strobes)
    elif order == "w_first":
        await drive_w(dut, data, strobes)
        await RisingEdge(dut.aclk)
        await drive_aw(dut, address)
    else:
        aw_task = cocotb.start_soon(drive_aw(dut, address))
        w_task = cocotb.start_soon(drive_w(dut, data, strobes))
        await aw_task
        await w_task

    await with_timeout(RisingEdge(dut.s_axi_bvalid), 1, "us")
    response = int(dut.s_axi_bresp.value)
    for _ in range(response_stall):
        previous = int(dut.s_axi_bresp.value)
        await RisingEdge(dut.aclk)
        assert dut.s_axi_bvalid.value == 1
        assert int(dut.s_axi_bresp.value) == previous

    dut.s_axi_bready.value = 1
    await RisingEdge(dut.aclk)
    dut.s_axi_bready.value = 0
    return response


async def axi_read(dut, address, response_stall=0):
    dut.s_axi_rready.value = 0
    dut.s_axi_araddr.value = address
    dut.s_axi_arvalid.value = 1
    while True:
        await RisingEdge(dut.aclk)
        if dut.s_axi_arready.value == 1:
            break
    dut.s_axi_arvalid.value = 0

    await with_timeout(RisingEdge(dut.s_axi_rvalid), 1, "us")
    data = int(dut.s_axi_rdata.value)
    response = int(dut.s_axi_rresp.value)
    for _ in range(response_stall):
        previous_data = int(dut.s_axi_rdata.value)
        previous_resp = int(dut.s_axi_rresp.value)
        await RisingEdge(dut.aclk)
        assert dut.s_axi_rvalid.value == 1
        assert int(dut.s_axi_rdata.value) == previous_data
        assert int(dut.s_axi_rresp.value) == previous_resp

    dut.s_axi_rready.value = 1
    await RisingEdge(dut.aclk)
    dut.s_axi_rready.value = 0
    return data, response


async def start_test(dut):
    clock = Clock(dut.aclk, 10, unit="ns")
    clock_task = cocotb.start_soon(clock.start())
    await reset_dut(dut)
    return clock_task


async def wait_for_rising_edge(signal):
    await RisingEdge(signal)


@cocotb.test()
async def identity_reset_and_access_errors(dut):
    clock_task = await start_test(dut)
    try:
        assert await axi_read(dut, ADDR_ID) == (0x46430001, OKAY)
        assert await axi_read(dut, ADDR_VERSION) == (0x00010000, OKAY)
        assert await axi_read(dut, ADDR_CAPABILITIES) == (0x3, OKAY)
        assert await axi_read(dut, ADDR_SCRATCH) == (0, OKAY)
        assert await axi_read(dut, ADDR_IRQ_ENABLE) == (0, OKAY)
        assert await axi_write(dut, ADDR_ID, 0xFFFFFFFF) == SLVERR
        assert await axi_write(dut, ADDR_SCRATCH + 1, 0x1) == SLVERR
        assert await axi_read(dut, 0x0080) == (0, SLVERR)
        assert await axi_read(dut, ADDR_IRQ_CLEAR) == (0, SLVERR)
    finally:
        clock_task.cancel()


@cocotb.test()
async def independent_channels_byte_strobes_and_backpressure(dut):
    clock_task = await start_test(dut)
    try:
        expected = 0
        transactions = [
            (0x11223344, 0xF, "aw_first", 3),
            (0xAABBCCDD, 0x5, "w_first", 2),
            (0x55AA00FF, 0xA, "simultaneous", 4),
        ]
        for data, strobes, order, stall in transactions:
            assert (
                await axi_write(
                    dut, ADDR_SCRATCH, data, strobes, order, response_stall=stall
                )
                == OKAY
            )
            expected = apply_strobes(expected, data, strobes)
            assert await axi_read(dut, ADDR_SCRATCH, response_stall=stall) == (
                expected,
                OKAY,
            )
    finally:
        clock_task.cancel()


@cocotb.test()
async def randomized_write_order_and_strobes(dut):
    clock_task = await start_test(dut)
    rng = random.Random(0xA71C0DE)
    expected = 0
    try:
        for _ in range(100):
            data = rng.getrandbits(32)
            strobes = rng.randrange(16)
            order = rng.choice(["aw_first", "w_first", "simultaneous"])
            stall = rng.randrange(4)
            response = await axi_write(
                dut, ADDR_SCRATCH, data, strobes, order, response_stall=stall
            )
            assert response == OKAY
            expected = apply_strobes(expected, data, strobes)
            read_data, read_response = await axi_read(
                dut, ADDR_SCRATCH, response_stall=rng.randrange(4)
            )
            assert read_response == OKAY
            assert read_data == expected
    finally:
        clock_task.cancel()


@cocotb.test()
async def coherent_time_snapshot(dut):
    clock_task = await start_test(dut)
    try:
        dut.time_cycles.value = 0x1122334455667788
        assert await axi_write(dut, ADDR_TIME_CAPTURE, 1) == OKAY
        dut.time_cycles.value = 0xFFEEDDCCBBAA0099

        assert await axi_read(dut, ADDR_TIME_LO) == (0x55667788, OKAY)
        assert await axi_read(dut, ADDR_TIME_HI) == (0x11223344, OKAY)
        assert await axi_read(dut, ADDR_TIME_SEQUENCE) == (1, OKAY)

        assert await axi_write(dut, ADDR_TIME_CAPTURE, 1, strobes=0x0) == OKAY
        assert await axi_read(dut, ADDR_TIME_SEQUENCE) == (1, OKAY)
        assert await axi_write(dut, ADDR_TIME_CAPTURE, 1) == OKAY
        assert await axi_read(dut, ADDR_TIME_SEQUENCE) == (2, OKAY)
    finally:
        clock_task.cancel()


@cocotb.test()
async def shadow_configuration_commits_atomically(dut):
    clock_task = await start_test(dut)
    try:
        assert await axi_write(dut, ADDR_CFG_SHADOW_LO, 0x89ABCDEF) == OKAY
        assert await axi_write(dut, ADDR_CFG_SHADOW_HI, 0x01234567) == OKAY
        assert int(dut.active_config.value) == 0
        assert await axi_read(dut, ADDR_CFG_ACTIVE_LO) == (0, OKAY)

        pulse = cocotb.start_soon(wait_for_rising_edge(dut.config_commit))
        assert await axi_write(dut, ADDR_CFG_COMMIT, 1) == OKAY
        await with_timeout(pulse, 1, "us")
        assert int(dut.active_config.value) == 0x0123456789ABCDEF
        assert await axi_read(dut, ADDR_CFG_SEQUENCE) == (1, OKAY)

        assert await axi_write(
            dut, ADDR_CFG_SHADOW_LO, 0xFFFF0000, strobes=0xC
        ) == OKAY
        assert await axi_read(dut, ADDR_CFG_SHADOW_LO) == (0xFFFFCDEF, OKAY)
        assert int(dut.active_config.value) == 0x0123456789ABCDEF
    finally:
        clock_task.cancel()


@cocotb.test()
async def interrupts_are_sticky_masked_and_write_one_to_clear(dut):
    clock_task = await start_test(dut)
    try:
        dut.irq_sources.value = 0x5
        await RisingEdge(dut.aclk)
        dut.irq_sources.value = 0
        await RisingEdge(dut.aclk)
        assert await axi_read(dut, ADDR_IRQ_STATUS) == (0x5, OKAY)
        assert dut.irq.value == 0

        assert await axi_write(dut, ADDR_IRQ_ENABLE, 0x1) == OKAY
        assert dut.irq.value == 1
        assert await axi_write(dut, ADDR_IRQ_CLEAR, 0x1) == OKAY
        assert await axi_read(dut, ADDR_IRQ_STATUS) == (0x4, OKAY)
        assert dut.irq.value == 0

        dut.irq_sources.value = 0x4
        assert await axi_write(dut, ADDR_IRQ_CLEAR, 0x4) == OKAY
        assert await axi_read(dut, ADDR_IRQ_STATUS) == (0x4, OKAY)
        dut.irq_sources.value = 0
        await RisingEdge(dut.aclk)
        assert await axi_write(dut, ADDR_IRQ_CLEAR, 0x4) == OKAY
        assert await axi_read(dut, ADDR_IRQ_STATUS) == (0, OKAY)
    finally:
        clock_task.cancel()


@cocotb.test()
async def reset_aborts_partial_and_stalled_transactions(dut):
    clock_task = await start_test(dut)
    try:
        await drive_aw(dut, ADDR_SCRATCH)
        dut.aresetn.value = 0
        await Timer(1, unit="ns")
        assert dut.s_axi_bvalid.value == 0
        await RisingEdge(dut.aclk)
        dut.aresetn.value = 1
        await RisingEdge(dut.aclk)

        await drive_w(dut, 0xDEADBEEF, 0xF)
        for _ in range(3):
            await RisingEdge(dut.aclk)
            assert dut.s_axi_bvalid.value == 0

        await drive_aw(dut, ADDR_SCRATCH)
        await with_timeout(RisingEdge(dut.s_axi_bvalid), 1, "us")
        dut.aresetn.value = 0
        await RisingEdge(dut.aclk)
        await Timer(1, unit="ps")
        assert dut.s_axi_bvalid.value == 0
        assert int(dut.active_config.value) == 0
    finally:
        clock_task.cancel()
