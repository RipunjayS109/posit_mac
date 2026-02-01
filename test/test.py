# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)

    # After reset, output must be zero
    assert int(dut.uo_out.value) == 0

    # Apply first inputs
    dut.ui_in.value = 0x20
    dut.uio_in.value = 0x30
    await ClockCycles(dut.clk, 1)
    first = int(dut.uo_out.value)

    # Apply same inputs again (accumulation must change result)
    await ClockCycles(dut.clk, 1)
    second = int(dut.uo_out.value)

    dut._log.info(f"First = {first}, Second = {second}")

    assert second != first, "MAC did not accumulate"

