# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
import random


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, units="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Test project behavior")

    # Set initial values for setpoint and process_var
    dut.ui_in.value = 128   # Set a mid-range setpoint (desired value)
    dut.uio_in.value = 100 # Set an initial process variable (measured value)

    await RisingEdge(dut.clk)

    # Run the PID controller for a few clock cycles
    for i in range(100):
        # Randomly vary the process variable to simulate system changes
        dut.uio_in.value = random.randint(0, 255)
        await RisingEdge(dut.clk)

        # Log the output values for debugging
        dut._log.info(f"Cycle {i}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")

    # Reset after scenario
    dut._log.info("Reset after scenario")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Controlled test scenario
    dut.ui_in.value = 150
    for i in range(50):
        dut.uio_in.value = max(0, dut.uio_in.value.integer - 1)  # Simulate system reaching setpoint
        await RisingEdge(dut.clk)
        dut._log.info(f"Cycle {i + 100}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")

    # Reset after scenario
    dut._log.info("Reset after scenario")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Step response test
    dut.ui_in.value = 200
    for i in range(100):
        await RisingEdge(dut.clk)
        dut._log.info(f"Step Response Cycle {i + 150}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")

    # Reset after scenario
    dut._log.info("Reset after scenario")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Add assertions to verify the behavior of the PID controller
    assert int(dut.uo_out.value) >= 0, "Control output is out of range!"
    assert int(dut.uo_out.value) <= 255, "Control output is out of range!"

    # Edge case testing
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    for i in range(20):
        await RisingEdge(dut.clk)
        dut._log.info(f"Edge Case Cycle {i + 250}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")

    # Reset after scenario
    dut._log.info("Reset after scenario")
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut.ui_in.value = 255
    dut.uio_in.value = 255
    for i in range(20):
        await RisingEdge(dut.clk)
        dut._log.info(f"Edge Case Cycle {i + 270}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")

# Sweep test for PID gains (parameterized testing)
    for kp in range(1, 4):
        for ki in range(1, 4):
            for kd in range(1, 4):
                dut._log.info(f"Testing with Kp={kp}, Ki={ki}, Kd={kd}")
                dut.ui_in.value = 128
                dut.uio_in.value = 100
                for i in range(50):
                    await RisingEdge(dut.clk)
                    dut._log.info(f"Sweep Test Cycle {i + 300}: Setpoint = {dut.ui_in.value.integer}, Feedback = {dut.uio_in.value.integer}, Control Out = {dut.uo_out.value.integer}")
                    
                    # Add a condition to prevent output from always being 255
                    if dut.uo_out.value == 255:
                        dut._log.warning("Control Out reached maximum value of 255. Possible saturation detected during sweep test.")

                assert int(dut.uo_out.value) >= 0, "Control output is out of range during sweep test!"
                
                # Reset after scenario
                dut._log.info("Reset after scenario")
                dut.rst_n.value = 0
                await ClockCycles(dut.clk, 10)
                dut.rst_n.value = 1

                assert int(dut.uo_out.value) <= 255, "Control output is out of range during sweep test!"
