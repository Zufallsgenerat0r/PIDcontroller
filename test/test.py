# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge
import random

import csv
from datetime import datetime

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

# Specify the CSV file to write to
csv_file = f'observation_data_{timestamp}.csv'

# Check if this is the first time running the script or if the file needs to be initialized
try:
    with open(csv_file, 'x', newline='') as f:
        writer = csv.writer(f)
        # Write the header to the CSV file
        writer.writerow(['Cycle', 'Setpoint', 'Feedback', 'Control Signal', 'Error'])
except FileExistsError:
    pass  # File already exists, no need to write the header again

@cocotb.test()
async def test_pid_controller(dut):
    dut._log.info("Start")

    # 10ns period = 100MHz clock
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())

    # Initialize values
    setpoint = 100
    feedback = 0
    dut.setpoint.value = setpoint
    dut.feedback.value = feedback
    dut.rst_n.value = 0

    # Reset the DUT
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1

    # Simulated system response
    for i in range(200):  # Run for 150 cycles
        await RisingEdge(dut.clk)

        # Update the feedback based on control signal (simple simulation of plant response)
        pid_output = dut.control_out.value.integer

        # Adjust feedback value to simulate approach toward setpoint
        if pid_output > 0:
            feedback += min(pid_output, 6)  # Increase
        elif pid_output <= 0:
            feedback -= 2  # Slow decrease

        # Apply the updated feedback to DUT
        dut.feedback.value = feedback

        # Monitor the output and log values
        control_signal = dut.control_out.value.integer
        error = setpoint - feedback

        # Print for observation
        dut._log.info(f"Cycle {i}: Setpoint={setpoint}, Feedback={feedback}, "
                      f"Control Signal={control_signal}, Error={error}")
        
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([i, setpoint, feedback, control_signal, error])

        # Assertion to check if the feedback stabilizes around setpoint
        if i > 190:  # Give some settling time
            assert abs(feedback - setpoint) <= 5, f"Feedback did not converge: {feedback}"

    # Final check if feedback is close enough to setpoint
    assert abs(feedback - setpoint) <= 3, "PID controller did not reach setpoint adequately"
