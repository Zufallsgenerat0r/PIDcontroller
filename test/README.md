# Testbench for the PID controller Tiny Tapeout project

This testbench uses [cocotb](https://docs.cocotb.org/en/stable/) to drive the
PID controller and check the outputs. For more Tiny Tapeout testing background,
see the [Tiny Tapeout HDL testing guide](https://tinytapeout.com/hdl/testing/).

## Setting up

Install the Python requirements, then run the simulation from this directory:

```sh
pip install -r requirements.txt
```

The testbench is already configured for `tt_um_pid_controller`. The Makefile
builds these source files from `../src`:

- `tt_um_pid_controller.v`
- `pid_controller.v`

## How to run

To run the RTL simulation:

```sh
make -B
```

The test loads Kp, Ki, and Kd during the first three post-reset clock cycles,
then applies a setpoint and feedback value to a simple simulated plant. It
passes when the feedback converges near the setpoint.

To run gate-level simulation, first harden the project and copy the generated
gate-level Verilog netlist to `gate_level_netlist.v`.

Then run:

```sh
make -B GATES=yes
```

## Outputs

The RTL test writes:

- `tb.vcd` - waveform dump for GTKWave
- `observation_data_<timestamp>.csv` - cycle-by-cycle setpoint, feedback,
  control signal, and error values

## How to view the waveform

```sh
gtkwave tb.vcd tb.gtkw
```
