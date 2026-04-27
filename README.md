# PID Controller ASIC

![GDS](../../workflows/gds/badge.svg) ![Docs](../../workflows/docs/badge.svg) ![Test](../../workflows/test/badge.svg) ![FPGA](../../workflows/fpga/badge.svg)

An 8-bit PID controller designed for Tiny Tapeout. The design samples a setpoint
and feedback value, computes proportional, integral, and derivative terms, and
drives an 8-bit saturated control output.

- [Project datasheet documentation](docs/info.md)
- [Simulation testbench notes](test/README.md)

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Project overview

The top-level Tiny Tapeout wrapper is `tt_um_pid_controller`. It maps the
dedicated input pins to the controller setpoint, the bidirectional pins to the
feedback input path, and the dedicated output pins to the control signal.

After reset, the first three clock cycles load the PID gain settings from
`ui_in[3:0]`:

1. Kp
2. Ki
3. Kd

After those setup cycles, `ui_in[7:0]` is treated as the setpoint and
`uio_in[7:0]` is treated as the measured feedback value. The controller computes
the PID output once per clock and clamps the output into the unsigned 8-bit
range `0x00` to `0xff`.

## Pinout

| Tiny Tapeout pins | Direction | Function |
| --- | --- | --- |
| `ui_in[7:0]` | Input | Setpoint during operation; `ui_in[3:0]` loads Kp, Ki, and Kd during the first three cycles after reset |
| `uo_out[7:0]` | Output | Saturated control signal |
| `uio_in[7:0]` | Input | Feedback/process value |
| `uio_out[7:0]` | Output | Unused, tied low |
| `uio_oe[7:0]` | Output enable | `0`, so all bidirectional pins are inputs |

## Source files

- `src/tt_um_pid_controller.v` - Tiny Tapeout wrapper and pin mapping
- `src/pid_controller.v` - PID controller implementation

## Running the tests

The cocotb testbench drives the gain setup sequence, applies a setpoint and
feedback value, and checks that the simulated plant response converges near the
target value.

```sh
cd test
make -B
```

The testbench writes `tb.vcd` for waveform inspection and also logs observation
CSV files named `observation_data_<timestamp>.csv`.

## Building for Tiny Tapeout

This repository follows the Tiny Tapeout Verilog project layout:

- `info.yaml` contains project metadata, source file list, clock, tile count,
  top module, and pin names.
- `docs/info.md` contains the datasheet text used by the Tiny Tapeout docs
  workflow.
- `test/README.md` explains the local RTL and gate-level simulation flow.

When pushed to GitHub with the Tiny Tapeout workflows enabled, the docs action
generates the documentation preview and the GDS action hardens the design.
