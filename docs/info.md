## How it works

This project implements an 8-bit PID controller. The controller compares a
desired setpoint against a feedback value, computes the error, and combines
proportional, integral, and derivative terms into a single control output.

The top-level module is `tt_um_pid_controller`. It uses the Tiny Tapeout pins as
follows:

- `ui_in[7:0]` is the setpoint input during normal operation.
- `uio_in[7:0]` is the feedback input.
- `uo_out[7:0]` is the saturated control output.
- `uio_oe[7:0]` is tied low, so the bidirectional pins are used only as inputs.

The first three clock cycles after reset load the gain settings from
`ui_in[3:0]`: Kp first, then Ki, then Kd. Each 4-bit gain code is converted to a
fixed-point gain value using the table in `pid_controller.v`. After those three
setup cycles, the module enters its operating state and treats `ui_in[7:0]` as
the live setpoint.

The controller output is unipolar. Negative PID results clamp to `0x00`, values
above the 8-bit range clamp to `0xff`, and in-range positive values are passed
to `uo_out[7:0]`. This makes the design suitable for one-direction actuators
such as a heater or a motor drive where zero means "do not drive harder" rather
than actively driving in the opposite direction.

## How to test

Run the cocotb RTL simulation from the `test` directory:

```sh
make -B
```

The testbench resets the design, loads Kp, Ki, and Kd, then applies a setpoint
and feedback value to a simple simulated plant. During the run it checks that
the feedback converges near the setpoint. It also writes a `tb.vcd` waveform and
an `observation_data_<timestamp>.csv` file with cycle, setpoint, feedback,
control signal, and error values.

To inspect the waveform:

```sh
gtkwave tb.vcd tb.gtkw
```

For gate-level simulation, first harden the design, copy the generated
gate-level netlist to `test/gate_level_netlist.v`, and run:

```sh
make -B GATES=yes
```

## External hardware

No external hardware is required for RTL simulation.

For use on a Tiny Tapeout demo board, the project needs external circuitry that
provides an 8-bit feedback value and accepts the 8-bit control output. A
practical closed-loop setup would typically include a sensor or ADC feeding the
feedback pins, plus an actuator driver controlled by `uo_out[7:0]`.
