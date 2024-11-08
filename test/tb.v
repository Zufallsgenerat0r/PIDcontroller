`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a VCD file. You can view it with gtkwave.
  initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  wire [7:0] setpoint;
  wire [7:0] feedback;
  wire [7:0] control_out;

  // Replace tt_um_example with your module name:
  tt_um_pid_controller pid_controller (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .clk    (clk),      // clock
      .rst_n  (rst_n),     // not reset
      .ui_in (setpoint),
      .uio_in (feedback),
      .uo_out (control_out)
      );

endmodule
