/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_pid_controller (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

assign uio_oe = 8'b11111111; //assign bidrectional as outputs.

 // List all unused inputs to prevent warnings
wire _unused = &{ena, 1'b0};

pid_controller pid(
  .setpoint (ui_in[7:0]), 
  .feedback (uio_in[7:0]), 
  .clk (clk), 
  .rst_n (rst_n)
);

endmodule

module pid_controller(
  input wire [7:0] setpoint,
  input wire [7:0] feedback,
  input wire clk,
  input wire rst_n,
  output wire control_signal
);

output reg [7:0] control_signal;

// Hardcoded PID coefficients
parameter reg [7:0] Kp = 8'h10; // Example proportional gain
parameter reg [7:0] Ki = 8'h02; // Example integral gain
parameter reg [7:0] Kd = 8'h01; // Example derivative gain

// Internal signals
reg [7:0] prev_error = 8'h00;
reg [7:0] integral = 8'h00;
reg [7:0] derivative = 8'h00;

always @(posedge clk or negedge rst_n) begin
  if (~rst_n) begin
    prev_error <= 8'h00;
    integral <= 8'h00;
    derivative <= 8'h00;
    outputs <= 8'h00;
  end
  else begin
    // PID Calculation
    error = (setpoint - feedback);
    integral <= integral + (Ki * error);
    derivative <= Kd * (error - prev_error);
    reg [15:0] pid_output;
    pid_output = (Kp * error) + integral + derivative;
    // Calculate control signal
    control_signal = pid_output[15:0];
    prev_error <= error; // Update previous error term to feed it for derrivative term.
  end
end
endmodule
