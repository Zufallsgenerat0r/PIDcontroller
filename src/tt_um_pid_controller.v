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

assign uio_oe = 8'h00; //assign bidrectional as outputs.

 // List all unused inputs to prevent warnings
wire _unused = &{uio_out, ena, 1'b0};

pid_controller pid(
  .setpoint (ui_in[7:0]), 
  .feedback (uio_in[7:0]), 
  .clk (clk), 
  .rst_n (rst_n),
  .control_out (uo_out[7:0])
);

endmodule

module pid_controller(
    input wire clk,
    input wire rst_n,
    input wire [7:0] setpoint,     // Desired value
    input wire [7:0] feedback,  // Measured value
    output reg [7:0] control_out   // Control signal
);

    // Parameters for PID constants (these can be tuned as needed)
    parameter Kp = 8'd2;   // Proportional gain
    parameter Ki = 8'd0;   // Integral gain
    parameter Kd = 8'd0;   // Derivative gain

    // Internal signals
    reg signed [15:0] error;
    reg signed [15:0] prev_error;
    reg signed [15:0] integral;
    reg signed [15:0] derivative;
    reg signed [15:0] pid_output;

    wire [15:0] setpoint_extended = {8'b0, setpoint};
    wire [15:0] feedback_extended = {8'b0, feedback};

    // State machine states
    typedef enum reg [2:0] {
        RESET_STATE,
        CALC_ERROR,
        CALC_TERMS,
        UPDATE_OUTPUT,
        UPDATE_PREV
    } state_t;

    state_t current_state, next_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all internal states
            current_state <= RESET_STATE;
            error <= 16'd0;
            prev_error <= 16'd0;
            integral <= 16'd0;
            derivative <= 16'd0;
            pid_output <= 16'd0;
            control_out <= 8'd0;
        end else begin
            // State transition
            current_state <= next_state;

            case (current_state)
                RESET_STATE: begin
                    // Reset internal signals
                    error <= 16'd0;
                    prev_error <= 16'd0;
                    integral <= 16'd0;
                    derivative <= 16'd0;
                    pid_output <= 16'd0;
                    control_out <= 8'd0;
                end
                CALC_ERROR: begin
                    // Calculate error
                    error <= setpoint_extended - feedback_extended;
                end
                CALC_TERMS: begin
                    // Calculate Proportional term
                    integral <= integral + (Ki * error);
                    derivative <= error - prev_error;
                end
                UPDATE_OUTPUT: begin
                    reg signed [15:0] p_term;
                    reg signed [15:0] d_term;
                    p_term = Kp * error;
                    d_term = Kd * derivative;

                    // PID output calculation
                    pid_output <= p_term + (integral >>> 8) + d_term;

                    // Clamp output to 8-bit range (0 to 255)
                    if (pid_output > 255) begin
                        control_out <= 8'd255;
                    end else if (pid_output < 0) begin
                        control_out <= 8'd0;
                    end else begin
                        control_out <= pid_output[7:0];
                    end
                end
                UPDATE_PREV: begin
                    // Store current error as previous error for next cycle
                    prev_error <= error;
                end
                default: begin
                    error <= 16'd0;
                    prev_error <= 16'd0;
                    integral <= 16'd0;
                    derivative <= 16'd0;
                    pid_output <= 16'd0;
                    control_out <= 8'd0;
                end
            endcase
        end
    end

    always @(*) begin
        // Default next state
        next_state = current_state;

        case (current_state)
            RESET_STATE: next_state = CALC_ERROR;
            CALC_ERROR: next_state = CALC_TERMS;
            CALC_TERMS: next_state = UPDATE_OUTPUT;
            UPDATE_OUTPUT: next_state = UPDATE_PREV;
            UPDATE_PREV: next_state = CALC_ERROR;
            default: next_state = RESET_STATE;
        endcase
    end

endmodule