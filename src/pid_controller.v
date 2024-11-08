`default_nettype none

module pid_controller(
    input wire clk,
    input wire rst_n,
    input wire [7:0] setpoint,
    input wire [7:0] feedback,
    output reg [7:0] control_out
);

    // Hardcoded PID coefficients
    parameter [3:0] Kp = 4'd2; // Example proportional gain
    parameter [3:0] Ki = 4'd1; // Example integral gain
    parameter [3:0] Kd = 4'd1; // Example derivative gain

    wire signed [8:0] Kp_ext = { {5{Kd[3]}}, Kp };
    wire signed [8:0] Ki_ext = { {5{Kd[3]}}, Ki };
    wire signed [8:0] Kd_ext = { {5{Kd[3]}}, Kd };


    // Internal signals
    reg signed [8:0] error = 0;
    reg signed [8:0] prev_error = 0;
    reg signed [8:0] diff_error = 0;

    reg signed [15:0] integral = 16'h0000;
    reg signed [15:0] derivative = 16'h0000;
    reg signed [15:0] pid_output = 16'h0000;

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            // Reset all terms
            prev_error <= 0;
            integral <= 0;
            control_out <= 0;
        end else begin
            // Calculate error
            error = setpoint - feedback;

            // Proportional term
            pid_output = Kp_ext * error;

            // Integral term with windup prevention
            integral = integral + (error / 4);
            // if (integral > 16'h7FFF) integral = 16'h7FFF; // Positive saturation
            // else if (integral < -16'h8000) integral = -16'h8000; // Negative saturation
            pid_output = pid_output + integral;

            // Derivative term
            diff_error = error - prev_error;
            derivative = Kd_ext  * diff_error;
            pid_output = pid_output + derivative;

            // Update previous error for the next derivative calculation
            prev_error = error;

            // Clamping the output to fit in 8 bits
            if (pid_output < 8'h00) begin
                control_out <= 8'h00;
            end else if ((pid_output >= 8'hFF) && (pid_output[15] == 1)) begin
                control_out <= 8'h00;
            end else if ((pid_output >= 8'hFF) && (pid_output[15] == 0)) begin
                control_out <= 8'hFF;
            end else begin
                control_out <= pid_output[7:0];
            end
        end
    end

endmodule