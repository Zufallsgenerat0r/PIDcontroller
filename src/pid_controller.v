`default_nettype none

module pid_controller(
    input wire clk,
    input wire rst_n,
    input wire [7:0] setpoint,
    input wire [7:0] feedback,
    output reg [7:0] control_out
);

    // Hardcoded PID coefficients
    parameter [7:0] Kp = 8'h02; // Example proportional gain
    parameter [7:0] Ki = 8'h01; // Example integral gain
    parameter [7:0] Kd = 8'h00; // Example derivative gain

    // Internal signals
    reg signed [8:0] error;
    reg signed [15:0] integral = 16'h0000;
    reg signed [15:0] derivative = 16'h0000;
    reg signed [15:0] pid_output = 16'h0000;
    reg signed [7:0] prev_error = 8'h00;

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            // Reset all terms
            prev_error <= 8'h00;
            integral <= 16'h0000;
            control_out <= 8'h00;
        end else begin
            // Calculate error
            error = setpoint - feedback;

            // Proportional term
            pid_output = Kp * error;

            // Integral term with windup prevention
            // integral = integral + (Ki * error);
            // if (integral > 16'h7FFF) integral = 16'h7FFF; // Positive saturation
            // else if (integral < -16'h8000) integral = -16'h8000; // Negative saturation
            // pid_output = pid_output + integral;

            // Derivative term
            derivative = Kd * (error - prev_error);
            pid_output = pid_output + derivative;

            // Update previous error for the next derivative calculation
            prev_error = error;

            // Clamping the output to fit in 8 bits
            if (pid_output < 8'h00) begin
                control_out <= 8'h00;
            end else if (pid_output > 8'hFF) begin
                control_out <= 8'h00;
            end else begin
                control_out <= pid_output[7:0];
            end
        end
    end

endmodule