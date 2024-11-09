`default_nettype none

module pid_controller(
    input wire clk,
    input wire rst_n,
    input wire [7:0] setpoint,
    input wire [7:0] feedback,
    output reg [7:0] control_out
);

    localparam FETCHING_KP = 2'b00;
    localparam FETCHING_KI = 2'b01;
    localparam FETCHING_KD = 2'b10;
    localparam OPERATING = 2'b11;

    reg [1:0] state; // Operating status

    reg [8:0] Kp;
    reg [8:0] Ki;
    reg [8:0] Kd;

    // Internal signals
    reg signed [8:0] error = 0;
    reg signed [8:0] prev_error = 0;
    reg signed [8:0] diff_error = 0;

    reg signed [15:0] integral = 16'h0000;
    reg signed [15:0] derivative = 16'h0000;
    reg signed [15:0] pid_output = 16'h0000;

    wire [24:0] raw_p_term;
    wire [40:0] raw_i_term;
    wire [24:0] raw_d_term;

    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            // Reset all terms
            error = 0;
            prev_error = 0;
            Kp <= 0;
            Ki <= 0;
            Kd <= 0;
            integral = 0;
            derivative = 0;
            control_out <= 0;
            state <= FETCHING_KP;
        end else begin
            case (state)
            FETCHING_KP: begin
                Kp <= pid_terms(setpoint[3:0]);
                state <= FETCHING_KD;
            end
            FETCHING_KI: begin
                Ki <= pid_terms(setpoint[3:0]);
                state <= FETCHING_KD;
            end
            FETCHING_KD: begin
                Kd <= pid_terms(setpoint[3:0]);
                state <= OPERATING;
            end
            OPERATING: begin
            // Calculate error
            error = setpoint - feedback;

            // Proportional term
            assign raw_p_term = Kp * error;

            pid_output = raw_p_term / 50;

            // Integral term
            raw_i_term = integral + (error * Ki);

            integral = raw_i_term / 50;
            pid_output = pid_output + integral;

            // Derivative term
            diff_error = error - prev_error;
            raw_d_term = Kd  * diff_error;
            derivative = raw_d_term / 50;
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

        default: begin
            // Must not happen, act as a reset
            // Reset all terms
            error = 0;
            prev_error = 0;
            Kp <= 0;
            Ki <= 0;
            Kd <= 0;
            integral = 0;
            derivative = 0;
            control_out <= 0;
            state <= FETCHING_KP;
            end
        endcase
    end
end

    // Function to replace pid_terms module
function [8:0] pid_terms;
    input [3:0] encoded_val;
    begin
        case (encoded_val)
            4'd0:  pid_terms = 9'd0;     // 0.0 * 50
            4'd1:  pid_terms = 9'd5;     // 0.1 * 50
            4'd2:  pid_terms = 9'd10;    // 0.2 * 50
            4'd3:  pid_terms = 9'd15;    // 0.3 * 50
            4'd4:  pid_terms = 9'd20;    // 0.4 * 50
            4'd5:  pid_terms = 9'd25;    // 0.5 * 50
            4'd6:  pid_terms = 9'd30;    // 0.6 * 50
            4'd7:  pid_terms = 9'd35;    // 0.7 * 50
            4'd8:  pid_terms = 9'd40;    // 0.8 * 50
            4'd9:  pid_terms = 9'd45;    // 0.9 * 50
            4'd10: pid_terms = 9'd50;    // 1.0 * 50
            4'd11: pid_terms = 9'd100;   // 2.0 * 50
            4'd12: pid_terms = 9'd150;   // 3.0 * 50
            4'd13: pid_terms = 9'd250;   // 5.0 * 50
            4'd14: pid_terms = 9'd350;   // 7.0 * 50
            4'd15: pid_terms = 9'd500;   // 10.0 * 50
            default: pid_terms = 9'd0;   // Default to 0.0 * 50
        endcase
    end
endfunction


endmodule