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
    localparam OPERATING   = 2'b11;

    reg [1:0] state;

    reg [8:0] Kp;
    reg [8:0] Ki;
    reg [8:0] Kd;

    // Registered signals
    reg signed [8:0] prev_error;
    reg signed [15:0] integral;

    // Combinational signals
    reg signed [8:0] error;
    reg signed [8:0] diff_error;
    reg signed [15:0] proportional;
    reg signed [15:0] integral_next;
    reg signed [15:0] derivative;
    reg signed [15:0] pid_output;

    // Function to replace pid_terms module
    function signed [8:0] pid_terms;
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
                default: pid_terms = 9'd0;
            endcase
        end
    endfunction

    // Combinational logic — compute PID terms from current inputs and state
    always @(*) begin
        error = setpoint - feedback;
        proportional = ($signed(Kp) * error) / $signed(50);
        integral_next = integral + (error * $signed(Ki)) / $signed(50);
        diff_error = error - prev_error;
        derivative = ($signed(Kd) * diff_error) / $signed(50);
        pid_output = proportional + integral_next + derivative;
    end

    // Sequential logic — all non-blocking assignments
    always @(posedge clk or negedge rst_n) begin
        if (~rst_n) begin
            prev_error <= 0;
            Kp <= 0;
            Ki <= 0;
            Kd <= 0;
            integral <= 0;
            control_out <= 0;
            state <= FETCHING_KP;
        end else begin
            case (state)
            FETCHING_KP: begin
                Kp <= pid_terms(setpoint[3:0]);
                state <= FETCHING_KI;
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
                prev_error <= error;
                integral <= integral_next;

                // Clamping the output to fit in 8 bits
                if (pid_output <= 16'h0000) begin
                    control_out <= 8'h00;
                end else if ((pid_output >= 16'h00FF) && (pid_output[15] == 1)) begin
                    control_out <= 8'h00;
                end else if ((pid_output >= 16'h00FF) && (pid_output[15] == 0)) begin
                    control_out <= 8'hFF;
                end else begin
                    control_out <= pid_output[7:0];
                end
            end
            default: begin
                prev_error <= 0;
                Kp <= 0;
                Ki <= 0;
                Kd <= 0;
                integral <= 0;
                control_out <= 0;
                state <= FETCHING_KP;
            end
            endcase
        end
    end

endmodule
