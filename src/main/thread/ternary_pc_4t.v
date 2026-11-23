`default_nettype none
module ternary_pc_4t (
    input  wire       clk,
    input  wire       rst_n,       // Active-low asynchronous reset
    input  wire       inc_enable,  // High to increment PC
    input  wire [1:0] inc_step,    // Binary input: 01 (+1), 10 (+2), 11 (+3)
    input  wire       w_enable,    // High to overwrite PC with w_data (Jump/Branch)
    input  wire       r_enable,    // High to drive PC onto r_data bus
    input  wire [7:0] w_data,      // 4-trit (8-bit) branch target input (Override Ternary)
    output wire [7:0] r_data,      // 4-trit (8-bit) gated read output (Override Ternary)
    output wire       overflow     // Pulses high when wrapping (carry-out != 0)
);

    // Override Ternary Encoding:
    // 2'b00 = -1
    // 2'b01 = +1
    // 2'b10 =  0 (canonical zero; 2'b11 also decodes as 0 and sanitizes to 2'b10)
    localparam [1:0] TRIT_NEG = 2'b00;
    localparam [1:0] TRIT_POS = 2'b01;
    localparam [1:0] TRIT_ZRO = 2'b10;
    localparam [7:0] ZERO_4T  = {4{TRIT_ZRO}}; // 8'hAA

    // Internal 8-bit register holding 4 trits
    reg  [7:0] pc_reg;
    wire [7:0] pc_inc;
    wire [7:0] w_data_sanitized;

    // 1. Decode Binary step to Override Balanced Ternary value
    reg [7:0] inc_ternary;
    always @(*) begin
        case (inc_step)
            // Trit weights: 1, 3, 9, 27
            2'b01:   inc_ternary = {TRIT_ZRO, TRIT_ZRO, TRIT_ZRO, TRIT_POS}; // +1 (0, 0, 0, +1)
            2'b10:   inc_ternary = {TRIT_ZRO, TRIT_ZRO, TRIT_POS, TRIT_NEG}; // +2 (0, 0, +1, -1) -> 3 - 1 = 2
            2'b11:   inc_ternary = {TRIT_ZRO, TRIT_ZRO, TRIT_POS, TRIT_ZRO}; // +3 (0, 0, +1,  0) -> 3 + 0 = 3
            default: inc_ternary = ZERO_4T;                                  //  0
        endcase
    end

    // Carry wire array (5 stages, 2-bit each; base carry is Override Zero 2'b10)
    wire [1:0] carry [4:0];
    assign carry[0] = TRIT_ZRO;

    // 2. Combinational function for an Override Balanced Ternary Full Adder
    function [3:0] trit_add(input [1:0] a, input [1:0] b, input [1:0] cin);
        reg signed [2:0] val_a, val_b, val_cin, sum_val;
        begin
            // Convert 2-bit override trits to signed integers (2'b10 and 2'b11 both map to 0)
            val_a   = (a   == TRIT_POS) ? 3'sb001 : (a   == TRIT_NEG) ? 3'sb111 : 3'sb000;
            val_b   = (b   == TRIT_POS) ? 3'sb001 : (b   == TRIT_NEG) ? 3'sb111 : 3'sb000;
            val_cin = (cin == TRIT_POS) ? 3'sb001 : (cin == TRIT_NEG) ? 3'sb111 : 3'sb000;

            sum_val = val_a + val_b + val_cin;

            // Map integer sum back to {Cout, Sum} in Override Ternary
            case (sum_val)
                -3: trit_add = {TRIT_NEG, TRIT_ZRO}; // Cout = -1, Sum =  0
                -2: trit_add = {TRIT_NEG, TRIT_POS}; // Cout = -1, Sum = +1
                -1: trit_add = {TRIT_ZRO, TRIT_NEG}; // Cout =  0, Sum = -1
                 0: trit_add = {TRIT_ZRO, TRIT_ZRO}; // Cout =  0, Sum =  0
                 1: trit_add = {TRIT_ZRO, TRIT_POS}; // Cout =  0, Sum = +1
                 2: trit_add = {TRIT_POS, TRIT_NEG}; // Cout = +1, Sum = -1
                 3: trit_add = {TRIT_POS, TRIT_ZRO}; // Cout = +1, Sum =  0
                 default: trit_add = {TRIT_ZRO, TRIT_ZRO};
            endcase
        end
    endfunction

    // 3. Chain adders and per-trit write sanitization (2'b11 -> 2'b10)
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_trit_inc
            wire [3:0] add_out;
            assign add_out = trit_add(pc_reg[2*i+1 : 2*i], inc_ternary[2*i+1 : 2*i], carry[i]);
            assign pc_inc[2*i+1 : 2*i] = add_out[1:0];
            assign carry[i+1]          = add_out[3:2];

            assign w_data_sanitized[2*i + 1] = w_data[2*i + 1];
            assign w_data_sanitized[2*i]     = w_data[2*i] & ~w_data[2*i + 1];
        end
    endgenerate

    // Overflow flag when the highest carry out is non-zero (neither 2'b10 nor 2'b11)
    assign overflow = (carry[4][1] == 1'b0);

    // Sequential State Update
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_reg <= 8'h00; // reset to -40 for reasons
        end else if (w_enable) begin
            pc_reg <= w_data_sanitized;
        end else if (inc_enable) begin
            pc_reg <= pc_inc;
        end
    end

    // Read Output Gating (Pull to canonical 0 when disabled)
    assign r_data = r_enable ? pc_reg : ZERO_4T;

endmodule