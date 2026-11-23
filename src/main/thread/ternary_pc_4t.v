`default_nettype none
module ternary_pc_4t (
    input  wire       clk,
    input  wire       rst_n,       // Active-low synchronous/asynchronous reset
    input  wire       inc_enable,  // High to increment PC
    input  wire [1:0] inc_step,    // Binary input: 01 (+1), 10 (+2), 11 (+3)
    input  wire       w_enable,    // High to overwrite PC with w_data (Jump/Branch)
    input  wire       r_enable,    // High to drive PC onto r_data bus
    input  wire [7:0] w_data,      // 4-trit (8-bit) branch target input
    output wire [7:0] r_data,      // 4-trit (8-bit) gated read output
    output wire       overflow     // Pulses high when wrapping (carry-out != 0)
);

    // Internal 8-bit register holding 4 trits
    reg  [7:0] pc_reg;
    wire [7:0] pc_inc;
    
    // 1. Decode Binary step to Balanced Ternary value
    reg [7:0] inc_ternary;
    always @(*) begin
        case (inc_step)
            // Trit weights: 1, 3, 9, 27
            2'b01: inc_ternary = 8'b00_00_00_10; // +1 (0, 0, 0, +1)
            2'b10: inc_ternary = 8'b00_00_10_01; // +2 (0, 0, +1, -1) -> 3 - 1 = 2
            2'b11: inc_ternary = 8'b00_00_10_00; // +3 (0, 0, +1,  0) -> 3 + 0 = 3
            default: inc_ternary = 8'b00_00_00_00; // 0
        endcase
    end

    // Carry wire array (5 stages, 2-bit each)
    wire [1:0] carry [4:0];
    assign carry[0] = 2'b00; // Initial base carry is 0

    // 2. Combinational function for a Balanced Ternary Full Adder
    function [3:0] trit_add(input [1:0] a, input [1:0] b, input [1:0] cin);
        reg signed [2:0] val_a, val_b, val_cin, sum_val;
        begin
            // Convert 2-bit trits to signed integers
            val_a   = (a == 2'b10) ? 3'sb001 : (a == 2'b01) ? 3'sb111 : 3'sb000;
            val_b   = (b == 2'b10) ? 3'sb001 : (b == 2'b01) ? 3'sb111 : 3'sb000;
            val_cin = (cin == 2'b10) ? 3'sb001 : (cin == 2'b01) ? 3'sb111 : 3'sb000;
            
            sum_val = val_a + val_b + val_cin;
            
            // Map integer sum back to {Cout, Sum} trits
            case (sum_val)
                -3: trit_add = {2'b01, 2'b00}; // Cout = -1, Sum =  0
                -2: trit_add = {2'b01, 2'b10}; // Cout = -1, Sum = +1
                -1: trit_add = {2'b00, 2'b01}; // Cout =  0, Sum = -1
                 0: trit_add = {2'b00, 2'b00}; // Cout =  0, Sum =  0
                 1: trit_add = {2'b00, 2'b10}; // Cout =  0, Sum = +1
                 2: trit_add = {2'b10, 2'b01}; // Cout = +1, Sum = -1
                 3: trit_add = {2'b10, 2'b00}; // Cout = +1, Sum =  0
                 default: trit_add = {2'b00, 2'b00};
            endcase
        end
    endfunction

    // 3. Chain adders together for 4 trits
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_trit_inc
            wire [3:0] add_out;
            assign add_out = trit_add(pc_reg[2*i+1 : 2*i], inc_ternary[2*i+1 : 2*i], carry[i]);
            assign pc_inc[2*i+1 : 2*i] = add_out[1:0];
            assign carry[i+1]          = add_out[3:2];
        end
    endgenerate

    // Overflow flag when the highest carry out is non-zero
    assign overflow = (carry[4] != 2'b00);

    // Sequential State Update 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to 0 (4 trits of 2'b00)
            pc_reg <= 8'b0;
        end else if (w_enable) begin
            pc_reg <= w_data;
        end else if (inc_enable) begin
            pc_reg <= pc_inc;
        end
    end

    // Read Output Gating (Pull-to-0 when disabled)
    assign r_data = r_enable ? pc_reg : 8'b0;

endmodule