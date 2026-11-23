`default_nettype none
module ternary_pc_12t (
    input  wire        clk,
    input  wire        rst_n,       // Active-low asynchronous reset
    input  wire        inc_enable,  // High to increment PC by +1 on clock edge
    input  wire        w_enable,    // High to overwrite PC with w_data (Jump/Branch)
    input  wire        r_enable,    // High to drive PC onto r_data bus
    input  wire [23:0] w_data,      // 12-trit (24-bit) branch target input (Override Ternary)
    output wire [23:0] r_data,      // 12-trit (24-bit) gated read output (Override Ternary)
    output wire        overflow     // Pulses high when wrapping from all +1s to all -1s
);

    // Override Ternary Encoding:
    // 2'b00 = -1
    // 2'b01 = +1
    // 2'b10 =  0 (canonical zero; 2'b11 also decodes as 0 and sanitizes to 2'b10)
    localparam [23:0] ZERO_12T = 24'h000000;

    // Internal 24-bit register holding 12 trits
    reg  [23:0] pc_reg;
    wire [23:0] pc_inc;
    wire [23:0] w_data_sanitized;

    // Carry and Propagate wires for 12 trits
    wire [11:0] prop;
    wire [12:0] carry;

    // Base carry-in is 1 for the +1 increment operation
    assign carry[0] = 1'b1;

    genvar i;
    generate
        for (i = 0; i < 12; i = i + 1) begin : gen_trit_inc
            wire t1 = pc_reg[2*i + 1];
            wire t0 = pc_reg[2*i];

            // Trit propagates carry ONLY if it is currently +1 (2'b01)
            assign prop[i] = ~t1 & t0;

            // Prefix Carry-Lookahead: carry[i+1] is high only if ALL trits 0..i are +1
            assign carry[i+1] = &prop[i:0];

            // Next-state combinational logic in Override Ternary (with 2'b11 -> 2'b10 sanitization):
            // carry[i] == 1 (+1): 00(-1)->10(0), 10/11(0)->01(+1), 01(+1)->00(-1)
            // carry[i] == 0 ( 0): 00(-1)->00(-1), 01(+1)->01(+1), 10/11(0)->10(0)
            assign pc_inc[2*i + 1] = carry[i] ? (~t1 & ~t0) : t1;
            assign pc_inc[2*i]     = carry[i] ? t1           : (~t1 & t0);

            // Sanitize incoming write data per trit so 2'b11 becomes 2'b10
            assign w_data_sanitized[2*i + 1] = w_data[2*i + 1];
            assign w_data_sanitized[2*i]     = w_data[2*i] & ~w_data[2*i + 1];
        end
    endgenerate

    // Overflow flag when wrapping from +265,720 (24'h555555) to -265,720 (24'h000000)
    assign overflow = carry[12];

    // Sequential State Update (Resolves in 1 Clock Cycle)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_reg <= ZERO_12T; // Reset to canonical 0 (24'hAAAAAA)
        end else if (w_enable) begin
            pc_reg <= w_data_sanitized;
        end else if (inc_enable) begin
            pc_reg <= pc_inc;
        end
    end

    // Read Output Gating (Pull to canonical 0 when disabled)
    assign r_data = r_enable ? pc_reg : ZERO_12T;

endmodule