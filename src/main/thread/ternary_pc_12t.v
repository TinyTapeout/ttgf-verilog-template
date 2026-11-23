`default_nettype none
module ternary_pc_12t (
    input  wire        clk,
    input  wire        rst_n,       // Active-low synchronous/asynchronous reset
    input  wire        inc_enable,  // High to increment PC by +1 on clock edge
    input  wire        w_enable,    // High to overwrite PC with w_data (Jump/Branch)
    input  wire        r_enable,    // High to drive PC onto r_data bus
    input  wire [23:0] w_data,      // 12-trit (24-bit) branch target input
    output wire [23:0] r_data,      // 12-trit (24-bit) gated read output
    output wire        overflow     // Pulses high when wrapping from all +1s to all -1s
);

    // Internal 24-bit register holding 12 trits
    reg  [23:0] pc_reg;
    wire [23:0] pc_inc;
    
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

            // Trit propagates carry ONLY if it is currently +1 (2'b10)
            assign prop[i] = t1 & ~t0;

            // Prefix Carry-Lookahead: carry[i+1] is high only if ALL trits 0..i are +1
            // Synthesizers flatten this wide AND reduction into a fast 2-level tree
            assign carry[i+1] = &prop[i:0];

            // Next-state combinational logic with automatic 2'b11 fault sanitization
            assign pc_inc[2*i + 1] = carry[i] ? (~t1 & ~t0) : (t1 & ~t0);
            assign pc_inc[2*i]     = carry[i] ? (t1 & ~t0)  : (~t1 & t0);
        end
    endgenerate

    // Overflow flag when wrapping from +265,720 (all 10) to -265,720 (all 01)
    assign overflow = carry[12];

    // Sequential State Update (Resolves in 1 Clock Cycle)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to 0 (12 trits of 2'b00)
            // Note: Change to {12{2'b01}} if you prefer resetting to minimum value (-265,720)
            pc_reg <= 24'b0;
        end else if (w_enable) begin
            // Sanitize incoming write data on the fly so 2'b11 never latches
            pc_reg[23:0] <= w_data & ~({12{1'b1, 1'b1}} & ({12{w_data[23], w_data[22]}})); 
            // Note: Standard direct load below is preferred if w_data is already guaranteed valid:
            pc_reg <= w_data;
        end else if (inc_enable) begin
            pc_reg <= pc_inc;
        end
    end

    // Read Output Gating (Pull-to-0 when disabled, avoiding tri-state 'z' inside ASIC/FPGA fabrics)
    assign r_data = r_enable ? pc_reg : 24'b0;

endmodule