`timescale 1ns / 1ps
`default_nettype none
module ternary_to_unsigned_binary_logic_convert_4t (
    // 8 wires total: [7:6] for Trit 3 (MSB), down to [1:0] for Trit 0 (LSB)
    input  wire [7:0] in_trits,
    
    // 7-bit unsigned binary output (Range: 0 to 80)
    output wire [6:0] out_bin
);

    //updated mappings

    //now 10 / 11 = +1
    //00 = 0
    //01 = 2
    //checks if its tern0 first, puts that to d1. if not, checks the msb and assigns to either 2 or 0
    wire [1:0] u3 = (in_trits[7] == 1'b1) ? 2'd1 : ((in_trits[6] == 1'b1) ? 2'd2 : 2'd0);
    wire [1:0] u2 = (in_trits[5] == 1'b1) ? 2'd1 : ((in_trits[4] == 1'b1) ? 2'd2 : 2'd0);
    wire [1:0] u1 = (in_trits[3] == 1'b1) ? 2'd1 : ((in_trits[2] == 1'b1) ? 2'd2 : 2'd0);
    wire [1:0] u0 = (in_trits[1] == 1'b1) ? 2'd1 : ((in_trits[0] == 1'b1) ? 2'd2 : 2'd0);

    // --- Final Unsigned Adder Tree ---
    // Weights: 3^3=27, 3^2=9, 3^1=3, 3^0=1
    // Max possible sum: (2 * 27) + (2 * 9) + (2 * 3) + 2 = 54 + 18 + 6 + 2 = 80
    assign out_bin = (u3 * 7'd27) + (u2 * 7'd9) + (u1 * 7'd3) + u0;

endmodule