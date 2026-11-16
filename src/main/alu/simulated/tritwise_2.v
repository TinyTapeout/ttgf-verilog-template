module tritwise_2 (
    input  wire [1:0] bin_in_a,
    input  wire [1:0] bin_in_b,
    input  wire [2:0] control_sig,
    output reg  [1:0] bin_out
);
    localparam [1:0] T_NEG  = 2'b00; // -1
    localparam [1:0] T_POS  = 2'b01; // +1
    localparam [1:0] T_ZERO = 2'b10; //  0

    wire a_zero = bin_in_a[1];
    wire a_pos  = (~bin_in_a[1]) &  bin_in_a[0];
    wire a_neg  = (~bin_in_a[1]) & (~bin_in_a[0]);
    wire [1:0] a_norm = a_zero ? T_ZERO : bin_in_a;

    wire b_zero = bin_in_b[1];
    wire b_pos  = (~bin_in_b[1]) &  bin_in_b[0];
    wire b_neg  = (~bin_in_b[1]) & (~bin_in_b[0]);
    wire [1:0] b_norm = b_zero ? T_ZERO : bin_in_b;

    always @(*) begin
        case (control_sig)
            // 000: Balanced Ternary Invert (STI: -1 -> +1, 0 -> 0, +1 -> -1)
            3'b000: begin
                if (a_zero)     bin_out = T_ZERO;
                else if (a_pos) bin_out = T_NEG;
                else            bin_out = T_POS;
            end

            // 001: Cycle up (-1 -> 0, 0 -> +1, +1 -> -1)
            //these can technically be done with native since i have the macros on hand but i don't want to do things in half measures
            3'b001: begin
                if (a_neg)       bin_out = T_ZERO;
                else if (a_zero) bin_out = T_POS;
                else             bin_out = T_NEG;
            end

            // 010: Cycle down (-1 -> +1, 0 -> -1, +1 -> 0)
            3'b010: begin
                if (a_neg)       bin_out = T_POS;
                else if (a_zero) bin_out = T_NEG;
                else             bin_out = T_ZERO;
            end

            // 011: Absolute (-1 -> +1, 0 -> 0, +1 -> +1)
            3'b011: begin
                bin_out = a_zero ? T_ZERO : T_POS;
            end

            // 100: bias-1 (Pulls 0 towards -1: -1 -> -1, 0 -> -1, +1 -> +1)
            3'b100: begin
                bin_out = a_pos ? T_POS : T_NEG;
            end

            // 101: bias1 (Pulls 0 towards +1: -1 -> -1, 0 -> +1, +1 -> +1)
            3'b101: begin
                bin_out = a_neg ? T_NEG : T_POS;
            end

            // 110: Sign match / Ternary Multiply
            // (-1,-1)->+1, (+1,+1)->+1, (-1,+1)->-1, (+1,-1)->-1, 0 if either is 0
            3'b110: begin
                if (a_zero | b_zero)            bin_out = T_ZERO;
                else if (a_pos == b_pos)        bin_out = T_POS;
                else                            bin_out = T_NEG;
            end

            // 111: NC add (No-carry / Modulo-3 balanced ternary sum)
            3'b111: begin
                if (a_zero)                     bin_out = b_norm;
                else if (b_zero)                bin_out = a_norm;
                else if (a_pos & b_pos)         bin_out = T_NEG;  // +1 + +1 = -1 (carry +1 dropped)
                else if (a_neg & b_neg)         bin_out = T_POS;  // -1 + -1 = +1 (carry -1 dropped)
                else                            bin_out = T_ZERO; // +1 + -1 =  0
            end

            default: bin_out = T_ZERO;
        endcase
    end
endmodule