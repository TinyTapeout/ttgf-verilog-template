module tritwise_1 (
    input  wire [1:0] bin_in_a,
    input  wire [1:0] bin_in_b,
    input  wire [2:0] control_sig,
    output reg  [1:0] bin_out
);
    localparam [1:0] T_NEG  = 2'b00; // -1
    localparam [1:0] T_POS  = 2'b01; // +1
    localparam [1:0] T_ZERO = 2'b10; //  0

    //normalises the representation first for A and B
    wire a_zero = bin_in_a[1];
    wire a_pos  = (~bin_in_a[1]) &  bin_in_a[0];
    wire a_neg  = (~bin_in_a[1]) & (~bin_in_a[0]);
    wire [1:0] a_norm = a_zero ? T_ZERO : bin_in_a;

    wire b_zero = bin_in_b[1];
    wire b_pos  = (~bin_in_b[1]) &  bin_in_b[0];
    wire b_neg  = (~bin_in_b[1]) & (~bin_in_b[0]);
    wire [1:0] b_norm = b_zero ? T_ZERO : bin_in_b;

    wire a_eq_b = (a_norm == b_norm);
    wire a_gt_b = (a_pos & (b_zero | b_neg)) | (a_zero & b_neg);

    always @(*) begin
        case (control_sig)
            // 000: Max (Maximum out of the 2)
            3'b000: begin
                if (a_pos | b_pos)       bin_out = T_POS;
                else if (a_zero | b_zero) bin_out = T_ZERO;
                else                     bin_out = T_NEG;
            end

            // 001: Min (Minimum out of the 2)
            3'b001: begin
                if (a_neg | b_neg)       bin_out = T_NEG;
                else if (a_zero | b_zero) bin_out = T_ZERO;
                else                     bin_out = T_POS;
            end

            // 010: Consensus bool (Returns +1 if A == B, -1 if A != B)
            3'b010: begin
                bin_out = a_eq_b ? T_POS : T_NEG;
            end

            // 011: Consensus value (Returns the value if A == B, else 0)
            3'b011: begin
                bin_out = a_eq_b ? a_norm : T_ZERO;
            end

            // 100: Consensus false / Anti-consensus (Returns inverted value if A == B, else 0)
            // Note: If you want it to return A when A != B and 0 when A == B, swap to: (!a_eq_b ? a_norm : T_ZERO)
            3'b100: begin
                bin_out = !a_eq_b ? a_norm : T_ZERO;
            end

            // 101: Compare value (+1 if A > B, 0 if A == B, -1 if A < B)
            3'b101: begin
                if (a_eq_b)      bin_out = T_ZERO;
                else if (a_gt_b) bin_out = T_POS;
                else             bin_out = T_NEG;
            end

            // 110: Positive Ternary Invert (PTI: -1 -> +1, 0 -> +1, +1 -> -1)
            3'b110: begin
                bin_out = a_pos ? T_NEG : T_POS;
            end

            // 111: Negative Ternary Invert (NTI: -1 -> +1, 0 -> -1, +1 -> -1)
            3'b111: begin
                bin_out = a_neg ? T_POS : T_NEG;
            end

            default: bin_out = T_ZERO;
        endcase
    end
endmodule