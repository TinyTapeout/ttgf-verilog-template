module tritwise_3 (
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
            // 000: Get definite (Returns A if A is +1/-1, else returns B if A is 0)
            3'b000: begin
                bin_out = a_zero ? b_norm : a_norm;
            end

            //Uncertain flag (If either is 0, throws +1, else -1)
            3'b001: begin
                bin_out = (a_zero | b_zero) ? T_POS : T_NEG;
            end

            //datablock (If B is not 0, Res is 0; if B is 0, passes A)
            3'b010: begin
                bin_out = b_zero ? a_norm : T_ZERO;
            end

            //Ternary XOR (Opposite sign -> +1, Same non-zero sign -> -1, either 0 -> 0)
            3'b011: begin
                if (a_zero | b_zero)     bin_out = T_ZERO;
                else if (a_pos != b_pos) bin_out = T_POS;
                else                     bin_out = T_NEG;
            end

            // 100: Paradox gate
            // Both 0 -> +1 (non-paradox), Both -1 -> +1 (non-paradox),
            // Both +1 -> 0 (possible paradox), All other combinations -> 0
            3'b100: begin
                if ((a_zero & b_zero) | (a_neg & b_neg))
                    bin_out = T_POS;
                else
                    bin_out = T_ZERO;
            end

            // 101, 110, 111: Unused -> Default to 0
            default: begin
                //probably something else can be here but i genuinely don't care
                bin_out = T_ZERO;
            end
        endcase
    end
endmodule