module bt_fa_1t (
    input  wire [1:0] a,
    input  wire [1:0] b,
    input  wire [1:0] cin,
    output wire [1:0] sum,
    output wire [1:0] cout
);
    // Decode -1 (2'b00) and +1 (2'b01); both 2'b10 and 2'b11 evaluate to 0 (na=0, pa=0)
    wire na = ~a[1]   & ~a[0];
    wire nb = ~b[1]   & ~b[0];
    wire nc = ~cin[1] & ~cin[0];

    wire pa = ~a[1]   &  a[0];
    wire pb = ~b[1]   &  b[0];
    wire pc = ~cin[1] &  cin[0];

    // Carry-out: -1 if >=2 inputs are -1; +1 if >=2 inputs are +1; else 0 (2'b10)
    wire cout_neg = (na & nb) | (nb & nc) | (na & nc);
    wire cout_pos = (pa & pb) | (pb & pc) | (pa & pc);

    assign cout[1] = ~(cout_neg | cout_pos);
    assign cout[0] = cout_pos;

    // Sum is non-zero (not 2'b10) iff an odd number of inputs are non-zero (1 or 3),
    // excluding +/-3 (all three inputs equal to +1 or all three equal to -1)
    wire odd_nz = (~a[1]) ^ (~b[1]) ^ (~cin[1]);
    wire all_eq = (na & nb & nc) | (pa & pb & pc);
    wire sum_nz = odd_nz & ~all_eq;

    // Sum is +1 (2'b01) iff total is +1 (one +1, two 0s) or -2 (one +1, two -1s)
    wire sum_pos = (pa & ((b[1] & cin[1]) | (nb & nc))) |
                   (pb & ((a[1] & cin[1]) | (na & nc))) |
                   (pc & ((a[1] & b[1])   | (na & nb)));

    assign sum[1] = ~sum_nz;
    assign sum[0] = sum_pos;
endmodule