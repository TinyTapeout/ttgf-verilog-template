// Minimal 2-trit half adder (a + b -> {cout, sum}, range [-2..+2])
module bt_ha_1t (
    input  wire [1:0] a,
    input  wire [1:0] b,
    output wire [1:0] sum,
    output wire [1:0] cout
);
    wire na = ~a[1] & ~a[0], nb = ~b[1] & ~b[0];
    wire pa = ~a[1] &  a[0], pb = ~b[1] &  b[0];

    // Carry is -1 if both -1; +1 if both +1; else 0 (2'b10)
    wire c_neg = na & nb;
    wire c_pos = pa & pb;
    assign cout = {~(c_neg | c_pos), c_pos};

    // Sum is non-zero iff exactly one input is non-zero OR both have same non-zero sign (+/-2 -> -/+1)
    wire sum_nz  = (~a[1] ^ ~b[1]) | c_neg | c_pos;
    wire sum_pos = (pa & b[1]) | (a[1] & pb) | c_neg; // +1 + 0, 0 + +1, or -1 + -1 (-2 = -3 + 1)
    assign sum   = {~sum_nz, sum_pos};
endmodule

module bt_mul_4t (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    // 2-gate per trit scalar multiply (2'b10 and 2'b11 both act as 0; outputs canonical 2'b10 for 0)
    function [7:0] scale_4t;
        input [7:0] v;
        input [1:0] s;
        integer k;
        reg z;
        begin
            for (k = 0; k < 4; k = k + 1) begin
                z = v[k*2 + 1] | s[1];
                scale_4t[k*2 +: 2] = {z, ~z & ~(v[k*2] ^ s[0])};
            end
        end
    endfunction

    wire [7:0] m0 = scale_4t(a, b[1:0]);
    wire [7:0] m1 = scale_4t(a, b[3:2]);
    wire [7:0] m2 = scale_4t(a, b[5:4]);
    wire [7:0] m3 = scale_4t(a, b[7:6]);

    // Trit 0: direct pass-through (0 adders)
    assign p[1:0] = m0[1:0];

    // Trit 1: 2 inputs (m0[1] + m1[0]) -> Half Adder
    wire [1:0] c1;
    bt_ha_1t u_t1 (.a(m0[3:2]), .b(m1[1:0]), .sum(p[3:2]), .cout(c1));

    // Trit 2: 3 inputs + c1 -> single 4-input adder (bt_fa3_1t)
    wire [1:0] c2;
    bt_fa3_1t u_t2 (.a(m0[5:4]), .b(m1[3:2]), .c(m2[1:0]), .cin(c1), .sum(p[5:4]), .cout(c2));

    // Trit 3: 4 inputs + c2 -> reduce 3 inputs with bt_fa_1t, then add 4th + c2 with bt_fa_1t
    wire [1:0] s3_a, c3_a, c3_b;
    bt_fa_1t u_t3a (.a(m0[7:6]), .b(m1[5:4]), .cin(m2[3:2]), .sum(s3_a), .cout(c3_a));
    bt_fa_1t u_t3b (.a(s3_a),    .b(m3[1:0]), .cin(c2),      .sum(p[7:6]), .cout(c3_b));

    // Trit 4: 3 inputs (m1[3], m2[2], m3[1]) + 2 carries (c3_a, c3_b)
    wire [1:0] s4_a, c4_a, c4_b;
    bt_fa_1t u_t4a (.a(m1[7:6]), .b(m2[5:4]), .cin(m3[3:2]), .sum(s4_a), .cout(c4_a));
    bt_fa_1t u_t4b (.a(s4_a),    .b(c3_a),    .cin(c3_b),    .sum(p[9:8]), .cout(c4_b));

    // Trit 5: 2 inputs (m2[3], m3[2]) + 2 carries (c4_a, c4_b) -> single bt_fa3_1t
    wire [1:0] c5;
    bt_fa3_1t u_t5 (.a(m2[7:6]), .b(m3[5:4]), .c(c4_a), .cin(c4_b), .sum(p[11:10]), .cout(c5));

    // Trit 6 & Trit 7: 1 input (m3[3]) + c5 -> Half Adder; carry-out IS Trit 7!
    bt_ha_1t u_t6 (.a(m3[7:6]), .b(c5), .sum(p[13:12]), .cout(p[15:14]));
endmodule