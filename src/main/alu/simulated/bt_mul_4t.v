module bt_mul_4t (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    // Multiply 4-trit vector by 1-trit scalar (+1: pass, -1: invert, 0: 8'hAA)
    function [7:0] scale_4t;
        input [7:0] v;
        input [1:0] s;
        integer k;
        reg [1:0] t;
        begin
            for (k = 0; k < 4; k = k + 1) begin
                t = v[k*2 +: 2];
                if (s == 2'b01)
                    scale_4t[k*2 +: 2] = (t == 2'b11) ? 2'b10 : t;
                else if (s == 2'b00)
                    scale_4t[k*2 +: 2] = (t == 2'b00) ? 2'b01 : (t == 2'b01) ? 2'b00 : 2'b10;
                else
                    scale_4t[k*2 +: 2] = 2'b10;
            end
        end
    endfunction

    // Shifted 8-trit partial products (padded with canonical 2'b10 zeros)
    wire [15:0] pp0 = {8'hAA, scale_4t(a, b[1:0])};
    wire [15:0] pp1 = {6'b101010, scale_4t(a, b[3:2]), 2'b10};
    wire [15:0] pp2 = {4'b1010,   scale_4t(a, b[5:4]), 4'b1010};
    wire [15:0] pp3 = {2'b10,     scale_4t(a, b[7:6]), 6'b101010};

    // Sum partial products using 8-trit ripple adders (two bt_add_4t per level)
    wire [15:0] s01, s23;
    wire [1:0]  c01, c23, cp, unused_c0, unused_c1, unused_c2;

    bt_add_4t u_s01_lo (.a(pp0[7:0]),  .b(pp1[7:0]),  .cin(2'b10), .sum(s01[7:0]),  .cout(c01));
    bt_add_4t u_s01_hi (.a(pp0[15:8]), .b(pp1[15:8]), .cin(c01),   .sum(s01[15:8]), .cout(unused_c0));

    bt_add_4t u_s23_lo (.a(pp2[7:0]),  .b(pp3[7:0]),  .cin(2'b10), .sum(s23[7:0]),  .cout(c23));
    bt_add_4t u_s23_hi (.a(pp2[15:8]), .b(pp3[15:8]), .cin(c23),   .sum(s23[15:8]), .cout(unused_c1));

    bt_add_4t u_sum_lo (.a(s01[7:0]),  .b(s23[7:0]),  .cin(2'b10), .sum(p[7:0]),    .cout(cp));
    bt_add_4t u_sum_hi (.a(s01[15:8]), .b(s23[15:8]), .cin(cp),    .sum(p[15:8]),   .cout(unused_c2));
endmodule