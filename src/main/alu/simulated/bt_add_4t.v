module bt_add_4t (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire [1:0] cin,
    output wire [7:0] sum,
    output wire [1:0] cout
);
    wire [1:0] c1, c2, c3;
    bt_fa_1t u0 (.a(a[1:0]), .b(b[1:0]), .cin(cin), .sum(sum[1:0]), .cout(c1));
    bt_fa_1t u1 (.a(a[3:2]), .b(b[3:2]), .cin(c1),  .sum(sum[3:2]), .cout(c2));
    bt_fa_1t u2 (.a(a[5:4]), .b(b[5:4]), .cin(c2),  .sum(sum[5:4]), .cout(c3));
    bt_fa_1t u3 (.a(a[7:6]), .b(b[7:6]), .cin(c3),  .sum(sum[7:6]), .cout(cout));
endmodule