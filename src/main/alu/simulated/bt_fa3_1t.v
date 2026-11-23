module bt_fa3_1t (
    input  wire [1:0] a,
    input  wire [1:0] b,
    input  wire [1:0] c,
    input  wire [1:0] cin,
    output reg  [1:0] sum,
    output reg  [1:0] cout
);
    // Since 2'b10 and 2'b11 are both 0:
    //   is_neg = ~t[1] & ~t[0]  (only 2'b00)
    //   is_pos = ~t[1] &  t[0]  (only 2'b01)
    wire [2:0] n_neg = (~a[1] & ~a[0]) + (~b[1] & ~b[0]) + (~c[1] & ~c[0]) + (~cin[1] & ~cin[0]);
    wire [2:0] n_pos = (~a[1] &  a[0]) + (~b[1] &  b[0]) + (~c[1] &  c[0]) + (~cin[1] &  cin[0]);

    // Unsigned range [0..8] corresponding to [-4..+4]
    wire [3:0] idx = (4'd4 + n_pos) - n_neg;

    always @(*) begin
        case (idx)
            4'd0:    {cout, sum} = {2'b00, 2'b00}; // -4 = -3 - 1
            4'd1:    {cout, sum} = {2'b00, 2'b10}; // -3 = -3 + 0
            4'd2:    {cout, sum} = {2'b00, 2'b01}; // -2 = -3 + 1
            4'd3:    {cout, sum} = {2'b10, 2'b00}; // -1 =  0 - 1
            4'd4:    {cout, sum} = {2'b10, 2'b10}; //  0 =  0 + 0
            4'd5:    {cout, sum} = {2'b10, 2'b01}; // +1 =  0 + 1
            4'd6:    {cout, sum} = {2'b01, 2'b00}; // +2 = +3 - 1
            4'd7:    {cout, sum} = {2'b01, 2'b10}; // +3 = +3 + 0
            4'd8:    {cout, sum} = {2'b01, 2'b01}; // +4 = +3 + 1
            default: {cout, sum} = {2'b10, 2'b10}; // Safe canonical 0 fallback (never hits for valid logic)
        endcase
    end
endmodule