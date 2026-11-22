module bt_fa_1t (
    input  wire [1:0] a,
    input  wire [1:0] b,
    input  wire [1:0] cin,
    output reg  [1:0] sum,
    output reg  [1:0] cout
);
    wire signed [2:0] va = (a   == 2'b00) ? -3'sd1 : (a   == 2'b01) ? 3'sd1 : 3'sd0;
    wire signed [2:0] vb = (b   == 2'b00) ? -3'sd1 : (b   == 2'b01) ? 3'sd1 : 3'sd0;
    wire signed [2:0] vc = (cin == 2'b00) ? -3'sd1 : (cin == 2'b01) ? 3'sd1 : 3'sd0;
    wire signed [2:0] tot = va + vb + vc; // Range: -3 to +3

    always @(*) begin
        case (tot)
            -3'sd3:  begin cout = 2'b00; sum = 2'b10; end // -3 = -3 + 0
            -3'sd2:  begin cout = 2'b00; sum = 2'b01; end // -2 = -3 + 1
            -3'sd1:  begin cout = 2'b10; sum = 2'b00; end // -1 =  0 - 1
             3'sd0:  begin cout = 2'b10; sum = 2'b10; end //  0 =  0 + 0
             3'sd1:  begin cout = 2'b10; sum = 2'b01; end // +1 =  0 + 1
             3'sd2:  begin cout = 2'b01; sum = 2'b00; end // +2 = +3 - 1
             3'sd3:  begin cout = 2'b01; sum = 2'b10; end // +3 = +3 + 0
            default: begin cout = 2'b10; sum = 2'b10; end
        endcase
    end
endmodule