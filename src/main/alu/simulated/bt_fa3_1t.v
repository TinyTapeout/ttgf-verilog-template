module bt_fa3_1t (
    input  wire [1:0] a,
    input  wire [1:0] b,
    input  wire [1:0] c,
    input  wire [1:0] cin,
    output reg  [1:0] sum,
    output reg  [1:0] cout
);
    wire signed [3:0] va  = (a   == 2'b00) ? -4'sd1 : (a   == 2'b01) ? 4'sd1 : 4'sd0;
    wire signed [3:0] vb  = (b   == 2'b00) ? -4'sd1 : (b   == 2'b01) ? 4'sd1 : 4'sd0;
    wire signed [3:0] vc  = (c   == 2'b00) ? -4'sd1 : (c   == 2'b01) ? 4'sd1 : 4'sd0;
    wire signed [3:0] vci = (cin == 2'b00) ? -4'sd1 : (cin == 2'b01) ? 4'sd1 : 4'sd0;
    wire signed [3:0] tot = va + vb + vc + vci; // Range: -4 to +4

    always @(*) begin
        case (tot)
            -4'sd4:  begin cout = 2'b00; sum = 2'b00; end // -4 = -3 - 1
            -4'sd3:  begin cout = 2'b00; sum = 2'b10; end // -3 = -3 + 0
            -4'sd2:  begin cout = 2'b00; sum = 2'b01; end // -2 = -3 + 1
            -4'sd1:  begin cout = 2'b10; sum = 2'b00; end // -1 =  0 - 1
             4'sd0:  begin cout = 2'b10; sum = 2'b10; end //  0 =  0 + 0
             4'sd1:  begin cout = 2'b10; sum = 2'b01; end // +1 =  0 + 1
             4'sd2:  begin cout = 2'b01; sum = 2'b00; end // +2 = +3 - 1
             4'sd3:  begin cout = 2'b01; sum = 2'b10; end // +3 = +3 + 0
             4'sd4:  begin cout = 2'b01; sum = 2'b01; end // +4 = +3 + 1
            default: begin cout = 2'b10; sum = 2'b10; end
        endcase
    end
endmodule