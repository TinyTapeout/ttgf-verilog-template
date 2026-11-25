module native_wrapper_sim (
    // inout  wire        VDD_1V65,
    input  wire [2:0]  active_cmd_t0,

    // Thread 0
    input  wire [7:0]  trit4_in_a_0,
    input  wire [7:0]  trit4_in_b_0,
    input  wire [7:0]  trit4_in_c_0,
    input  wire [2:0]  tritwise_control_0,
    input  wire [2:0]  shift_control_0,
    input  wire [1:0]  trit4_in_cin_0,
    input  wire [1:0]  trit1_in_b_0,
    output wire [7:0]  trit4_out_0,
    output wire [1:0]  trit4_out_cout_0,
    output wire [15:0] multiplier_out_0
);

    //constexpr guys
    localparam [1:0]  OV_ZERO_1T = 2'b10;
    localparam [7:0]  OV_ZERO_4T = {4{OV_ZERO_1T}}; // 8'b10_10_10_10
    localparam [15:0] OV_ZERO_8T = {8{OV_ZERO_1T}}; // 16'b1010_1010_1010_1010

    wire [7:0] bin_add3_sum;
    wire [1:0] bin_add3_cout;
    wire [7:0] add3_in_c = (active_cmd_t0 == 3'd6) ? trit4_in_c_0 : OV_ZERO_4T;

    bt_add3_4t adder3 (
        .a    (trit4_in_a_0),
        .b    (trit4_in_b_0),
        .c    (add3_in_c),
        .cin  (trit4_in_cin_0),
        .sum  (bin_add3_sum),
        .cout (bin_add3_cout)
    );

    // Output selection (both 3'd1 and 3'd6 use adder3)

    //hello everybody my name is multiplier
        wire [15:0] bin_mult_out;

    bt_mul_4t multiplier (
        .a (trit4_in_a_0),
        .b (trit4_in_b_0),
        .p (bin_mult_out)
    );
    //its so sad that tritwise died of ligma
    wire [1:0] bin_tw1_out, bin_tw2_out, bin_tw3_out;
        
        tritwise_1 u_tritwise_1 (
            .bin_in_a    (trit4_in_cin_0),
            .bin_in_b    (trit1_in_b_0),
            .control_sig (tritwise_control_0),
            .bin_out     (bin_tw1_out)
        );

        tritwise_2 u_tritwise_2 (
            .bin_in_a    (trit4_in_cin_0),
            .bin_in_b    (trit1_in_b_0),
            .control_sig (tritwise_control_0),
            .bin_out     (bin_tw2_out)
        );

        tritwise_3 u_tritwise_3 (
            .bin_in_a    (trit4_in_cin_0),
            .bin_in_b    (trit1_in_b_0),
            .control_sig (tritwise_control_0),
            .bin_out     (bin_tw3_out)
        );

    //there be digital trit shifts
    //cause shifting trits and picking a 4-trit window (-1=Left, 0=Center, +1=Right)
    //is pure wire routing, doing this directly in 2-bit Override Ternary saves two B2T/T2B conversions
        wire [23:0] base_canvas = {OV_ZERO_4T, trit4_in_a_0, OV_ZERO_4T};
    reg  [23:0] shifted_canvas;

    always @(*) begin
        case (shift_control_0[1:0])
            2'd0: shifted_canvas = base_canvas;
            2'd1: shifted_canvas = shift_control_0[2] ? {base_canvas[21:0], OV_ZERO_1T}     : {OV_ZERO_1T, base_canvas[23:2]};
            2'd2: shifted_canvas = shift_control_0[2] ? {base_canvas[19:0], {2{OV_ZERO_1T}}} : {{2{OV_ZERO_1T}}, base_canvas[23:4]};
            2'd3: shifted_canvas = shift_control_0[2] ? {base_canvas[17:0], {3{OV_ZERO_1T}}} : {{3{OV_ZERO_1T}}, base_canvas[23:6]};
        endcase
    end

    wire [7:0] bin_shift_res = (trit4_in_cin_0[1]) ? shifted_canvas[15:8] :
                               (trit4_in_cin_0[0]) ? shifted_canvas[7:0]  :
                                                     shifted_canvas[23:16];
    // Output selection
    assign trit4_out_0 = (active_cmd_t0 == 3'd1 || active_cmd_t0 == 3'd6) ? bin_add3_sum :
                         (active_cmd_t0 == 3'd7)                          ? bin_shift_res : OV_ZERO_4T;

    assign trit4_out_cout_0 = (active_cmd_t0 == 3'd1 || active_cmd_t0 == 3'd6) ? bin_add3_cout :
                              (active_cmd_t0 == 3'd3)                          ? bin_tw1_out   :
                              (active_cmd_t0 == 3'd4)                          ? bin_tw2_out   :
                              (active_cmd_t0 == 3'd5)                          ? bin_tw3_out   : OV_ZERO_1T;

    assign multiplier_out_0 = (active_cmd_t0 == 3'd2) ? bin_mult_out : OV_ZERO_8T;

endmodule