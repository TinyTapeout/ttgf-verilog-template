(* keep_hierarchy = "true" *)
module native_wrapper (
    inout  wire        VDD_1V65,

    input  wire [2:0]  active_cmd_t0,
    input  wire [2:0]  active_cmd_t1,
    input  wire [2:0]  active_cmd_t2,

    input  wire [1:0]  adder_mux,
    input  wire [1:0]  multi_mux,
    input  wire [1:0]  shift_mux,
    input  wire [1:0]  tritwise_1_mux,
    input  wire [1:0]  tritwise_2_mux,
    input  wire [1:0]  tritwise_3_mux,

    // Thread 0 (2-bit Override Ternary per trit)
    input  wire [7:0]  trit4_in_a_0,
    input  wire [7:0]  trit4_in_b_0,
    input  wire [2:0]  tritwise_control_0,
    input  wire [2:0]  shift_control_0,
    input  wire [1:0]  trit4_in_cin_0,
    input  wire [1:0]  trit1_in_b_0,
    output wire [7:0]  trit4_out_0,
    output wire [1:0]  trit4_out_cout_0,
    output wire [15:0] multiplier_out_0,

    // Thread 1 (2-bit Override Ternary per trit)
    input  wire [7:0]  trit4_in_a_1,
    input  wire [7:0]  trit4_in_b_1,
    input  wire [2:0]  tritwise_control_1,
    input  wire [2:0]  shift_control_1,
    input  wire [1:0]  trit4_in_cin_1,
    input  wire [1:0]  trit1_in_b_1,
    output wire [7:0]  trit4_out_1,
    output wire [1:0]  trit4_out_cout_1,
    output wire [15:0] multiplier_out_1,

    // Thread 2 (2-bit Override Ternary per trit)
    input  wire [7:0]  trit4_in_a_2,
    input  wire [7:0]  trit4_in_b_2,
    input  wire [2:0]  tritwise_control_2,
    input  wire [2:0]  shift_control_2,
    input  wire [1:0]  trit4_in_cin_2,
    input  wire [1:0]  trit1_in_b_2,
    output wire [7:0]  trit4_out_2,
    output wire [1:0]  trit4_out_cout_2,
    output wire [15:0] multiplier_out_2
);

    //ternary representation constants
    localparam [1:0]  OV_ZERO_1T = 2'b10;
    localparam [7:0]  OV_ZERO_4T = {4{OV_ZERO_1T}}; // 8'b10_10_10_10
    localparam [15:0] OV_ZERO_8T = {8{OV_ZERO_1T}}; // 16'b1010_1010_1010_1010

    //4 trit full adder with mux. remember to construct 
    wire [7:0] bin_adder_a   = (adder_mux == 2'd0) ? trit4_in_a_0   :
                               (adder_mux == 2'd1) ? trit4_in_a_1   :
                               (adder_mux == 2'd2) ? trit4_in_a_2   : OV_ZERO_4T;

    wire [7:0] bin_adder_b   = (adder_mux == 2'd0) ? trit4_in_b_0   :
                               (adder_mux == 2'd1) ? trit4_in_b_1   :
                               (adder_mux == 2'd2) ? trit4_in_b_2   : OV_ZERO_4T;

    wire [1:0] bin_adder_cin = (adder_mux == 2'd0) ? trit4_in_cin_0 :
                               (adder_mux == 2'd1) ? trit4_in_cin_1 :
                               (adder_mux == 2'd2) ? trit4_in_cin_2 : OV_ZERO_1T;

    // Native ternary nets (strictly local point-to-point inside macro island!)
    wire [3:0] nat_adder_a, nat_adder_b, nat_adder_sum;
    wire       nat_adder_cin, nat_adder_cout;
    wire [7:0] bin_adder_sum;
    wire [1:0] bin_adder_cout;

    b2t_override_4t u_adder_b2t_a   (.VDD_1V65(VDD_1V65), .bin_in(bin_adder_a),   .trit_out(nat_adder_a));
    b2t_override_4t u_adder_b2t_b   (.VDD_1V65(VDD_1V65), .bin_in(bin_adder_b),   .trit_out(nat_adder_b));
    b2t_override_1t u_adder_b2t_cin (.VDD_1V65(VDD_1V65), .bin_in(bin_adder_cin), .trit_out(nat_adder_cin));

    native_full_adder_4t ternary_adder (
        .VDD_1V65       (VDD_1V65),
        .trit_in_a      (nat_adder_a),
        .trit_in_b      (nat_adder_b),
        .trit_carry_in  (nat_adder_cin),
        .trit_carry_out (nat_adder_cout),
        .trit_out       (nat_adder_sum)
    );

    t2b_override_4t u_adder_t2b_sum  (.VDD_1V65(VDD_1V65), .trit_in(nat_adder_sum),  .bin_out(bin_adder_sum));
    t2b_override_1t u_adder_t2b_cout (.VDD_1V65(VDD_1V65), .trit_in(nat_adder_cout), .bin_out(bin_adder_cout));


    //multiplier
    wire [7:0] bin_mult_a = (multi_mux == 2'd0) ? trit4_in_a_0 :
                            (multi_mux == 2'd1) ? trit4_in_a_1 :
                            (multi_mux == 2'd2) ? trit4_in_a_2 : OV_ZERO_4T;

    wire [7:0] bin_mult_b = (multi_mux == 2'd0) ? trit4_in_b_0 :
                            (multi_mux == 2'd1) ? trit4_in_b_1 :
                            (multi_mux == 2'd2) ? trit4_in_b_2 : OV_ZERO_4T;

    wire [3:0]  nat_mult_a, nat_mult_b;
    wire [7:0]  nat_mult_out;
    wire [15:0] bin_mult_out;

    b2t_override_4t u_mult_b2t_a (.VDD_1V65(VDD_1V65), .bin_in(bin_mult_a), .trit_out(nat_mult_a));
    b2t_override_4t u_mult_b2t_b (.VDD_1V65(VDD_1V65), .bin_in(bin_mult_b), .trit_out(nat_mult_b));

    native_multiplier ternary_multiplier (
        .VDD_1V65  (VDD_1V65),
        .trit_in_a (nat_mult_a),
        .trit_in_b (nat_mult_b),
        .trit_out  (nat_mult_out)
    );

    t2b_override_8t u_mult_t2b_out (.VDD_1V65(VDD_1V65), .trit_in(nat_mult_out), .bin_out(bin_mult_out));

    //due to time constraints, tritwise components are all simulated in standard binary
        //tragic i know

            wire [1:0] bin_tw2_a    = (tritwise_2_mux == 2'd0) ? trit4_in_cin_0     :
                                (tritwise_2_mux == 2'd1) ? trit4_in_cin_1     :
                                (tritwise_2_mux == 2'd2) ? trit4_in_cin_2     : OV_ZERO_1T;
        wire [1:0] bin_tw2_b    = (tritwise_2_mux == 2'd0) ? trit1_in_b_0       :
                                (tritwise_2_mux == 2'd1) ? trit1_in_b_1       :
                                (tritwise_2_mux == 2'd2) ? trit1_in_b_2       : OV_ZERO_1T;
        wire [2:0] bin_tw2_ctrl = (tritwise_2_mux == 2'd0) ? tritwise_control_0 :
                                (tritwise_2_mux == 2'd1) ? tritwise_control_1 :
                                (tritwise_2_mux == 2'd2) ? tritwise_control_2 : 3'd0;

        wire [1:0] bin_tw3_a    = (tritwise_3_mux == 2'd0) ? trit4_in_cin_0     :
                                (tritwise_3_mux == 2'd1) ? trit4_in_cin_1     :
                                (tritwise_3_mux == 2'd2) ? trit4_in_cin_2     : OV_ZERO_1T;
        wire [1:0] bin_tw3_b    = (tritwise_3_mux == 2'd0) ? trit1_in_b_0       :
                                (tritwise_3_mux == 2'd1) ? trit1_in_b_1       :
                                (tritwise_3_mux == 2'd2) ? trit1_in_b_2       : OV_ZERO_1T;
        wire [2:0] bin_tw3_ctrl = (tritwise_3_mux == 2'd0) ? tritwise_control_0 :
                                (tritwise_3_mux == 2'd1) ? tritwise_control_1 :
                                (tritwise_3_mux == 2'd2) ? tritwise_control_2 : 3'd0;

        wire [1:0] bin_tw1_a    = (tritwise_1_mux == 2'd0) ? trit4_in_cin_0     :
                                (tritwise_1_mux == 2'd1) ? trit4_in_cin_1     :
                                (tritwise_1_mux == 2'd2) ? trit4_in_cin_2     : OV_ZERO_1T;
        wire [1:0] bin_tw1_b    = (tritwise_1_mux == 2'd0) ? trit1_in_b_0       :
                                (tritwise_1_mux == 2'd1) ? trit1_in_b_1       :
                                (tritwise_1_mux == 2'd2) ? trit1_in_b_2       : OV_ZERO_1T;
        wire [2:0] bin_tw1_ctrl = (tritwise_1_mux == 2'd0) ? tritwise_control_0 :
                                (tritwise_1_mux == 2'd1) ? tritwise_control_1 :
                                (tritwise_1_mux == 2'd2) ? tritwise_control_2 : 3'd0;
        wire [1:0] bin_tw1_out;

        tritwise_1 u_tritwise_1 (
            .bin_in_a   (bin_tw1_a),
            .bin_in_b   (bin_tw1_b),
            .control_sig(bin_tw1_ctrl),
            .bin_out    (bin_tw1_out)
        );
        wire [1:0] bin_tw2_out;

        tritwise_2 u_tritwise_2 (
            .bin_in_a   (bin_tw2_a),
            .bin_in_b   (bin_tw2_b),
            .control_sig(bin_tw2_ctrl),
            .bin_out    (bin_tw2_out)
        );

        wire [1:0] bin_tw3_out;

        tritwise_3 u_tritwise_3 (
            .bin_in_a   (bin_tw3_a),
            .bin_in_b   (bin_tw3_b),
            .control_sig(bin_tw3_ctrl),
            .bin_out    (bin_tw3_out)
        );

    //digital trit shifting
        //cause shifting trits and picking a 4-trit window (-1=Left, 0=Center, +1=Right)
        //is pure wire routing, doing this directly in 2-bit Override Ternary saves two B2T/T2B conversions
        wire [7:0] bin_shift_data = (shift_mux == 2'd0) ? trit4_in_a_0      :
                                    (shift_mux == 2'd1) ? trit4_in_a_1      :
                                    (shift_mux == 2'd2) ? trit4_in_a_2      : OV_ZERO_4T;

        wire [1:0] bin_shift_sel  = (shift_mux == 2'd0) ? trit4_in_cin_0    :
                                    (shift_mux == 2'd1) ? trit4_in_cin_1    :
                                    (shift_mux == 2'd2) ? trit4_in_cin_2    : OV_ZERO_1T;

        // Fixed bug: connected to shift_control_0/1/2 instead of tritwise_control_0/1/2
        wire [2:0] bin_shift_amt  = (shift_mux == 2'd0) ? shift_control_0   :
                                    (shift_mux == 2'd1) ? shift_control_1   :
                                    (shift_mux == 2'd2) ? shift_control_2   : 3'd0;

        // Build 12-trit (24-bit) padded canvas initialized with Override Ternary Zeroes:
        // Center [15:8] holds the 4-trit input data; [23:16] is Left overflow, [7:0] is Right overflow
        wire [23:0] base_canvas = {OV_ZERO_4T, bin_shift_data, OV_ZERO_4T};
        reg  [23:0] shifted_canvas;

        always @(*) begin
            case (bin_shift_amt[1:0])
                2'd0: shifted_canvas = base_canvas;
                2'd1: shifted_canvas = bin_shift_amt[2] ? {base_canvas[21:0], OV_ZERO_1T} : {OV_ZERO_1T, base_canvas[23:2]};
                2'd2: shifted_canvas = bin_shift_amt[2] ? {base_canvas[19:0], {2{OV_ZERO_1T}}} : {{2{OV_ZERO_1T}}, base_canvas[23:4]};
                2'd3: shifted_canvas = bin_shift_amt[2] ? {base_canvas[17:0], {3{OV_ZERO_1T}}} : {{3{OV_ZERO_1T}}, base_canvas[23:6]};
            endcase
        end

        // Window Selector using Override Ternary on bin_shift_sel:
        // 2'b00 = -1 (Left [23:16]), 2'b01 = +1 (Right [7:0]), 2'b10 or 2'b11 = 0 (Center [15:8])
        wire [7:0] bin_shift_res = (bin_shift_sel[1])  ? shifted_canvas[15:8] : // 2'b10 or 2'b11 -> 0 (Center)
                                (bin_shift_sel[0])  ? shifted_canvas[7:0]  : // 2'b01          -> +1 (Right)
                                                        shifted_canvas[23:16]; // 2'b00          -> -1 (Left)
    
    //readout demuxing
        assign trit4_out_0 = (active_cmd_t0 == 3'd1) ? bin_adder_sum :
                            (active_cmd_t0 == 3'd7) ? bin_shift_res : OV_ZERO_4T;

        assign trit4_out_1 = (active_cmd_t1 == 3'd1) ? bin_adder_sum :
                            (active_cmd_t1 == 3'd7) ? bin_shift_res : OV_ZERO_4T;

        assign trit4_out_2 = (active_cmd_t2 == 3'd1) ? bin_adder_sum :
                            (active_cmd_t2 == 3'd7) ? bin_shift_res : OV_ZERO_4T;

        //(i exploded tritwise 4)
        // 1-Trit Output (Driven by Adder Carry-Out [cmd=1] or Tritwise 1..3 [cmd=3..5])
        assign trit4_out_cout_0 = (active_cmd_t0 == 3'd1) ? bin_adder_cout :
                                  (active_cmd_t0 == 3'd3) ? bin_tw1_out    :
                                  (active_cmd_t0 == 3'd4) ? bin_tw2_out    :
                                  (active_cmd_t0 == 3'd5) ? bin_tw3_out    : OV_ZERO_1T;

        assign trit4_out_cout_1 = (active_cmd_t1 == 3'd1) ? bin_adder_cout :
                                  (active_cmd_t1 == 3'd3) ? bin_tw1_out    :
                                  (active_cmd_t1 == 3'd4) ? bin_tw2_out    :
                                  (active_cmd_t1 == 3'd5) ? bin_tw3_out    : OV_ZERO_1T;

        assign trit4_out_cout_2 = (active_cmd_t2 == 3'd1) ? bin_adder_cout :
                                  (active_cmd_t2 == 3'd3) ? bin_tw1_out    :
                                  (active_cmd_t2 == 3'd4) ? bin_tw2_out    :
                                  (active_cmd_t2 == 3'd5) ? bin_tw3_out    : OV_ZERO_1T;

        // 8-Trit Multiplier Output (Driven by Multiplier [cmd=2])
        assign multiplier_out_0 = (active_cmd_t0 == 3'd2) ? bin_mult_out : OV_ZERO_8T;
        assign multiplier_out_1 = (active_cmd_t1 == 3'd2) ? bin_mult_out : OV_ZERO_8T;
        assign multiplier_out_2 = (active_cmd_t2 == 3'd2) ? bin_mult_out : OV_ZERO_8T;

endmodule