(* blackbox *)
//this is smaller by a bit
//this is to operate the trit shifter

module native_3i_8t_shift_mux (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    inout mux_line0, //flex that 3 level logic while you still can
    inout mux_line1,
    // Ternary Inputs (4 trits each)
    inout       in_res_sel_0,
    inout [3:0] in_data_0, 
    inout [2:0] in_shift_amt_0,
    //4t + 3 + 1

    inout       in_res_sel_1,
    inout [3:0] in_data_1, 
    inout [2:0] in_shift_amt_1,

    inout       in_res_sel_2,
    inout [3:0] in_data_2, 
    inout [2:0] in_shift_amt_2,

    inout       out_res_sel,
    inout [3:0] out_data, 
    inout [2:0] out_shift_amt,
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule