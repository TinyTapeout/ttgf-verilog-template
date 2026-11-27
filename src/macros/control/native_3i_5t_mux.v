(* blackbox *)
//this is smaller by a bit
module native_3i_5t_mux (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,

    inout mux_line0, //flex that 3 level logic while you still can
    inout mux_line1,
    // Ternary Inputs (4 trits each)
    inout trit_in_a_0,
    inout trit_in_b_0,
    inout [2:0] control_pass_in_0, //make sure this can also pass binary and ternary

    inout trit_in_a_1,
    inout trit_in_b_1,
    inout [2:0] control_pass_in_1,

    inout trit_in_a_2,
    inout trit_in_b_2,
    inout [2:0] control_pass_in_2,

    inout trit_out_a,
    inout trit_out_b,
    inout [2:0] control_pass_out
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule