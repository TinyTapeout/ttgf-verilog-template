(* blackbox *)
module native_2i_1t_mux (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,

    //literally just a pass gate
    input select_a,
    inout trit_in_a,
    inout trit_in_b,
    inout trit_out
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule