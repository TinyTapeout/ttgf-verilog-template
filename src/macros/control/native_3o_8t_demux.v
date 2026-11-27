(* blackbox *)
module native_3o_8t_demux (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    inout mux_line0, //flex that 3 level logic while you still can
    inout mux_line1,

    inout [7:0] trit_in,
    // Ternary outputs (4 trits each)
    inout [7:0] trit_out_0,
    inout [7:0] trit_out_1,
    inout [7:0] trit_out_2,
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule