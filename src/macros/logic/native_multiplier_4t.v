(* blackbox *)
module native_full_adder_4t (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    // Ternary Inputs (4 trits each)
    inout [3:0] trit_in_a,
    inout [3:0] trit_in_b,
    
    // Ternary Output
    inout [7:0] trit_out
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule