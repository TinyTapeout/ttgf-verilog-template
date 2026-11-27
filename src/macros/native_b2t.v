(* blackbox *)
//this has no warning flag and should NEVER be used outside of very controlled circumstances
module native_b2t (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    inout [1:0] binary_input,
    inout ternary_result
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule