(* blackbox *)
module native_shift_selector (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    inout selector, // -1 is left, 0 is center , 1 is right 

    inout [11:0] trit_in,
    inout [3:0]  trit_out
);
    // EMPTY: No logic here. 
    // OpenLane will route wires to these pins based on the bounding box in your LEF.
endmodule