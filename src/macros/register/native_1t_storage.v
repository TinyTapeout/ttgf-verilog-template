(* blackbox *)
module native_1t_storage (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    input r_enable,
    input w_enable,
    inout data_in,
    inout data_out
);
endmodule