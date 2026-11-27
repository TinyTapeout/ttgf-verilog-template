(* blackbox *)
module native_4t4t4t (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    input r_enable,
    input w_enable,
    input full12t,
    input [1:0] position,
    // 00 do nothing
    // 01 for 4t at position 0
    // 10 for 4t at position 1
    // 11 for 4t at position 2
    inout [11:0] data_in_12T,
    inout [3:0]  data_in_4T,

    inout [11:0] data_out_12T,
    inout [3:0]  data_out_4T
);

endmodule