(* blackbox *)
module native_12t_ladder (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    input r_enable,
    input w_enable,
    input full12t,
    input ladder_pump,
    input [1:0] position,
    // 00 for 4t at position 0
    // 01 for 4t at position 1
    // 10 for 4t at position 2
    // 11 as do nothing state
    inout [11:0] data_in_12T,
    inout [7:0]  data_in_8T,
    inout [3:0]  data_in_4T,
    inout [11:0] data_out_12T,
    inout [3:0]  data_out_4T
);

endmodule