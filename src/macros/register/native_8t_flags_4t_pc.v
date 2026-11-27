(* blackbox *)
module native_8t_flags_4t_pc (
    // Power & Mid-Rail
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif
    inout VDD_1V65,
    
    input r_enable,
    input w_enable,
    input full12t,
    input [3:0] position,
    // 0000 for 12T
    // 0001 for 4t at position 0
    // 0010 for 4t at position 1
    // 0011 for 4t at position 2
    // 1xxx for 8 1-trit flags

    inout [11:0] data_in_12T,
    inout [3:0]  data_in_4T,
    inout data_in_1T,

    inout [11:0] data_out_12T,
    inout [3:0]  data_out_4T,
    inout data_out_1T
);

//the blackbox macro for the 8 trit flags and the micro op action count
//
// what is it gotta do?
/*
it has to be able to take in 1 4 or 12 trits of data and cover it (assuming masked already)
it should also be able to be read in batches of 1 or 4 or 12

*/
endmodule