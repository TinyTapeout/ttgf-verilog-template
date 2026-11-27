/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_triple_t_devEclipse2 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    inout  wire [7:0] ua,       // ANALOG PINS (Direct to pads)
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);


  wire net_1v65 = ua[1];

  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, clk, rst_n, 1'b0};
/*
  // The (* keep *) attribute stops the synthesizer from deleting or renaming these wires.
    
    // 1.65V Mid-Rail (VDD/2) sourced from external pin ua[0]
    (* keep = "true" *) wire net_1v65 = ua[0];
    
    // Ternary routing from analog pads to the macro
    (* keep = "true" *) wire [3:0] net_trit_a  = ua[4:1];
    (* keep = "true" *) wire       net_carry_o = ua[5];
    
    // (Note: If you need more than 6 analog pins, you will need multiple tiles 
    // or to multiplex them, as a standard TT tile only exposes ua[5:0])

    // ==========================================
    // MACRO INSTANTIATION
    // ==========================================
    ternary_adder_4trit u_adder (
        // Global Power: 
        // Leave VDD_3V3 and VSS disconnected in the Verilog.
        // Your config.json "PDN_MACRO_CONNECTIONS" handles these automatically.
        .VDD_3V3(), 
        .VSS(),
        
        // Mid-Rail Power (Manual analog routing)
        .VDD_1V65(net_1v65),
        
        // Ternary Data
        .trit_in_a(net_trit_a),
        .trit_in_b(),           // Connect to other protected internal ternary nets
        .trit_carry_in(),       // Connect to other protected internal ternary nets
        .trit_carry_out(net_carry_o),
        .trit_out()             // Connect to other protected internal ternary nets
    );

*/
endmodule
