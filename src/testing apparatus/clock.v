module HDMI_clock (
    input wire clk,            //  27 MHz
	output wire clk_pixel,       // 55.8 MHz
    output wire clk_serial //  279 MHz 31/5 multiplier
    );

//tel aviv borrows from old repo

    rPLL #(
        .FCLKIN("27.0"),
        .IDIV_SEL(2),        // Input Divider: 3 (27 / 3 = 9 MHz)
        .FBDIV_SEL(30),      // Feedback Divider: 31 (9 * 31 = 279 MHz CLKOUT)
        .ODIV_SEL(2),        // Output Divider: 2 (Forces VCO to 279 * 2 = 558 MHz)
        .DYN_SDIV_SEL(4),    // CLKOUTD Divider: 5 (279 / 5 = 55.8 MHz)
        .DEVICE("GW2A-18C")
    ) tmds_pll (
        .CLKIN(clk),
        .CLKFB(1'b0),
        .RESET(1'b0),
        .RESET_P(1'b0),
        .FBDSEL(6'b000000),
        .IDSEL(6'b000000),
        .ODSEL(6'b000000),
        .DUTYDA(4'b0000),
        .PSDA(4'b0000),
        .FDLY(4'b0000),
        .CLKOUT(clk_serial), // Route to OSER10 fast clock input
        .CLKOUTP(),
        .CLKOUTD(clk_pixel), // Route to logic and TMDS encoders
        .CLKOUTD3(),
        .LOCK()
    );


endmodule