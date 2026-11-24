`default_nettype none
module top (
    input  wire clk_27m,       // 27 MHz board oscillator
    input  wire rst_n_btn,     // Active-low reset button

    // Physical HDMI TMDS outputs
    output wire Tx_0,
    output wire Tx_1,
    output wire Tx_2,
    output wire Tx_C
);

    // -------------------------------------------------------------------------
    // 1. Clocks & Reset
    // -------------------------------------------------------------------------
    wire clk_pixel;  // 55.8 MHz
    wire clk_serial; // 279 MHz
    wire reset_hi = ~rst_n_btn;

    HDMI_clock u_clocks (
        .clk        (clk_27m),
        .clk_pixel  (clk_pixel),
        .clk_serial (clk_serial)
    );

    // -------------------------------------------------------------------------
    // 2. Interconnects between Test Suite, ASIC, and HDMI Display
    // -------------------------------------------------------------------------
    wire        asic_clk;
    wire        asic_tick;
    wire        asic_rst_n;

    wire [7:0]  ui_in;
    wire [7:0]  uo_out;
    wire [7:0]  uio_in;
    wire [7:0]  uio_out;
    wire [7:0]  uio_oe;
    wire [7:0]  ua_dummy;

    wire [23:0] test_status;
    wire [7:0]  live_in_byte;
    wire [7:0]  live_out_byte;
    wire [7:0]  live_exp_byte;
    wire [7:0]  live_err_byte;
    wire [2:0]  live_mux_mode;
    wire [3:0]  live_bus_stat;

    // -------------------------------------------------------------------------
    // 3. HDMI 1024x768 Display + Slow ASIC Clock Generator
    // -------------------------------------------------------------------------
    DVI_asic_test #(
        .ASIC_DIV_BITS (23) // ~6.65 Hz slow ASIC clock (~150 ms/cycle)
    ) u_dvi (
        .serial_clk    (clk_serial),
        .pixel_clk     (clk_pixel),
        .reset         (reset_hi),
        .asic_clk      (asic_clk),
        .asic_tick     (asic_tick),
        .test_status   (test_status),
        .live_in_byte  (live_in_byte),
        .live_out_byte (live_out_byte),
        .live_exp_byte (live_exp_byte),
        .live_err_byte (live_err_byte),
        .live_mux_mode (live_mux_mode),
        .live_bus_stat (live_bus_stat),
        .Tx_0          (Tx_0),
        .Tx_1          (Tx_1),
        .Tx_2          (Tx_2),
        .Tx_C          (Tx_C)
    );

    // -------------------------------------------------------------------------
    // 4. ASIC Under Test (Single-Thread Mode 3'b010)
    // -------------------------------------------------------------------------
    tt_um_triple_t_devEclipse2 u_asic (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ua      (ua_dummy),
        .ena     (1'b1),
        .clk     (asic_clk),
        .rst_n   (rst_n_btn & asic_rst_n)
    );

    // -------------------------------------------------------------------------
    // 5. Swappable 8-Test Stimulus & Pass/Fail Bank
    //    Swap module name here to load a different set of 8 tests.
    // -------------------------------------------------------------------------
    asic_test_suite_bank0 u_test_suite (
        .pixel_clk     (clk_pixel),
        .asic_tick     (asic_tick),
        .rst_n         (rst_n_btn),

        // ASIC Pin Interface
        .asic_rst_n    (asic_rst_n),
        .ui_in         (ui_in),
        .uo_out        (uo_out),
        .uio_in        (uio_in),
        .uio_out       (uio_out),
        .uio_oe        (uio_oe),

        // HDMI Telemetry Interface
        .test_status   (test_status),
        .live_in_byte  (live_in_byte),
        .live_out_byte (live_out_byte),
        .live_exp_byte (live_exp_byte),
        .live_err_byte (live_err_byte),
        .live_mux_mode (live_mux_mode),
        .live_bus_stat (live_bus_stat)
    );

endmodule