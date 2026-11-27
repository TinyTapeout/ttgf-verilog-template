`default_nettype none
module top (
    input  wire       base_clock,
    input  wire       rst_n_btn,
    input  wire       HPD,
    output wire       tmds_clk_p,
    output wire [2:0] tmds_d_p
);

    // -------------------------------------------------------------------------
    // 1. Clocks (27 MHz -> 55.8 MHz pixel + 279 MHz serial)
    // -------------------------------------------------------------------------
    wire clk_pixel;
    wire clk_serial;

    HDMI_clock u_clocks (
        .clk        (base_clock),
        .clk_pixel  (clk_pixel),
        .clk_serial (clk_serial)
    );

    // -------------------------------------------------------------------------
    // 2. Combined Reset Synchronizer (rst_n_btn & HPD do the exact same thing)
    //    Holds reset low whenever EITHER rst_n_btn is pressed (0) OR HPD is low (0),
    //    then waits 255 pixel clocks after both are high before starting tests.
    // -------------------------------------------------------------------------
    reg [7:0] por_cnt  = 8'd0;
    reg [1:0] rst_sync = 2'b00;

    wire raw_run_en = rst_n_btn & HPD;

    always @(posedge clk_pixel) begin
        rst_sync <= {rst_sync[0], raw_run_en};
        if (!rst_sync[1]) begin
            por_cnt <= 8'd0;
        end else if (por_cnt != 8'hFF) begin
            por_cnt <= por_cnt + 8'd1;
        end
    end

    wire sys_rst_n = (por_cnt == 8'hFF);
    wire dvi_reset = ~sys_rst_n;

    // -------------------------------------------------------------------------
    // 3. Interconnects between Test Suite, ASIC, and HDMI Display
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
    // 4. HDMI 1024x768 Display + Slow ASIC Clock Generator
    // -------------------------------------------------------------------------
    DVI_asic_test #(
        .ASIC_DIV_BITS (23) // ~6.65 Hz slow ASIC clock (~150 ms/cycle)
    ) u_dvi (
        .serial_clk    (clk_serial),
        .pixel_clk     (clk_pixel),
        .reset         (dvi_reset),
        .asic_clk      (asic_clk),
        .asic_tick     (asic_tick),
        .test_status   (test_status),
        .live_in_byte  (live_in_byte),
        .live_out_byte (live_out_byte),
        .live_exp_byte (live_exp_byte),
        .live_err_byte (live_err_byte),
        .live_mux_mode (live_mux_mode),
        .live_bus_stat (live_bus_stat),
        .Tx_0          (tmds_d_p[0]),
        .Tx_1          (tmds_d_p[1]),
        .Tx_2          (tmds_d_p[2]),
        .Tx_C          (tmds_clk_p)
    );

    // -------------------------------------------------------------------------
    // 5. ASIC Under Test (Single-Thread Mode 3'b010)
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
        .rst_n   (sys_rst_n & asic_rst_n)
    );

    // -------------------------------------------------------------------------
    // 6. Swappable 8-Test Stimulus & Pass/Fail Bank
    // -------------------------------------------------------------------------
    asic_test_suite_bank0 u_test_suite (
        .pixel_clk     (clk_pixel),
        .asic_tick     (asic_tick),
        .rst_n         (sys_rst_n),
        .asic_rst_n    (asic_rst_n),
        .ui_in         (ui_in),
        .uo_out        (uo_out),
        .uio_in        (uio_in),
        .uio_out       (uio_out),
        .uio_oe        (uio_oe),
        .test_status   (test_status),
        .live_in_byte  (live_in_byte),
        .live_out_byte (live_out_byte),
        .live_exp_byte (live_exp_byte),
        .live_err_byte (live_err_byte),
        .live_mux_mode (live_mux_mode),
        .live_bus_stat (live_bus_stat)
    );

endmodule