`default_nettype none
module DVI_asic_test #(
    // 55.8 MHz / 2^23 ~= 6.65 Hz ASIC clock (~150 ms/cycle).
    // Lower = faster (21 ~= 26.6 Hz), higher = slower (24 ~= 3.3 Hz).
    parameter integer ASIC_DIV_BITS = 23
)(
    input  wire        serial_clk,    // 279 MHz from HDMI_clock
    input  wire        pixel_clk,     // 55.8 MHz from HDMI_clock
    input  wire        reset,         // Active-high reset (matches your original DVI)

    // Slow clock & sample strobe out to your ASIC + stimulus FSM
    output wire        asic_clk,      // Slow clock driving tt_um_triple_t_devEclipse2
    output wire        asic_tick,     // 1 pixel_clk pulse on asic_clk rising edge

    // Top row: 8 tests x 3-bit status (0=black/untested, 1=blue/testing, 2=green/pass, 3=red/fail, 4=yellow/timeout)
    input  wire [23:0] test_status,

    // Live signals captured once per slow ASIC clock cycle (on falling edge after ASIC settles)
    input  wire [7:0]  live_in_byte,
    input  wire [7:0]  live_out_byte,
    input  wire [7:0]  live_exp_byte,
    input  wire [7:0]  live_err_byte,
    input  wire [2:0]  live_mux_mode, // ui_in[2:0]
    input  wire [3:0]  live_bus_stat, // Bus/handshake status color code

    // Physical TMDS outputs
    output wire        Tx_0,
    output wire        Tx_1,
    output wire        Tx_2,
    output wire        Tx_C
);

    // -------------------------------------------------------------------------
    // 1. Slow ASIC Clock Generator (from 55.8 MHz pixel_clk)
    // -------------------------------------------------------------------------
    reg [ASIC_DIV_BITS-1:0] div_cnt;
    reg                     asic_clk_prev;

    always @(posedge pixel_clk or posedge reset) begin
        if (reset) begin
            div_cnt       <= {ASIC_DIV_BITS{1'b0}};
            asic_clk_prev <= 1'b0;
        end else begin
            div_cnt       <= div_cnt + 1'b1;
            asic_clk_prev <= div_cnt[ASIC_DIV_BITS-1];
        end
    end

    assign asic_clk  = div_cnt[ASIC_DIV_BITS-1];
    assign asic_tick =  asic_clk && !asic_clk_prev; // Rising edge of slow clock
    wire   cap_tick  = !asic_clk &&  asic_clk_prev; // Falling edge: capture settled ASIC outputs

    // -------------------------------------------------------------------------
    // 2. 64-Sample Scrolling Trace Buffer (16 px/column across 1024 px)
    // -------------------------------------------------------------------------
    reg [7:0] buf_in   [0:63];
    reg [7:0] buf_out  [0:63];
    reg [7:0] buf_exp  [0:63];
    reg [7:0] buf_err  [0:63];
    reg [6:0] buf_stat [0:63]; // {live_mux_mode[2:0], live_bus_stat[3:0]}
    reg [5:0] wr_ptr;

    integer i;
    always @(posedge pixel_clk or posedge reset) begin
        if (reset) begin
            wr_ptr <= 6'd0;
            for (i = 0; i < 64; i = i + 1) begin
                buf_in[i]   <= 8'h00;
                buf_out[i]  <= 8'h00;
                buf_exp[i]  <= 8'h00;
                buf_err[i]  <= 8'h00;
                buf_stat[i] <= 7'h00;
            end
        end else if (cap_tick) begin
            buf_in[wr_ptr]   <= live_in_byte;
            buf_out[wr_ptr]  <= live_out_byte;
            buf_exp[wr_ptr]  <= live_exp_byte;
            buf_err[wr_ptr]  <= live_err_byte;
            buf_stat[wr_ptr] <= {live_mux_mode, live_bus_stat};
            wr_ptr           <= wr_ptr + 6'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 3. Beam Tracking (Your 55.8 MHz 1024x768 Reduced-Blanking Timing)
    // -------------------------------------------------------------------------
    localparam active_width  = 1024;
    localparam full_width    = 1184;
    localparam active_height = 768;
    localparam full_height   = 790;

    reg [11:0] h_cnt;
    reg [10:0] v_cnt;

    always @(posedge pixel_clk or posedge reset) begin
        if (reset) begin
            h_cnt <= 12'd0;
            v_cnt <= 11'd0;
        end else if (h_cnt == full_width - 1) begin
            h_cnt <= 12'd0;
            v_cnt <= (v_cnt == full_height - 1) ? 11'd0 : (v_cnt + 11'd1);
        end else begin
            h_cnt <= h_cnt + 12'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 4. 6-Band Pixel Renderer (128 px per band: 6 x 128 = 768)
    // -------------------------------------------------------------------------
    wire       next_de     = (h_cnt < active_width) && (v_cnt < active_height);
    wire [2:0] band        = v_cnt[9:7];          // 0=Tests, 1=IN, 2=OUT, 3=EXP, 4=ERR, 5=STATUS
    wire [2:0] test_idx    = h_cnt[9:7];          // 0..7 across 1024 px
    wire [5:0] col_idx     = wr_ptr + h_cnt[9:4]; // Oldest left, newest right
    wire [2:0] bit_idx     = 3'd7 - v_cnt[6:4];   // MSB at top of each strip
    wire       alt         = col_idx[0];          // Alternating column tint

    wire [2:0] cur_tstat   = test_status[test_idx * 3 +: 3];
    wire [7:0] cur_in      = buf_in[col_idx];
    wire [7:0] cur_out     = buf_out[col_idx];
    wire [7:0] cur_exp     = buf_exp[col_idx];
    wire [7:0] cur_err     = buf_err[col_idx];
    wire [6:0] cur_stat    = buf_stat[col_idx];

    wire is_band_div = (v_cnt[6:0] == 7'd0) || (v_cnt[6:0] == 7'd127);
    wire is_bit_div  = (v_cnt[3:0] == 4'd0);
    wire is_box_edge = (h_cnt[6:0] < 7'd8) || (h_cnt[6:0] >= 7'd120) ||
                       (v_cnt[6:0] < 7'd8) || (v_cnt[6:0] >= 7'd120);

    // 1 = Light Blue / Dark Blue, 0 = Grey / Black
    function [23:0] bit_color;
        input b;
        input a;
        begin
            case ({b, a})
                2'b10: bit_color = 24'h40_C0_FF; // 1 (even): Light Blue
                2'b11: bit_color = 24'h00_48_B0; // 1 (odd):  Dark Blue
                2'b00: bit_color = 24'h48_48_48; // 0 (even): Grey
                2'b01: bit_color = 24'h00_00_00; // 0 (odd):  Black
            endcase
        end
    endfunction

    reg [23:0] pix_rgb;
    always @(*) begin
        pix_rgb = 24'h00_00_00;
        if (next_de) begin
            case (band)
                3'd0: begin // Row 0: 8 Test Squares
                    if (is_box_edge) begin
                        pix_rgb = 24'h20_20_20;
                    end else begin
                        case (cur_tstat)
                            3'd0:    pix_rgb = 24'h00_00_00; // Untested: Black
                            3'd1:    pix_rgb = 24'h20_80_FF; // Testing:  Blue
                            3'd2:    pix_rgb = 24'h20_E0_40; // Pass:     Green
                            3'd3:    pix_rgb = 24'hF0_20_20; // Fail:     Red
                            3'd4:    pix_rgb = 24'hF0_D0_00; // Timeout:  Yellow
                            default: pix_rgb = 24'h80_00_80;
                        endcase
                    end
                end
                3'd1: pix_rgb = is_band_div ? 24'hFF_FF_FF :
                                is_bit_div  ? 24'h18_18_18 : bit_color(cur_in[bit_idx],  alt);
                3'd2: pix_rgb = is_band_div ? 24'hFF_FF_FF :
                                is_bit_div  ? 24'h18_18_18 : bit_color(cur_out[bit_idx], alt);
                3'd3: pix_rgb = is_band_div ? 24'hFF_FF_FF :
                                is_bit_div  ? 24'h18_18_18 : bit_color(cur_exp[bit_idx], alt);
                3'd4: begin // Flagged Error Bits
                    if (is_band_div)           pix_rgb = 24'hFF_FF_FF;
                    else if (is_bit_div)       pix_rgb = 24'h18_18_18;
                    else if (cur_err[bit_idx]) pix_rgb = alt ? 24'hB0_00_00 : 24'hFF_30_30;
                    else                       pix_rgb = alt ? 24'h00_00_00 : 24'h48_48_48;
                end
                3'd5: begin // Status Bar (Top 64px = Mux Mode, Bottom 64px = Bus State)
                    if (is_band_div || v_cnt[6:0] == 7'd64) begin
                        pix_rgb = 24'hFF_FF_FF;
                    end else if (!v_cnt[6]) begin
                        case (cur_stat[6:4])
                            3'b000:  pix_rgb = 24'h30_30_30; // Tester
                            3'b001:  pix_rgb = 24'h90_30_E0; // Overlord (Purple)
                            3'b010:  pix_rgb = 24'h00_90_90; // Thread 0 (Teal)
                            3'b101:  pix_rgb = 24'hFF_80_00; // Microcode (Orange)
                            default: pix_rgb = 24'h10_10_10;
                        endcase
                    end else begin
                        case (cur_stat[3:0])
                            4'd0:    pix_rgb = 24'h20_20_20; // Idle
                            4'd1:    pix_rgb = 24'hFF_E0_00; // Handshake (Yellow)
                            4'd2:    pix_rgb = 24'h00_E0_FF; // TX FPGA->ASIC (Cyan)
                            4'd3:    pix_rgb = 24'hFF_00_C0; // RX ASIC->FPGA (Magenta)
                            4'd4:    pix_rgb = 24'hFF_80_00; // Microcode Switch (Orange)
                            4'd5:    pix_rgb = 24'h40_FF_40; // Mem Turnaround (Lime)
                            4'd6:    pix_rgb = 24'hFF_20_20; // Error / Stall (Red)
                            default: pix_rgb = 24'h40_40_40;
                        endcase
                    end
                end
                default: pix_rgb = 24'h00_00_00;
            endcase
        end
    end

    reg       de, hsync, vsync;
    reg [7:0] red_8b, grn_8b, blu_8b;

    always @(posedge pixel_clk) begin
        de     <= next_de;
        hsync  <= (h_cnt >= 12'd1048) && (h_cnt < 12'd1080);
        vsync  <= (v_cnt >= 11'd771)  && (v_cnt < 11'd775);
        red_8b <= pix_rgb[23:16];
        grn_8b <= pix_rgb[15:8];
        blu_8b <= pix_rgb[7:0];
    end

    // -------------------------------------------------------------------------
    // 5. TMDS Encoders & Gowin OSER10 Serializers
    // -------------------------------------------------------------------------
    wire [9:0] tmds_red, tmds_green, tmds_blue;
    wire [9:0] tmds_clk = 10'b1111100000;

    TMDS_encoder enc_b (.clk(pixel_clk), .VD(blu_8b), .CD({vsync, hsync}), .VDE(de), .TMDS(tmds_blue));
    TMDS_encoder enc_g (.clk(pixel_clk), .VD(grn_8b), .CD(2'b00),          .VDE(de), .TMDS(tmds_green));
    TMDS_encoder enc_r (.clk(pixel_clk), .VD(red_8b), .CD(2'b00),          .VDE(de), .TMDS(tmds_red));

    OSER10 oser_0 (
        .Q(Tx_0), .FCLK(serial_clk), .PCLK(pixel_clk), .RESET(reset),
        .D0(tmds_blue[0]), .D1(tmds_blue[1]), .D2(tmds_blue[2]), .D3(tmds_blue[3]), .D4(tmds_blue[4]),
        .D5(tmds_blue[5]), .D6(tmds_blue[6]), .D7(tmds_blue[7]), .D8(tmds_blue[8]), .D9(tmds_blue[9])
    );

    OSER10 oser_1 (
        .Q(Tx_1), .FCLK(serial_clk), .PCLK(pixel_clk), .RESET(reset),
        .D0(tmds_green[0]), .D1(tmds_green[1]), .D2(tmds_green[2]), .D3(tmds_green[3]), .D4(tmds_green[4]),
        .D5(tmds_green[5]), .D6(tmds_green[6]), .D7(tmds_green[7]), .D8(tmds_green[8]), .D9(tmds_green[9])
    );

    OSER10 oser_2 (
        .Q(Tx_2), .FCLK(serial_clk), .PCLK(pixel_clk), .RESET(reset),
        .D0(tmds_red[0]), .D1(tmds_red[1]), .D2(tmds_red[2]), .D3(tmds_red[3]), .D4(tmds_red[4]),
        .D5(tmds_red[5]), .D6(tmds_red[6]), .D7(tmds_red[7]), .D8(tmds_red[8]), .D9(tmds_red[9])
    );

    OSER10 oser_c (
        .Q(Tx_C), .FCLK(serial_clk), .PCLK(pixel_clk), .RESET(reset),
        .D0(tmds_clk[0]), .D1(tmds_clk[1]), .D2(tmds_clk[2]), .D3(tmds_clk[3]), .D4(tmds_clk[4]),
        .D5(tmds_clk[5]), .D6(tmds_clk[6]), .D7(tmds_clk[7]), .D8(tmds_clk[8]), .D9(tmds_clk[9])
    );

endmodule