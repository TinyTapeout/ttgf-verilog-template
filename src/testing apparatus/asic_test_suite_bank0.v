`default_nettype none
module asic_test_suite_bank0 (
    input  wire        pixel_clk,
    input  wire        asic_tick,     // 1-cycle pulse on rising edge of slow asic_clk
    input  wire        rst_n,

    // ASIC Interface
    output reg         asic_rst_n,
    output reg  [7:0]  ui_in,
    input  wire [7:0]  uo_out,
    output reg  [7:0]  uio_in,
    input  wire [7:0]  uio_out,
    input  wire [7:0]  uio_oe,

    // HDMI Display Telemetry
    output reg  [23:0] test_status,   // 8 x 3-bit (0=black, 1=blue, 2=green, 3=red, 4=yellow)
    output wire [7:0]  live_in_byte,
    output wire [7:0]  live_out_byte,
    output reg  [7:0]  live_exp_byte,
    output reg  [7:0]  live_err_byte,
    output wire [2:0]  live_mux_mode,
    output reg  [3:0]  live_bus_stat
);

    // Test status codes (match DVI_asic_test)
    localparam [2:0]
        T_UNTESTED = 3'd0,
        T_TESTING  = 3'd1,
        T_PASS     = 3'd2,
        T_FAIL     = 3'd3,
        T_TIMEOUT  = 3'd4;

    // Bus status bar color codes (match DVI_asic_test)
    localparam [3:0]
        BS_IDLE  = 4'd0,
        BS_HS    = 4'd1,
        BS_TX    = 4'd2,
        BS_RX    = 4'd3,
        BS_UCODE = 4'd4,
        BS_MEM   = 4'd5,
        BS_ERR   = 4'd6;

    // Reusable Protocol Engine Operations
    localparam [3:0]
        OP_DONE      = 4'd0, // End of test suite
        OP_RESET     = 4'd1, // Pulse asic_rst_n low for 4 slow cycles
        OP_CHK_IDLE  = 4'd2, // Verify Overlord & T0 idle status
        OP_OVL_WR24  = 4'd3, // Overlord 0101 (Write PC) with 24-bit arg_a
        OP_OVL_RD24  = 4'd4, // Overlord 0010 (Read PC) & compare with arg_exp
        OP_DIR_EXEC  = 4'd5, // Stream 3-byte Direct-Exec micro-op (arg_a = {hdr, b0, b1})
        OP_SHOW_REG  = 4'd6, // Wait for req_type==001 (Show Reg) & compare 24-bit with arg_exp
        OP_PASS_TEST = 4'd7; // Mark current test PASS and advance to next square

    // Live trace taps
    assign live_mux_mode = ui_in[2:0];
    assign live_in_byte  = uio_in;
    assign live_out_byte = (uio_oe != 8'h00) ? uio_out : uo_out;

    // -------------------------------------------------------------------------
    // SWAPPABLE TEST SEQUENCE TABLE (Bank 0: Tests 0..7)
    // Direct-Exec 24-bit packet format: { {2'b11, cmd[3:0], 2'b00}, {p0, p1}, {p2, 4'b0} }
    // -------------------------------------------------------------------------
    reg [5:0]  step_idx;
    reg [2:0]  step_test_id;
    reg [3:0]  step_op;
    reg [23:0] step_arg_a;
    reg [23:0] step_arg_exp;

    always @(*) begin
        step_test_id = 3'd0;
        step_op      = OP_DONE;
        step_arg_a   = 24'h000000;
        step_arg_exp = 24'h000000;

        case (step_idx)
            // --- TEST 0: Reset & Idle Check ---
            6'd0: begin step_test_id = 3'd0; step_op = OP_RESET;     end
            6'd1: begin step_test_id = 3'd0; step_op = OP_CHK_IDLE;  end
            6'd2: begin step_test_id = 3'd0; step_op = OP_PASS_TEST; end

            // --- TEST 1: Overlord PC Write + 2'b11 -> 2'b10 Sanitization Readback ---
            6'd3: begin step_test_id = 3'd1; step_op = OP_OVL_WR24;  step_arg_a   = 24'hFF_55_00; end
            6'd4: begin step_test_id = 3'd1; step_op = OP_OVL_RD24;  step_arg_exp = 24'hAA_55_00; end
            6'd5: begin step_test_id = 3'd1; step_op = OP_PASS_TEST; end

            // --- TEST 2: Direct-Exec 4t Imm Load (sp0_trib0 = 8'h55) + Show Reg 1 ---
            6'd6: begin step_test_id = 3'd2; step_op = OP_DIR_EXEC;  step_arg_a   = {8'b11_1011_00, 8'h55, 8'h00}; end // 1011: load 8'h55 -> 4t addr 0
            6'd7: begin step_test_id = 3'd2; step_op = OP_DIR_EXEC;  step_arg_a   = {8'b11_0001_00, 8'hD1, 8'h00}; end // 0001:1101 show reg 1
            6'd8: begin step_test_id = 3'd2; step_op = OP_SHOW_REG;  step_arg_exp = 24'hAA_AA_55; end
            6'd9: begin step_test_id = 3'd2; step_op = OP_PASS_TEST; end

            // --- TEST 3: Direct-Exec 12t Copy (reg 1 -> reg 6) + Show Reg 6 ---
            6'd10: begin step_test_id = 3'd3; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0010_00, 8'h01, 8'h60}; end // 0010:0000 copy 1 -> 6
            6'd11: begin step_test_id = 3'd3; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'hD6, 8'h00}; end // 0001:1101 show reg 6
            6'd12: begin step_test_id = 3'd3; step_op = OP_SHOW_REG; step_arg_exp = 24'hAA_AA_55; end
            6'd13: begin step_test_id = 3'd3; step_op = OP_PASS_TEST; end

            // --- TEST 4: ALU 4-Trit Add (sp0_trib0 [+40] + sp0_trib0 [+40] -> sp0_trib1) ---
            6'd14: begin step_test_id = 3'd4; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0011_00, 8'h00, 8'h20}; end // 0011: add r0 + r0 -> r1
            6'd15: begin step_test_id = 3'd4; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'hD1, 8'h00}; end // show reg 1
            6'd16: begin step_test_id = 3'd4; step_op = OP_SHOW_REG; step_arg_exp = 24'hAA_00_55; end // +40+40 = +80 -> sum=-1(00), cout=+1
            6'd17: begin step_test_id = 3'd4; step_op = OP_PASS_TEST; end

            // --- TEST 5: Single-Use Mask Load + Masked 4t Move ---
            6'd18: begin step_test_id = 3'd5; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'h72, 8'h00}; end // 0001:0111 4t single-use mask from r1 (8'h00 = -1)
            6'd19: begin step_test_id = 3'd5; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0010_00, 8'h41, 8'h40}; end // 0010:0100 masked move r0(55) -> r2 (expect 00)
            6'd20: begin step_test_id = 3'd5; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'hD1, 8'h00}; end // show reg 1
            6'd21: begin step_test_id = 3'd5; step_op = OP_SHOW_REG; step_arg_exp = 24'h00_00_55; end
            6'd22: begin step_test_id = 3'd5; step_op = OP_PASS_TEST; end

            // --- TEST 6: 12t Swap (reg 1 <-> reg 6) & Immediate Alias Read ---
            6'd23: begin step_test_id = 3'd6; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0010_00, 8'h51, 8'h60}; end // 0010:0101 swap 1 <-> 6
            6'd24: begin step_test_id = 3'd6; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'hD6, 8'h00}; end // show reg 6 (aliased)
            6'd25: begin step_test_id = 3'd6; step_op = OP_SHOW_REG; step_arg_exp = 24'h00_00_55; end
            6'd26: begin step_test_id = 3'd6; step_op = OP_PASS_TEST; end

            // --- TEST 7: Clear Scratchpads (0000:0111) + Verify Canonical Zero ---
            6'd27: begin step_test_id = 3'd7; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0000_00, 8'h70, 8'h00}; end // 0000:0111 clear scratchpads
            6'd28: begin step_test_id = 3'd7; step_op = OP_DIR_EXEC; step_arg_a   = {8'b11_0001_00, 8'hD1, 8'h00}; end // show reg 1
            6'd29: begin step_test_id = 3'd7; step_op = OP_SHOW_REG; step_arg_exp = 24'hAA_AA_AA; end
            6'd30: begin step_test_id = 3'd7; step_op = OP_PASS_TEST; end

            default: begin step_test_id = 3'd7; step_op = OP_DONE; end
        endcase
    end

    // -------------------------------------------------------------------------
    // Protocol Engine FSM (Advances once per slow asic_tick)
    // -------------------------------------------------------------------------
    reg [3:0]  sub_st;
    reg [1:0]  byte_i;
    reg [7:0]  watchdog;
    reg        hs_lvl;
    reg        halted;
    reg [23:0] rx_acc;

    wire [7:0] exp_byte_sel = (byte_i == 2'd0) ? step_arg_exp[7:0]  :
                              (byte_i == 2'd1) ? step_arg_exp[15:8] : step_arg_exp[23:16];

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            asic_rst_n    <= 1'b0;
            ui_in         <= 8'h00;
            uio_in        <= 8'h00;
            test_status   <= 24'd0; // All 8 squares black (untested)
            live_exp_byte <= 8'h00;
            live_err_byte <= 8'h00;
            live_bus_stat <= BS_IDLE;
            step_idx      <= 6'd0;
            sub_st        <= 4'd0;
            byte_i        <= 2'd0;
            watchdog      <= 8'd0;
            hs_lvl        <= 1'b0;
            halted        <= 1'b0;
            rx_acc        <= 24'd0;
        end else if (asic_tick && !halted) begin
            live_exp_byte <= 8'h00;
            live_err_byte <= 8'h00;

            // Watchdog timeout (64 slow cycles per handshake)
            if (watchdog == 8'd64) begin
                test_status[step_test_id * 3 +: 3] <= T_TIMEOUT;
                live_bus_stat <= BS_ERR;
                halted        <= 1'b1;
            end else begin
                // Mark active test Blue if still Untested
                if (step_op != OP_DONE && test_status[step_test_id * 3 +: 3] == T_UNTESTED) begin
                    test_status[step_test_id * 3 +: 3] <= T_TESTING;
                end

                case (step_op)
                    // ---------------------------------------------------------
                    OP_RESET: begin
                        live_bus_stat <= BS_IDLE;
                        ui_in         <= 8'b0000_0010; // Park on Thread 0
                        if (sub_st < 4'd4) begin
                            asic_rst_n <= 1'b0;
                            sub_st     <= sub_st + 4'd1;
                        end else begin
                            asic_rst_n <= 1'b1;
                            sub_st     <= 4'd0;
                            step_idx   <= step_idx + 6'd1;
                        end
                    end

                    // ---------------------------------------------------------
                    OP_CHK_IDLE: begin
                        ui_in         <= 8'b0000_0010; // Thread 0
                        live_bus_stat <= BS_IDLE;
                        // Expect req_type==000, cur_full==0, data_phase==11 -> uo_out[6:1] == 6'b000_0_11
                        live_exp_byte <= 8'b0000_0110;
                        if (uo_out[6:1] == 6'b000011) begin
                            step_idx <= step_idx + 6'd1;
                        end else begin
                            live_err_byte <= uo_out ^ 8'b0000_0110;
                            test_status[step_test_id * 3 +: 3] <= T_FAIL;
                            live_bus_stat <= BS_ERR;
                            halted        <= 1'b1;
                        end
                    end

                    // ---------------------------------------------------------
                    // Overlord Write PC (0101) / Read PC (0010)
                    // ---------------------------------------------------------
                    OP_OVL_WR24, OP_OVL_RD24: begin
                        case (sub_st)
                            4'd0: begin // Issue cmd_opcode on ui_in[6:3] with mux=3'b001
                                ui_in         <= {1'b0, (step_op == OP_OVL_WR24 ? 4'b0101 : 4'b0010), 3'b001};
                                live_bus_stat <= BS_HS;
                                watchdog      <= watchdog + 8'd1;
                                if (uo_out[5]) begin // command_op_ack
                                    ui_in    <= {1'b0, 4'b0000, 3'b001};
                                    byte_i   <= 2'd0;
                                    watchdog <= 8'd0;
                                    sub_st   <= 4'd1;
                                end
                            end
                            4'd1: begin // Wait for bus_ready (uo_out[7])
                                ui_in         <= {1'b0, 4'b0000, 3'b001};
                                live_bus_stat <= (step_op == OP_OVL_WR24) ? BS_TX : BS_RX;
                                watchdog      <= watchdog + 8'd1;
                                if (uo_out[7]) begin
                                    if (step_op == OP_OVL_WR24) begin
                                        uio_in <= (byte_i == 2'd0) ? step_arg_a[7:0]  :
                                                  (byte_i == 2'd1) ? step_arg_a[15:8] : step_arg_a[23:16];
                                    end else begin
                                        live_exp_byte <= exp_byte_sel;
                                        live_err_byte <= uio_out ^ exp_byte_sel;
                                        if (uio_out != exp_byte_sel) begin
                                            test_status[step_test_id * 3 +: 3] <= T_FAIL;
                                            live_bus_stat <= BS_ERR;
                                            halted        <= 1'b1;
                                        end
                                    end
                                    ui_in    <= {1'b1, 4'b0000, 3'b001}; // fpga_ack = 1
                                    watchdog <= 8'd0;
                                    sub_st   <= 4'd2;
                                end
                            end
                            4'd2: begin // Drop fpga_ack, advance byte
                                ui_in <= {1'b0, 4'b0000, 3'b001};
                                if (byte_i == 2'd2) begin
                                    sub_st   <= 4'd0;
                                    step_idx <= step_idx + 6'd1;
                                end else begin
                                    byte_i <= byte_i + 2'd1;
                                    sub_st <= 4'd1;
                                end
                            end
                            default: sub_st <= 4'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    // Direct-Exec 3-Byte Stream (1-byte cmd header + 2-byte payload)
                    // ---------------------------------------------------------
                    OP_DIR_EXEC: begin
                        case (sub_st)
                            4'd0: begin // Start cmd byte (fpga_is_cmd=1): hs 0->1
                                ui_in         <= {3'd0, 1'b1, 1'b1, 3'b010};
                                live_bus_stat <= BS_HS;
                                watchdog      <= watchdog + 8'd1;
                                if (uo_out[2:1] == 2'b00) begin // data_phase == 00 (BUS_RX_WAIT)
                                    ui_in    <= {3'd0, 1'b1, 1'b0, 3'b010}; // hs 1->0 -> BUS_RX_STRM
                                    watchdog <= 8'd0;
                                    sub_st   <= 4'd1;
                                end
                            end
                            4'd1: begin // Toggle hs 0->1 to latch header byte
                                uio_in        <= step_arg_a[23:16];
                                ui_in         <= {3'd0, 1'b1, 1'b1, 3'b010};
                                live_bus_stat <= BS_TX;
                                sub_st        <= 4'd2;
                            end
                            4'd2: begin // Drop hs to 0 before payload transfer
                                ui_in         <= {3'd0, 1'b0, 1'b0, 3'b010};
                                live_bus_stat <= BS_IDLE;
                                sub_st        <= 4'd3;
                            end
                            4'd3: begin // Start 2-byte payload (fpga_is_cmd=0): hs 0->1
                                ui_in         <= {3'd0, 1'b0, 1'b1, 3'b010};
                                live_bus_stat <= BS_HS;
                                watchdog      <= watchdog + 8'd1;
                                if (uo_out[2:1] == 2'b00) begin
                                    ui_in    <= {3'd0, 1'b0, 1'b0, 3'b010}; // hs 1->0 -> BUS_RX_STRM
                                    watchdog <= 8'd0;
                                    sub_st   <= 4'd4;
                                end
                            end
                            4'd4: begin // Payload byte 0 (p0, p1): hs 0->1
                                uio_in        <= step_arg_a[15:8];
                                ui_in         <= {3'd0, 1'b0, 1'b1, 3'b010};
                                live_bus_stat <= BS_TX;
                                sub_st        <= 4'd5;
                            end
                            4'd5: begin // Payload byte 1 (p2, 0): hs 1->0
                                uio_in        <= step_arg_a[7:0];
                                ui_in         <= {3'd0, 1'b0, 1'b0, 3'b010};
                                live_bus_stat <= BS_TX;
                                sub_st        <= 4'd0;
                                step_idx      <= step_idx + 6'd1;
                            end
                            default: sub_st <= 4'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    // Wait for ASIC Show-Reg (req_type == 3'b001) & Check 3 Bytes
                    // ---------------------------------------------------------
                    OP_SHOW_REG: begin
                        case (sub_st)
                            4'd0: begin // Poll uo_out[6:4] == 3'b001
                                ui_in         <= {3'd0, 1'b0, 1'b0, 3'b010};
                                live_bus_stat <= BS_IDLE;
                                watchdog      <= watchdog + 8'd1;
                                if (uo_out[6:4] == 3'b001) begin
                                    ui_in    <= {3'd0, 1'b0, 1'b1, 3'b010}; // hs 0->1 -> BUS_ASIC_TX
                                    byte_i   <= 2'd0;
                                    watchdog <= 8'd0;
                                    sub_st   <= 4'd1;
                                end
                            end
                            4'd1: begin // Sample byte_i (0..2) on uio_out
                                live_bus_stat <= BS_RX;
                                live_exp_byte <= exp_byte_sel;
                                live_err_byte <= uio_out ^ exp_byte_sel;
                                if (uio_out != exp_byte_sel) begin
                                    test_status[step_test_id * 3 +: 3] <= T_FAIL;
                                    live_bus_stat <= BS_ERR;
                                    halted        <= 1'b1;
                                end else if (byte_i == 2'd2) begin
                                    ui_in  <= {3'b111, 1'b0, 1'b0, 3'b010}; // phase=111 + toggle hs 1->0 (Done)
                                    sub_st <= 4'd2;
                                end else begin
                                    byte_i <= byte_i + 2'd1;
                                    ui_in  <= {{1'b0, (byte_i + 2'd1)}, 1'b0, 1'b1, 3'b010};
                                end
                            end
                            4'd2: begin // Return to idle
                                ui_in         <= {3'd0, 1'b0, 1'b0, 3'b010};
                                live_bus_stat <= BS_IDLE;
                                sub_st        <= 4'd0;
                                step_idx      <= step_idx + 6'd1;
                            end
                            default: sub_st <= 4'd0;
                        endcase
                    end

                    // ---------------------------------------------------------
                    OP_PASS_TEST: begin
                        test_status[step_test_id * 3 +: 3] <= T_PASS;
                        live_bus_stat <= BS_IDLE;
                        step_idx      <= step_idx + 6'd1;
                    end

                    // ---------------------------------------------------------
                    OP_DONE: begin
                        live_bus_stat <= BS_IDLE;
                        halted        <= 1'b1;
                    end

                    default: halted <= 1'b1;
                endcase
            end
        end
    end

endmodule