`default_nettype none
//the toppest level of all time
module tt_um_triple_t_devEclipse2 (
    `ifdef USE_POWER_PINS
    inout VDD_3V3,  // Hidden from Yosys, used by OpenROAD
    inout VSS,
    `endif

    input  wire [7:0] ui_in,    // Dedicated inputs (Pins 0-7)
    output wire [7:0] uo_out,   // Dedicated outputs (Pins 16-23)
    input  wire [7:0] uio_in,   // IOs: Input path (Pins 8-15)
    output wire [7:0] uio_out,  // IOs: Output path (Pins 8-15)
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 1=drive, 0=high-Z)
    // inout  wire [7:0] ua,
    input  wire       ena,      // Always 1 when powered
    input  wire       clk,      // Clock 
    input  wire       rst_n     // Reset (Active low)
);

    wire [2:0] mux_sel = ui_in[2:0];

    // Output busses from the distinct sub-modules
    wire [7:0] uo_000,      uo_001,      uo_111,      uo_def;
    wire [7:0] uio_out_000, uio_out_111, uio_out_def;
    wire [7:0] uio_oe_000,  uio_oe_111,  uio_oe_def;

    // Standalone mode (3'b111) placeholders
    assign uo_111      = 8'h00;
    assign uio_out_111 = 8'h00;
    assign uio_oe_111  = 8'h00;

//enable later
    // logic_tester testbench (
    //     .VDD_1V65 (ua[0]),
    //     .ui_in    (ui_in),
    //     .uo_out   (uo_000),
    //     .uio_in   (uio_in),
    //     .uio_out  (uio_out_000),
    //     .uio_oe   (uio_oe_000),
    //     .clk      (clk),
    //     .rst_n    (rst_n)
    // );

    wire [7:0] commander_tracker_data;
    wire       commander_tracker_enable;

    wire [2:0] pc_overflow_err_wire;
    wire [2:0] micro_op_action_ovf_err_wire;
    wire [2:0] micro_op_param_ovf_err_wire;
    wire [2:0] reg_overflow_err_wire;

    wire [1:0] thread_sel_wire;
    wire [1:0] reg_read_stage_wire;
    wire [4:0] reg_addr_wire;
    wire       is_read_wire;
    wire [2:0] reg_handover_done_wire;
    wire [7:0] ovl_dbg_dout_wire;

    wire       o_microcode_fence_t0_wire, o_microcode_fence_t1_wire, o_microcode_fence_t2_wire;
    wire       o_direct_instr_t0_wire,    o_direct_instr_t1_wire,    o_direct_instr_t2_wire;
    wire       single_execute_t0_wire,    single_execute_t1_wire,    single_execute_t2_wire;
    wire       single_execute_ack_t0_wire,single_execute_ack_t1_wire,single_execute_ack_t2_wire;
    wire       cmd_buf_clr_t0_wire,       cmd_buf_clr_t1_wire,       cmd_buf_clr_t2_wire;
    wire       cmd_buf_clr_ack_t0_wire,   cmd_buf_clr_ack_t1_wire,   cmd_buf_clr_ack_t2_wire;

tt_um_core_commander core_commander (
    .clk                     (clk),
    .rst_n                   (rst_n),
    .ui_in                   (ui_in),
    .uo_out                  (uo_001),
    .tracker                 (commander_tracker_data),
    .show_tracker            (commander_tracker_enable),

    // Per-Thread Error Inputs (b2t & microcode_illegal cut -> tied low)
    .pc_overflow_err         (pc_overflow_err_wire),
    .micro_op_action_ovf_err (micro_op_action_ovf_err_wire),
    .micro_op_param_ovf_err  (micro_op_param_ovf_err_wire),
    .reg_overflow_err        (reg_overflow_err_wire),

    // Register & Command Buffer Handover Interface
    .thread_select           (thread_sel_wire),
    .register_read_stage     (reg_read_stage_wire),
    .register_address        (reg_addr_wire),
    .is_read                 (is_read_wire),
    .reg_handover_done       (reg_handover_done_wire),

    // Per-Thread Execution Control & Handshakes
    .o_microcode_fence_t0    (o_microcode_fence_t0_wire),
    .o_microcode_fence_t1    (o_microcode_fence_t1_wire),
    .o_microcode_fence_t2    (o_microcode_fence_t2_wire),
    .o_direct_instr_t0       (o_direct_instr_t0_wire),
    .o_direct_instr_t1       (o_direct_instr_t1_wire),
    .o_direct_instr_t2       (o_direct_instr_t2_wire),
    .single_execute_t0       (single_execute_t0_wire),
    .single_execute_t1       (single_execute_t1_wire),
    .single_execute_t2       (single_execute_t2_wire),
    .single_execute_ack_t0   (single_execute_ack_t0_wire),
    .single_execute_ack_t1   (single_execute_ack_t1_wire),
    .single_execute_ack_t2   (single_execute_ack_t2_wire),
    .cmd_buf_clr_t0          (cmd_buf_clr_t0_wire),
    .cmd_buf_clr_t1          (cmd_buf_clr_t1_wire),
    .cmd_buf_clr_t2          (cmd_buf_clr_t2_wire),
    .cmd_buf_clr_ack_t0      (cmd_buf_clr_ack_t0_wire),
    .cmd_buf_clr_ack_t1      (cmd_buf_clr_ack_t1_wire),
    .cmd_buf_clr_ack_t2      (cmd_buf_clr_ack_t2_wire)
);
    
logic_container main_logic (
    // .VDD_1V65               (ua[0]),
    .clk                    (clk),
    .rst_n                  (rst_n),
    .ui_in                  (ui_in),
    .uo_out                 (uo_def),
    .uio_in                 (uio_in),
    .uio_out                (uio_out_def),
    .uio_oe                 (uio_oe_def),

    // Overlord Control & Debug
    .ovl_thread_sel         (thread_sel_wire),
    .ovl_reg_read_stage     (reg_read_stage_wire),
    .ovl_reg_address        (reg_addr_wire),
    .ovl_is_read            (is_read_wire),
    .ovl_reg_handover_done  (reg_handover_done_wire),
    .ovl_dbg_dout           (ovl_dbg_dout_wire),
    .ovl_dbg_din            (uio_in),
    .ovl_microcode_fence    ({o_microcode_fence_t2_wire, o_microcode_fence_t1_wire, o_microcode_fence_t0_wire}),
    .ovl_direct_instr       ({o_direct_instr_t2_wire,    o_direct_instr_t1_wire,    o_direct_instr_t0_wire}),
    .ovl_single_execute     ({single_execute_t2_wire,    single_execute_t1_wire,    single_execute_t0_wire}),
    .ovl_single_execute_ack ({single_execute_ack_t2_wire,single_execute_ack_t1_wire,single_execute_ack_t0_wire}),
    .ovl_cmd_buf_clr        ({cmd_buf_clr_t2_wire,       cmd_buf_clr_t1_wire,       cmd_buf_clr_t0_wire}),
    .ovl_cmd_buf_clr_ack    ({cmd_buf_clr_ack_t2_wire,   cmd_buf_clr_ack_t1_wire,   cmd_buf_clr_ack_t0_wire}),

    // Errors
    .ovl_pc_overflow        (pc_overflow_err_wire),
    .ovl_a_ctr_overflow     (micro_op_action_ovf_err_wire),
    .ovl_p_ctr_overflow     (micro_op_param_ovf_err_wire),
    .ovl_reg_overflow       (reg_overflow_err_wire),
    .ovl_offending_id_t0    (),
    .ovl_offending_id_t1    (),
    .ovl_offending_id_t2    ()
);

//multiplexer
    
    reg [7:0] uo_out_reg;
    reg [7:0] uio_out_reg;
    reg [7:0] uio_oe_reg;

    wire ovl_drive_bus = commander_tracker_enable || (is_read_wire && (reg_addr_wire != 5'd0));

    always @(*) begin
        case (mux_sel)
            3'b000: begin // Debug / Logic Tester
                uo_out_reg  = uo_000;
                uio_out_reg = uio_out_000;
                uio_oe_reg  = uio_oe_000;
            end
            3'b001: begin // Core Overlord
                uo_out_reg  = uo_001;
                uio_out_reg = commander_tracker_enable ? commander_tracker_data : ovl_dbg_dout_wire;
                uio_oe_reg  = ovl_drive_bus ? 8'hFF : 8'h00;
            end
            3'b111: begin // Standalone Mode
                uo_out_reg  = uo_111;
                uio_out_reg = uio_out_111;
                uio_oe_reg  = uio_oe_111;
            end
            default: begin // 010, 011, 100, 101, 110 -> Logic Container
                uo_out_reg  = uo_def;
                uio_out_reg = uio_out_def;
                uio_oe_reg  = uio_oe_def;
            end
        endcase
    end

    assign uo_out  = uo_out_reg;
    assign uio_out = uio_out_reg;
    assign uio_oe  = uio_oe_reg;

    // Tie off unused signals to prevent synthesis warnings
    wire _unused = &{ena, 1'b0};
    assign uo_000 = 8'h00; assign uio_out_000 = 8'h00; assign uio_oe_000 = 8'h00;
endmodule