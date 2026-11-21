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
    inout  wire [7:0] ua,
    input  wire       ena,      // Always 1 when powered
    input  wire       clk,      // Clock 
    input  wire       rst_n     // Reset (Active low)
);

    wire [2:0] mux_sel     = ui_in[7:5];
    wire [7:0] forwarded_ui = ui_in[7:0]; //fully forwarded

    // Output busses from the 4 distinct sub-modules
    wire [7:0] uo_000,  uo_001,  uo_111,  uo_def;
    wire [7:0] uio_out_000, uio_out_001, uio_out_111, uio_out_def;
    wire [7:0] uio_oe_000,  uio_oe_001,  uio_oe_111,  uio_oe_def;

    logic_tester testbench(

        VDD_1V65(ua[0]),
        .ui_in(ui_in),    // Dedicated inputs (Pins 0-7)
        .uo_out(uo_000),   // Dedicated outputs (Pins 16-23)
        .uio_in(uio_in),   // IOs: Input path (Pins 8-15)
        .uio_out(uio_out_000),  // IOs: Output path (Pins 8-15)
        .uio_oe(uio_oe_000),   // IOs: Enable path (active high: 1=drive, 0=high-Z)
        .clk(clk),
        .rst_n(rst_n),
    );

    //out from commander
        wire [7:0] commander_tracker_data;
        wire [7:0] commander_tracker_enable;

        wire [2:0] b2t_translation_err_wire;
        wire [2:0] microcode_illegal_err_wire;
        wire [2:0] pc_overflow_err_wire;
        wire [2:0] micro_op_action_ovf_err_wire;
        wire [2:0] micro_op_param_ovf_err_wire;
        wire [2:0] reg_overflow_err_wire;

        wire [1:0] thread_sel_wire;
        wire [1:0] reg_read_stage_wire;
        wire [4:0] reg_addr_wire;
        wire       is_read_wire;
        wire [2:0] reg_handover_done_wire; //remember on thread side when it stops reading this thread it resets

        wire o_microcode_fence_t0_wire;
        wire o_microcode_fence_t1_wire;
        wire o_microcode_fence_t2_wire;

        wire o_direct_instr_t0_wire;
        wire o_direct_instr_t1_wire;
        wire o_direct_instr_t2_wire;
        wire single_execute_t0_wire;
        wire single_execute_t1_wire;
        wire single_execute_t2_wire;
        wire single_execute_ack_t0_wire;
        wire single_execute_ack_t1_wire;
        wire single_execute_ack_t2_wire;

        wire cmd_buf_clr_t0_wire;
        wire cmd_buf_clr_t1_wire;
        wire cmd_buf_clr_t2_wire;

        wire cmd_buf_clr_ack_t0_wire;
        wire cmd_buf_clr_ack_t1_wire;
        wire cmd_buf_clr_ack_t2_wire;

    tt_um_core_commander core_commander (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ui_in                  (ui_in), 
        
        // Output routing
        .uo_out                 (uo_001),
        // Telemetry bus to external MUX (routes selected error counter to uio_out when show_tracker=1)
        .tracker                (commander_tracker_data),
        .show_tracker           (commander_tracker_enable),

        // -------------------------------------------------------------------------
        // Per-Thread Error Inputs (3 Threads x 6 Error Types)
        // Bit 0 = Thread 0, Bit 1 = Thread 1, Bit 2 = Thread 2
        // -------------------------------------------------------------------------
        .b2t_translation_err(b2t_translation_err_wire),         // Error ID 3'd0
        .microcode_illegal_err(microcode_illegal_err_wire),     // Error ID 3'd1
        .pc_overflow_err(pc_overflow_err_wire),                 // Error ID 3'd2
        .micro_op_action_ovf_err(micro_op_action_ovf_err_wire), // Error ID 3'd3 (Action overflow)
        .micro_op_param_ovf_err(micro_op_param_ovf_err_wire),   // Error ID 3'd4 (Param overflow)
        .reg_overflow_err(reg_overflow_err_wire),               // Error ID 3'd5

        // -------------------------------------------------------------------------
        // Register & Command Buffer Handover Interface
        // -------------------------------------------------------------------------
        .thread_select(thread_sel_wire),             // Active thread: 2'b00=T0, 2'b01=T1, 2'b10=T2
        .register_read_stage(reg_read_stage_wire),       // Byte chunk (0..2) or Cmd Buffer index (0=Active, 1=Next)
        .register_address(reg_addr_wire),          // Flat 5-bit address (0..31) incremented per handover
        .is_read(is_read_wire),                   // 1 = ASIC transmits to FPGA, 0 = ASIC receives from FPGA
        .reg_handover_done(reg_handover_done_wire),         // Pulled high by active thread when entire register dump finishes

        // -------------------------------------------------------------------------
        // Per-Thread Execution Control & Handshakes (Threads 0, 1, 2)
        // -------------------------------------------------------------------------
        .o_microcode_fence_t0(o_microcode_fence_t0_wire),
        .o_microcode_fence_t1(o_microcode_fence_t1_wire),
        .o_microcode_fence_t2(o_microcode_fence_t2_wire),

        .o_direct_instr_t0(o_direct_instr_t0_wire),
        .o_direct_instr_t1(o_direct_instr_t1_wire),
        .o_direct_instr_t2(o_direct_instr_t2_wire),

        .single_execute_t0(single_execute_t0_wire),
        .single_execute_t1(single_execute_t1_wire),
        .single_execute_t2(single_execute_t2_wire),
        .single_execute_ack_t0(single_execute_ack_t0_wire),
        .single_execute_ack_t1(single_execute_ack_t1_wire),
        .single_execute_ack_t2(single_execute_ack_t2_wire),

        .cmd_buf_clr_t0(cmd_buf_clr_t0_wire),
        .cmd_buf_clr_t1(cmd_buf_clr_t1_wire),
        .cmd_buf_clr_t2(cmd_buf_clr_t2_wire),
        .cmd_buf_clr_ack_t0(cmd_buf_clr_ack_t0_wire),
        .cmd_buf_clr_ack_t1(cmd_buf_clr_ack_t1_wire),
        .cmd_buf_clr_ack_t2(cmd_buf_clr_ack_t2_wire)
    );
    
    //multiplexer
    reg [7:0] uo_out_reg;
    reg [7:0] uio_out_reg;
    reg [7:0] uio_oe_reg;



wire ovl_selected = (ui_in[2:0] == 3'b001);

// uo_out: Overlord owns it on 3'b001, logic_container owns it otherwise
assign uo_out = ovl_selected ? ovl_uo_out : lc_uo_out;

// uio_out / uio_oe: Overlord drives tracker or debug readout when selected
wire ovl_drive_bus = ovl_selected && (ovl_show_tracker || (ovl_is_read && ovl_reg_address != 5'd0));
assign uio_out = ovl_selected ? (ovl_show_tracker ? ovl_tracker : ovl_dbg_dout) : lc_uio_out;
assign uio_oe  = ovl_selected ? (ovl_drive_bus ? 8'hFF : 8'h00)


    always @(*) begin
        case (mux_sel)
            3'b000: begin
                //debug
                uo_out_reg  = uo_000;
                uio_out_reg = uio_out_000;
                uio_oe_reg  = uio_oe_000;
            end
            3'b001: begin
                //core overlord
                uo_out_reg  = uo_001;
                uio_out_reg = commander_tracker_enable? commander_tracker_data : uio_out_def;
                uio_oe_reg  = commander_tracker_enable? 8'b11111111 : uio_oe_def;


                
            end
            3'b111: begin
                //le epic standalone mode
                // this one has internal sram!
                uio_out_reg = uio_out_111;
                uio_out_reg = uio_oe_111;
                uo_out_reg = uo_111;
            end
            default: begin // Covers 010, 011, 100, 101, 110
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
endmodule