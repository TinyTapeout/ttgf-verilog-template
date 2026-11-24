//the logic container is in charge of housing the 3 threads and 1 core plus controller
//the overlord is a separate spectator
/*
for the logic container it is in charge of handling command streaming into the things, 
can also do memory read and writes through the io for all 3 threads, as well as handling the changes between microcode, thread command intake and stuff 
*/
`default_nettype none
//also logic testing etc is done somewhere else, preferably somewhere that i can't see
module logic_container(
    
    // inout VDD_1V65,
    input clk,
    input rst_n,

    input  wire [7:0] ui_in,    // Dedicated inputs (Pins 0-7)
    output reg [7:0] uo_out,   // Dedicated outputs (Pins 16-23)
    input  wire [7:0] uio_in,   // IOs: Input path (Pins 8-15)
    output reg [7:0] uio_out,  // IOs: Output path (Pins 8-15)
    output reg [7:0] uio_oe,   // IOs: Enable path (active high: 1=drive, 0=high-Z)

    //add inputs from core overcommand
    //sounds like some evil shi from wolfenstein
    // Core Overlord / Debug Interface (Shared controls + thread select)
    input  wire [1:0]  ovl_reg_read_stage,  //byte chunk (0->1->2)
    input  wire [4:0]  ovl_reg_address,     //5-bit address (31)
    input  wire        ovl_is_read,         //read write toggle
    output wire  ovl_reg_handover_done,
    output wire [7:0]  ovl_dbg_dout,
    input  wire [7:0]  ovl_dbg_din,
    input  wire ovl_microcode_fence, //threadly basis
    input  wire ovl_direct_instr,
    input  wire ovl_single_execute,
    output wire ovl_single_execute_ack,
    input  wire ovl_cmd_buf_clr,
    output wire ovl_cmd_buf_clr_ack,

    //complaints hotline
    output wire ovl_pc_overflow,
    output wire ovl_a_ctr_overflow,
    output wire ovl_p_ctr_overflow,
    output wire ovl_reg_overflow,
    output wire [3:0]  ovl_offending_id_t0
);

/*
//interconnects!
    //thread 0
        wire [2:0]  cmd_t0_wire, tritwise_cmd_t0_wire, shift_wise_cmd_t0_wire;
        wire        cmd_ready_t0_wire, res_ready_t0_wire, prio_inc_t0_wire;
        wire [7:0]  t4_a_t0_wire, t4_b_t0_wire, t4_c_t0_wire, t4_o_t0_wire;
        wire [1:0]  t1_a_t0_wire, t1_b_t0_wire, t1_o_t0_wire;
        wire [15:0] mul_o_t0_wire;
        wire [3:0]  hog_t0_wire;
    //thread 1
        wire [2:0]  cmd_t1_wire, tritwise_cmd_t1_wire, shift_wise_cmd_t1_wire;
        wire        cmd_ready_t1_wire, res_ready_t1_wire, prio_inc_t1_wire;
        wire [7:0]  t4_a_t1_wire, t4_b_t1_wire, t4_c_t1_wire, t4_o_t1_wire;
        wire [1:0]  t1_a_t1_wire, t1_b_t1_wire, t1_o_t1_wire;
        wire [15:0] mul_o_t1_wire;
        wire [3:0]  hog_t1_wire;
    //thread 2
        wire [2:0]  cmd_t2_wire, tritwise_cmd_t2_wire, shift_wise_cmd_t2_wire;
        wire        cmd_ready_t2_wire, res_ready_t2_wire, prio_inc_t2_wire;
        wire [7:0]  t4_a_t2_wire, t4_b_t2_wire, t4_c_t2_wire, t4_o_t2_wire;
        wire [1:0]  t1_a_t2_wire, t1_b_t2_wire, t1_o_t2_wire;
        wire [15:0] mul_o_t2_wire;
        wire [3:0]  hog_t2_wire;

    //its called circuitry baby, look it up
        wire [7:0]  macro_op_t0, macro_op_t1, macro_op_t2;
        wire [7:0]  action_ctr_t0, action_ctr_t1, action_ctr_t2;
        wire [7:0]  param_ctr_t0,  param_ctr_t1,  param_ctr_t2;
        wire [6:0]  conv_act_t0,   conv_act_t1,   conv_act_t2;
        wire [6:0]  conv_prm_t0,   conv_prm_t1,   conv_prm_t2;

        wire [3:0]  micro_cmd_t0, micro_p0_t0, micro_p1_t0, micro_p2_t0;
        wire [3:0]  micro_cmd_t1, micro_p0_t1, micro_p1_t1, micro_p2_t1;
        wire [3:0]  micro_cmd_t2, micro_p0_t2, micro_p1_t2, micro_p2_t2;
        wire        micro_req_t0, micro_sent_t0, micro_flush_t0;
        wire        micro_req_t1, micro_sent_t1, micro_flush_t1;
        wire        micro_req_t2, micro_sent_t2, micro_flush_t2;

        wire [11:0] microcode_address;
        wire [7:0]  dbus = {uio_in[2:0], ui_in[7:3]};
        wire        cycle_threads;
        wire        microcode_idle;

    //debug and snoop
        wire [2:0]  req_show_reg, req_set_pc, req_mem_access, is_write_mem;
        wire [2:0]  pipe_empty, pipe_full_rej, req_new_cmd;
        wire [23:0] pc_out_t0,   pc_out_t1,   pc_out_t2;
        wire [23:0] addr_out_t0, addr_out_t1, addr_out_t2;
        wire [15:0] ext_addr_t0, ext_addr_t1, ext_addr_t2; // 8-trit (16-bit) high memory address for up to 9.7 gb of ram
        wire [23:0] d_out_t0,    d_out_t1,    d_out_t2;
    //data bus
        reg  [7:0]  bus_data_out;      // Sanitized 8-bit stream for macro-op / params
        reg  [2:0]  bus_param_idx;     // Forwarded ui_in[7:5] (param reg index)
        reg  [1:0]  bus_byte_phase;    // Byte chunk (00, 01, 10) of incoming 24-bit param
        reg         bus_is_cmd;        // 1 = 8-bit macro-op, 0 = 24-bit param byte
        reg  [2:0]  thread_data_valid; // 1-cycle strobe per thread when byte latched
        reg  [2:0]  thread_mem_done;   // 1-cycle strobe per thread when PC/mem/reg op finishes
        reg  [23:0] mem_d_in;          // Assembled 24-bit read data from FPGA

        wire [2:0]  dbg_handover_done;
        wire [7:0]  dbg_dout_t0, dbg_dout_t1, dbg_dout_t2;

        //debug readback to Overlord
        assign ovl_reg_handover_done = dbg_handover_done;
        assign ovl_dbg_dout          = (ovl_thread_sel == 2'd0) ? dbg_dout_t0 :
                                    (ovl_thread_sel == 2'd1) ? dbg_dout_t1 :
                                                                dbg_dout_t2;
*/

    wire [2:0]  cmd_w;
    wire [2:0]  tw_cmd_w;
    wire [2:0]  shift_cmd_w;
    wire cmd_ready_w;
    wire res_ready_w;

    wire [7:0]  t4_a_w;
    wire [7:0]  t4_b_w;
    wire [7:0]  t4_c_w;
    wire [7:0]  t4_o_w;
    wire [1:0]  t1_a_w;
    wire [1:0]  t1_b_w;
    wire [1:0]  t1_o_w;
    wire [15:0] mul_o_w;

    wire [11:0] microcode_address;
    wire [7:0]  dbus = {uio_in[2:0], ui_in[7:3]};
    wire        cycle_threads;
    wire        microcode_idle;

    // Per-thread Bus / Debug arrays
    wire req_show_reg, req_set_pc, req_mem_access, is_write_mem;
    wire pipe_full_rej, req_new_cmd;
    wire [23:0] addr_out_w    ;
    wire [15:0] ext_addr_w    ;
    wire [23:0] d_out_w       ;
    wire dbg_handover_done;
    wire [7:0]  dbg_dout_w    ;
    wire [3:0]  offending_id_w;

    // Bus staging registers
    reg  [7:0]  bus_data_out;
    reg  [2:0]  bus_param_idx;
    reg  [1:0]  bus_byte_phase;
    reg         bus_is_cmd;
    reg  thread_data_valid;
    reg  thread_mem_done;
    reg  [23:0] mem_d_in;

    assign ovl_reg_handover_done = dbg_handover_done;
    assign ovl_dbg_dout          = dbg_dout_w;
    assign ovl_offending_id_t0   = offending_id_w;


//1 thread 1 core
native_wrapper_sim native_alu(
        // .VDD_1V65           (VDD_1V65),

        .active_cmd_t0      (cmd_w),

        // Thread 0
        .trit4_in_a_0       (t4_a_w),
        .trit4_in_b_0       (t4_b_w),
        .trit4_in_c_0       (t4_c_w),
        .tritwise_control_0 (tw_cmd_w),
        .shift_control_0    (shift_cmd_w),
        .trit4_in_cin_0     (t1_a_w),
        .trit1_in_b_0       (t1_b_w),
        .trit4_out_0        (t4_o_w),
        .trit4_out_cout_0   (t1_o_w),
        .multiplier_out_0   (mul_o_w)
    );

// microcode microcode_parser (
//         .clk                        (clk),
//         .rst_n                      (rst_n),
//         //from 1  to 31 , each micro op corresponds to 2 byte chunked reads
//         //from 32 to 112, each micro op corresponds to 4 byte chunked reads
//         //from 113 to 191, each micro op corresponds to 8 byte chunked reads
//         // uses an 1024 offset for parameters
//         // 32 x 2 + 80 x 4 + 80 x 8 = 1024
//         //anything beyond 191 is ILLEGAL and should never be seen
//         // Thread 1
//         .macro_op_t0                (macro_op_w),
//         .microcode_action_offset_t0 (action_ctr_w), // 8-bit raw ternary (converted inside microcode)
//         .microcode_param_offset_t0  (param_ctr_w),
//         .next_microop_t0            (micro_req_w),
//         .flush_t0                   (micro_flush_w),
//         //this forces the microcode handler to pause, flush cache and read from action position & param position, as well as macro op position
//         //chunked reads mean that you start from indexes 0 , 4 , 8 , 12, etc, but the offsets can allow it to read any position
//         //the params are chunked separately, on the same chip but with an offset
//         .data_ready_t0              (micro_sent_w),
//         .action_t0                  (micro_cmd_w),
//         .param0_t0                  (micro_p0_w),
//         .param1_t0                  (micro_p1_w),
//         .param2_t0                  (micro_p2_w),

//         .bus                        (dbus),
//         .address                    (microcode_address),
//         .cycle_threads              (cycle_threads),
//         .microcode_idle             (microcode_idle)
//     );

//back to 1 thread...


            thread_command u_thread (
                .clk                        (clk),
                .rst_n                      (rst_n),

                .ucode_bus                  (dbus),
                .ucode_address              (microcode_address),
                .cycle_threads              (cycle_threads),
                .microcode_idle             (microcode_idle),

                .alu_command                (cmd_w),
                .tritwise_command           (tw_cmd_w),
                .tritshift_tritwise_command (shift_cmd_w),
                .alu_t4_a                   (t4_a_w),
                .alu_t4_b                   (t4_b_w),
                .alu_t4_c                   (t4_c_w),
                .alu_t1_a                   (t1_a_w),
                .alu_t1_b                   (t1_b_w),
                .alu_t4_o                   (t4_o_w),
                .alu_t1_o                   (t1_o_w),
                .alu_multiply_o             (mul_o_w),
                //cut hog and prior ++

                .bus_data_in                (bus_data_out),
                .bus_is_cmd                 (bus_is_cmd),
                .bus_param_idx              (bus_param_idx),
                .bus_byte_phase             (bus_byte_phase),
                .data_valid                 (thread_data_valid),
                .data_done                  (thread_mem_done),
                .request_show_reg           (req_show_reg),
                .request_set_pc             (req_set_pc),
                .request_memory_access      (req_mem_access),
                .is_write_memory            (is_write_mem),
                .addr_out                   (addr_out_w),
                .ext_addr_out               (ext_addr_w),
                .d_in                       (mem_d_in),
                .d_out                      (d_out_w),
                .pipe_full_rej              (pipe_full_rej),

                .register_read_stage        (ovl_reg_read_stage),
                .register_address           (ovl_reg_address),
                .is_read                    (ovl_is_read),
                .reg_handover_done          (dbg_handover_done),
                .dbg_dout                   (dbg_dout_w),
                .dbg_din                    (ovl_dbg_din),
                .microcode_fence            (ovl_microcode_fence),
                .direct_instr               (ovl_direct_instr),
                .single_execute             (ovl_single_execute),
                .single_execute_ack         (ovl_single_execute_ack),
                .cmd_buf_clr                (ovl_cmd_buf_clr),
                .cmd_buf_clr_ack            (ovl_cmd_buf_clr_ack),

                .pc_overflow                (ovl_pc_overflow),
                .a_ctr_overflow             (ovl_a_ctr_overflow),
                .p_ctr_overflow             (ovl_p_ctr_overflow),
                .reg_overflow               (ovl_reg_overflow),
                .offending_id               (offending_id_w)
            );

//reinforcement of canonical zero as 2'b10
function [7:0] sanitize_trits_8b;
    input [7:0] raw;
    begin
        sanitize_trits_8b[1:0] = (raw[1:0] == 2'b11) ? 2'b10 : raw[1:0];
        sanitize_trits_8b[3:2] = (raw[3:2] == 2'b11) ? 2'b10 : raw[3:2];
        sanitize_trits_8b[5:4] = (raw[5:4] == 2'b11) ? 2'b10 : raw[5:4];
        sanitize_trits_8b[7:6] = (raw[7:6] == 2'b11) ? 2'b10 : raw[7:6];
    end
endfunction


//updated muxing
    wire [2:0] mode          = ui_in[2:0];
    wire       is_thread     = (mode == 3'b010);
    wire       is_ucode      = (mode == 3'b101);

    wire       fpga_hs       = ui_in[3];   //handshake
    wire       fpga_is_cmd   = ui_in[4];   // 1 = macro-op, 0 = param (FPGA->ASIC)
    wire [2:0] fpga_phase_3b = ui_in[7:5]; //param addr (FPGA->ASIC) / byte idx (ASIC->FPGA)

    //cut pc
    wire [23:0] cur_addr      = addr_out_w;
    wire [15:0] cur_ext       = ext_addr_w;
    wire [23:0] cur_dout      = d_out_w;
    wire        cur_full      = pipe_full_rej;



    // Request Type Priority Encoder (000..110)
    wire [2:0] req_type =
        !is_thread             ? 3'b000 :
        !microcode_idle        ? 3'b010 : // request switch to microcode
        req_set_pc      ? 3'b110 : // update program counter
        req_mem_access  ? (is_write_mem ? 3'b101 : 3'b100) : // 101: Wr / 100: Rd
        req_show_reg    ? 3'b001 : // show register data
                                 3'b000;

    localparam [2:0]
        BUS_IDLE     = 3'd0, //data_phase = 2'b11 (Locked until fresh hs_rise)
        BUS_RX_WAIT  = 3'd1, //FPGA 2 ASIC: data_phase = 2'b00, wait for hs_fall
        BUS_RX_STRM  = 3'd2, //FPGA 2 ASIC: latch bytes on hs_toggle (00->01->10->11)
        BUS_ASIC_TX  = 3'd3, //ASIC 2 FPGA: ASIC drives uio_oe = 8'hFF
        BUS_MEM_TURN = 3'd4, //Mem Read Turnaround: 1-cycle High-Z dead time + handshake
        BUS_MEM_RD   = 3'd5; //Mem Read Data: FPGA drives uio_in, ASIC latches 3 bytes



    reg [2:0] bus_state;
    reg [2:0] active_req_type; // Latched so !microcode_idle cannot hijack mid-TX
    reg [1:0] data_phase;
    reg       hs_prev;
    reg       mem_data_phase; // 0 = 5-byte address, 1 = 3-byte write data (101)
    reg       turn_ready; // Guarantees >= 1 full High-Z clock before FPGA drives
    reg rx_is_direct;    // Per-thread flag: 1 when expecting 2 raw direct-exec bytes
    wire hs_rise   =  fpga_hs && !hs_prev;
    wire hs_fall   = !fpga_hs &&  hs_prev;
    wire hs_toggle =  fpga_hs ^   hs_prev;



    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bus_state         <= BUS_IDLE;
            active_req_type   <= 3'b000;
            data_phase        <= 2'b11;
            hs_prev           <= 1'b0;
            mem_data_phase    <= 1'b0;
            turn_ready        <= 1'b0;
            rx_is_direct      <= 1'b0;
            thread_data_valid <= 1'b0;
            thread_mem_done   <= 1'b0;
        end else begin
            hs_prev           <= fpga_hs;
            thread_data_valid <= 1'b0;
            thread_mem_done   <= 1'b0;
            if (!is_thread) begin
                bus_state       <= BUS_IDLE;
                active_req_type <= 3'b000;
                data_phase      <= 2'b11;
                mem_data_phase  <= 1'b0;
                turn_ready      <= 1'b0;
            end else begin
                case (bus_state)
                    BUS_IDLE: begin
                        data_phase     <= 2'b11;
                        mem_data_phase <= 1'b0;
                        turn_ready     <= 1'b0;

                        if (req_type != 3'b000) begin
                            if (hs_rise && (req_type == 3'b001 || req_type == 3'b100 ||
                                            req_type == 3'b101 || req_type == 3'b110)) begin
                                active_req_type <= req_type;
                                bus_state       <= BUS_ASIC_TX;
                            end
                        end else if (hs_rise && !cur_full) begin
                            data_phase <= 2'b00;
                            bus_state  <= BUS_RX_WAIT;
                        end
                    end

                    BUS_RX_WAIT: begin
                        if (hs_fall) bus_state <= BUS_RX_STRM;
                    end

                    BUS_RX_STRM: begin
                        if (hs_toggle) begin
                            bus_is_cmd               <= fpga_is_cmd;
                            bus_param_idx            <= fpga_phase_3b;
                            bus_byte_phase           <= data_phase;
                            // Skip ternary sanitization for macro-op bytes AND direct-exec param bytes
                            bus_data_out             <= (fpga_is_cmd || rx_is_direct) ? uio_in : sanitize_trits_8b(uio_in);
                            thread_data_valid <= 1'b1;

                            if (fpga_is_cmd) begin
                                rx_is_direct <= (uio_in[7:6] == 2'b11);
                                data_phase          <= 2'b11;
                                bus_state           <= BUS_IDLE;
                            end else if (rx_is_direct && (data_phase == 2'b01)) begin
                                rx_is_direct <= 1'b0; // 2-byte direct-exec payload complete
                                data_phase          <= 2'b11;
                                bus_state           <= BUS_IDLE;
                            end else if (data_phase == 2'b10) begin
                                data_phase          <= 2'b11;
                                bus_state           <= BUS_IDLE;
                            end else begin
                                data_phase          <= data_phase + 2'd1;
                            end
                        end
                    end

                    BUS_ASIC_TX: begin
                        data_phase <= 2'b11;
                        if (hs_toggle && (fpga_phase_3b == 3'b111)) begin
                            if (active_req_type == 3'b100) begin
                                turn_ready <= 1'b0;
                                bus_state  <= BUS_MEM_TURN;
                            end else if (active_req_type == 3'b101 && !mem_data_phase) begin
                                mem_data_phase <= 1'b1;
                            end else begin
                                thread_mem_done <= 1'b1;
                                mem_data_phase         <= 1'b0;
                                active_req_type        <= 3'b000;
                                bus_state              <= BUS_IDLE;
                            end
                        end
                    end

                    BUS_MEM_TURN: begin
                        if (!turn_ready) begin
                            turn_ready <= 1'b1;
                            data_phase <= 2'b00;
                        end else if (hs_toggle) begin
                            data_phase <= 2'b11;
                            bus_state  <= BUS_MEM_RD;
                        end
                    end

                    BUS_MEM_RD: begin
                        data_phase <= 2'b11;
                        if (hs_toggle) begin
                            case (fpga_phase_3b)
                                3'b000: mem_d_in[7:0]   <= sanitize_trits_8b(uio_in);
                                3'b001: mem_d_in[15:8]  <= sanitize_trits_8b(uio_in);
                                3'b010: mem_d_in[23:16] <= sanitize_trits_8b(uio_in);
                                3'b111: begin
                                    thread_mem_done <= 1'b1;
                                    active_req_type        <= 3'b000;
                                    bus_state              <= BUS_IDLE;
                                end
                                default: ;
                            endcase
                        end
                    end

                    default: bus_state <= BUS_IDLE;
                endcase
            end
        end
    end

    //TX Byte Selector (ASIC -> FPGA)
    reg [7:0] tx_byte;
    always @(*) begin
        tx_byte = 8'hAA;//canon 0
        case (active_req_type)
            3'b110, 3'b001: begin // can also be pc
                case (fpga_phase_3b)
                    3'b000:  tx_byte = cur_dout[7:0];
                    3'b001:  tx_byte = cur_dout[15:8];
                    3'b010:  tx_byte = cur_dout[23:16];
                    default: tx_byte = 8'hAA;
                endcase
            end
            3'b100, 3'b101: begin
                if (!mem_data_phase) begin
                    case (fpga_phase_3b)
                        3'b000:  tx_byte = cur_addr[7:0];
                        3'b001:  tx_byte = cur_addr[15:8];
                        3'b010:  tx_byte = cur_addr[23:16];
                        3'b011:  tx_byte = cur_ext[7:0];
                        3'b100:  tx_byte = cur_ext[15:8];
                        default: tx_byte = 8'hAA;
                    endcase
                end else begin
                    case (fpga_phase_3b)
                        3'b000:  tx_byte = cur_dout[7:0];
                        3'b001:  tx_byte = cur_dout[15:8];
                        3'b010:  tx_byte = cur_dout[23:16];
                        default: tx_byte = 8'hAA;
                    endcase
                end
            end
            default: tx_byte = 8'hAA;
        endcase
    end
    // External Pin Output Multiplexer
    always @(*) begin
        uo_out  = 8'h00;
        uio_out = 8'h00;
        uio_oe  = 8'h00; // Default High-Z

        if (is_ucode) begin
            //microcode!
            uio_out[7:3] = microcode_address[4:0];
            uio_oe[7:3]  = 5'b11111;
            uo_out[6:0]  = microcode_address[11:5];
            uo_out[7]    = cycle_threads;
        end else if (is_thread) begin
            //spi stuffs
            uo_out[0]   = 1'b0;       // SPI clk placeholder
            uo_out[7]   = 1'b0;       // SPI MISO placeholder
            uo_out[6:4] = (bus_state == BUS_ASIC_TX || bus_state == BUS_MEM_TURN || bus_state == BUS_MEM_RD)
                          ? active_req_type : req_type; // Request type [2:0]
            //data phase
            uo_out[2:1] = data_phase; // 00/01/10 active, 11 idle/locked
            uo_out[3]   = cur_full;   // Command register full kickback

            if (bus_state == BUS_ASIC_TX) begin
                uio_out = tx_byte;
                uio_oe  = 8'hFF;
            end
        end
    end
endmodule