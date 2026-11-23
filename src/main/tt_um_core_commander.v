
//original written by azzy
//updated by gemini
//fixed by azzy
`default_nettype none
module tt_um_core_commander (
    input  wire [7:0] ui_in,    // [2:0] enable tag (3'b001), [6:3] cmd_opcode, [7] FPGA read/write ack
    output wire [7:0] uo_out,   // Multiplexed status / address / error / handshake outputs
    input  wire       clk,
    input  wire       rst_n,

    
    // Telemetry bus to external MUX (routes selected error counter to uio_out when show_tracker=1)
    output reg  [7:0] tracker,
    output reg        show_tracker,

    // -------------------------------------------------------------------------
    // Per-Thread Error Inputs (3 Threads x 6 Error Types)
    // Bit 0 = Thread 0, Bit 1 = Thread 1, Bit 2 = Thread 2
    // -------------------------------------------------------------------------
    input  wire [2:0] pc_overflow_err,           // Error ID 3'd2
    input  wire [2:0] micro_op_action_ovf_err,   // Error ID 3'd3 (Action overflow)
    input  wire [2:0] micro_op_param_ovf_err,    // Error ID 3'd4 (Param overflow)
    input  wire [2:0] reg_overflow_err,          // Error ID 3'd5

    // -------------------------------------------------------------------------
    // Register & Command Buffer Handover Interface
    // -------------------------------------------------------------------------
    output reg  [1:0] thread_select,             // Active thread: 2'b00=T0, 2'b01=T1, 2'b10=T2
    output reg  [1:0] register_read_stage,       // Byte chunk (0..2) or Cmd Buffer index (0=Active, 1=Next)
    output reg  [4:0] register_address,          // Flat 5-bit address (0..31) incremented per handover
    output reg        is_read,                   // 1 = ASIC transmits to FPGA, 0 = ASIC receives from FPGA
    input  wire [2:0] reg_handover_done,         // Pulled high by active thread when entire register dump finishes

    // -------------------------------------------------------------------------
    // Per-Thread Execution Control & Handshakes (Threads 0, 1, 2)
    // -------------------------------------------------------------------------
    output wire       o_microcode_fence_t0,
    output wire       o_microcode_fence_t1,
    output wire       o_microcode_fence_t2,

    output wire       o_direct_instr_t0,
    output wire       o_direct_instr_t1,
    output wire       o_direct_instr_t2,

    output reg        single_execute_t0,
    output reg        single_execute_t1,
    output reg        single_execute_t2,
    input  wire       single_execute_ack_t0,
    input  wire       single_execute_ack_t1,
    input  wire       single_execute_ack_t2,

    output reg        cmd_buf_clr_t0,
    output reg        cmd_buf_clr_t1,
    output reg        cmd_buf_clr_t2,
    input  wire       cmd_buf_clr_ack_t0,
    input  wire       cmd_buf_clr_ack_t1,
    input  wire       cmd_buf_clr_ack_t2
);

    // -------------------------------------------------------------------------
    // Input Decoding
    // -------------------------------------------------------------------------
    wire       module_selected = (ui_in[2:0] == 3'b001);
    wire [3:0] cmd_opcode      = ui_in[6:3];
    wire       fpga_ack        = ui_in[7];

    // -------------------------------------------------------------------------
    // Controller FSM States (6 States total - removed soft_reset)
    // -------------------------------------------------------------------------
    localparam [2:0]
        state_idle    = 3'b000,
        state_rcv_pc  = 3'b001, // Receive 3-byte PC FROM FPGA (Reg Addr 5)
        state_trn_pc  = 3'b010, // Transmit 3-byte PC TO FPGA  (Reg Addr 5)
        state_rcv_reg = 3'b011, // Receive Registers FROM FPGA until reg_handover_done
        state_trn_reg = 3'b100, // Transmit Registers TO FPGA  until reg_handover_done
        state_cmd_clr = 3'b101, // Clear command buffer for selected thread
        state_cmd_trn = 3'b110; // Transmit 8-bit macro-op buffer (0=active, 1=next) TO FPGA

    reg [2:0] controller_state;

    // -------------------------------------------------------------------------
    // Per-Thread Governance Arrays (freeze_core removed -> uses microcode_fence)
    // -------------------------------------------------------------------------
    reg [2:0] microcode_fence;
    reg [2:0] direct_instruction;

    // When the bus is active (not idle), lock down ALL threads
    wire bus_active = (controller_state != state_idle);

    assign o_microcode_fence_t0 = microcode_fence[0] | bus_active;
    assign o_microcode_fence_t1 = microcode_fence[1] | bus_active;
    assign o_microcode_fence_t2 = microcode_fence[2] | bus_active;

    assign o_direct_instr_t0    = direct_instruction[0];
    assign o_direct_instr_t1    = direct_instruction[1];
    assign o_direct_instr_t2    = direct_instruction[2];

    // -------------------------------------------------------------------------
    // 6 Error Counter Arrays (3 Threads x 6 Error Types) & Latched Offender Info
    // -------------------------------------------------------------------------
    reg [7:0] err_pc_ovf_t      [0:2]; // ID 3'd2       //program counter overflow
    reg [7:0] err_uop_act_ovf_t [0:2]; // ID 3'd3       //micro pc action overflow
    reg [7:0] err_uop_prm_ovf_t [0:2]; // ID 3'd4       //micro pc param overflow
    reg [7:0] err_reg_ovf_t     [0:2]; // ID 3'd5       //register overflow

    //these are asynchronously assigned
    reg [2:0] prev_pc_ovf_err;
    reg [2:0] prev_uop_act_err;
    reg [2:0] prev_uop_prm_err;
    reg [2:0] prev_reg_ovf_err;

    reg       new_error_flag;     //this one has a reset command
    reg [1:0] offending_thread;   // Shown on uo_out[4:3] when new_error_flag == 1
    reg [2:0] offending_error_id; // Shown on uo_out[2:0] when new_error_flag == 1

    reg       showing_errors;
    reg [2:0] error_id_select;    // Incremented by 4'b1010 while showing_errors, reset when turned off

    // -------------------------------------------------------------------------
    // Tracker Multiplexer (Routes error counter of selected thread & error_id)
    // -------------------------------------------------------------------------
    wire [1:0] show_tracker_id = showing_errors ? thread_select : 2'b11;

    //handles showing priority
    always @(*) begin
        case (show_tracker_id)
            //thread 0
            2'b00: begin
                show_tracker = 1'b1;
                case (error_id_select)
                    3'd2:    tracker = err_pc_ovf_t[0];
                    3'd3:    tracker = err_uop_act_ovf_t[0];
                    3'd4:    tracker = err_uop_prm_ovf_t[0];
                    3'd5:    tracker = err_reg_ovf_t[0];
                    default: tracker = 8'h00;
                endcase
            end
            //thread 1
            2'b01: begin
                show_tracker = 1'b1;
                case (error_id_select)
                    3'd2:    tracker = err_pc_ovf_t[1];
                    3'd3:    tracker = err_uop_act_ovf_t[1];
                    3'd4:    tracker = err_uop_prm_ovf_t[1];
                    3'd5:    tracker = err_reg_ovf_t[1];
                    default: tracker = 8'h00;
                endcase
            end
            //thread 2
            2'b10: begin
                show_tracker = 1'b1;
                case (error_id_select)
                    3'd2:    tracker = err_pc_ovf_t[2];
                    3'd3:    tracker = err_uop_act_ovf_t[2];
                    3'd4:    tracker = err_uop_prm_ovf_t[2];
                    3'd5:    tracker = err_reg_ovf_t[2];
                    default: tracker = 8'h00;
                endcase
            end
            // zero threads
            default: begin // 2'b11: Off state
                show_tracker = 1'b0;
                tracker      = 8'h00;
            end
        endcase
    end

    // -------------------------------------------------------------------------
    // Output Pin Multiplexing (uo_out[7:0]) Strictly Following Header Spec
    // -------------------------------------------------------------------------
    reg command_op_ack;
    reg bus_ready;

    //these show what mode of bus access we are in
    wire is_reg_mode = (controller_state == state_rcv_reg) || (controller_state == state_trn_reg) ||
                       (controller_state == state_rcv_pc)  || (controller_state == state_trn_pc);
    // command mode means we are transmitting our commands out
    wire is_cmd_mode = (controller_state == state_cmd_trn);

    // Priority:
    // 1. Register / PC R/W Mode -> uo_out[4:0] = register_address[4:0]
    // 2. New Error Flag Active  -> uo_out[4:3] = offending_thread, uo_out[2:0] = offending_error_id
    // 3. Command Buffer Read    -> uo_out[4:3] = thread_select, uo_out[2:1] = state[1:0], uo_out[0] = active(0)/next(1)
    // 4. Default Idle/Status    -> uo_out[4:3] = thread_select, uo_out[2] = 0, uo_out[1:0] = controller_state[1:0]

    //these assign the uo to what i've specified
    assign uo_out[0] = is_reg_mode    ? register_address[0]   :
                       new_error_flag ? offending_error_id[0] :
                       is_cmd_mode    ? register_read_stage[0]:
                                        controller_state[0];

    assign uo_out[1] = is_reg_mode    ? register_address[1]   :
                       new_error_flag ? offending_error_id[1] :
                                        controller_state[1];

    assign uo_out[2] = is_reg_mode    ? register_address[2]   :
                       new_error_flag ? offending_error_id[2] :
                                        controller_state[2];    //added this bit

    assign uo_out[3] = is_reg_mode    ? register_address[3]   :
                       new_error_flag ? offending_thread[0]   :
                                        thread_select[0];

    assign uo_out[4] = is_reg_mode    ? register_address[4]   :
                       new_error_flag ? offending_thread[1]   :
                                        thread_select[1];

    assign uo_out[5] = command_op_ack;
    assign uo_out[6] = new_error_flag;
    assign uo_out[7] = bus_ready;

    // -------------------------------------------------------------------------
    // Main Sequential Logic
    // -------------------------------------------------------------------------
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            
            //replaced that with hard coded stuff cause i dont trust the verilog
                err_pc_ovf_t[0]      <= 8'b0;
                err_uop_act_ovf_t[0] <= 8'b0;
                err_uop_prm_ovf_t[0] <= 8'b0;
                err_reg_ovf_t[0]     <= 8'b0;
            //err thread 1
                err_pc_ovf_t[1]      <= 8'b0;
                err_uop_act_ovf_t[1] <= 8'b0;
                err_uop_prm_ovf_t[1] <= 8'b0;
                err_reg_ovf_t[1]     <= 8'b0;
            //err thread 2
                err_pc_ovf_t[2]      <= 8'b0;
                err_uop_act_ovf_t[2] <= 8'b0;
                err_uop_prm_ovf_t[2] <= 8'b0;
                err_reg_ovf_t[2]     <= 8'b0;
            prev_pc_ovf_err     <= 3'b0;
            prev_uop_act_err    <= 3'b0;
            prev_uop_prm_err    <= 3'b0;
            prev_reg_ovf_err    <= 3'b0;

            new_error_flag      <= 1'b0;
            offending_thread    <= 2'b00;
            offending_error_id  <= 3'd0;

            showing_errors      <= 1'b0;
            error_id_select     <= 3'd0;

            thread_select       <= 2'b00;
            controller_state    <= state_idle;
            register_read_stage <= 2'b00;
            register_address    <= 5'd0;
            is_read             <= 1'b0;
            command_op_ack      <= 1'b0;
            bus_ready           <= 1'b0;

            microcode_fence     <= 3'b000;
            direct_instruction  <= 3'b000;

            single_execute_t0   <= 1'b0;
            single_execute_t1   <= 1'b0;
            single_execute_t2   <= 1'b0;

            cmd_buf_clr_t0      <= 1'b0;
            cmd_buf_clr_t1      <= 1'b0;
            cmd_buf_clr_t2      <= 1'b0;
        end else begin
            // -----------------------------------------------------------------
            // 1. Rising-Edge Error Detection, Auto-Fence, and Offender Latching
            // -----------------------------------------------------------------
            prev_pc_ovf_err  <= pc_overflow_err;
            prev_uop_act_err <= micro_op_action_ovf_err;
            prev_uop_prm_err <= micro_op_param_ovf_err;
            prev_reg_ovf_err <= reg_overflow_err;

            //explicit collision handling
            
            begin : error_evaluation
                reg error_this_tick;
                error_this_tick = 1'b0; // Resets at the start of every clock edge
            
                for (i = 0; i < 3; i = i + 1) begin
                    // Check if ANY error triggered for this thread on this clock cycle
                    if ((pc_overflow_err[i] && !prev_pc_ovf_err[i]) ||
                        (micro_op_action_ovf_err[i] && !prev_uop_act_err[i]) ||
                        (micro_op_param_ovf_err[i] && !prev_uop_prm_err[i]) ||
                        (reg_overflow_err[i] && !prev_reg_ovf_err[i])) begin
                        
                        if (error_this_tick) begin
                            // COLLISION: Another thread already threw an error this exact cycle
                            offending_thread <= 2'b11;
                        end else begin
                            // First error detected this cycle, latch its details
                            offending_thread <= i[1:0];
                            if (pc_overflow_err[i] && !prev_pc_ovf_err[i])            offending_error_id <= 3'd2;
                            else if (micro_op_action_ovf_err[i] && !prev_uop_act_err[i])   offending_error_id <= 3'd3;
                            else if (micro_op_param_ovf_err[i] && !prev_uop_prm_err[i])    offending_error_id <= 3'd4;
                            else if (reg_overflow_err[i] && !prev_reg_ovf_err[i])          offending_error_id <= 3'd5;
                        end

                        error_this_tick    = 1'b1;
                        new_error_flag     <= 1'b1;
                        microcode_fence[i] <= 1'b1; // Auto-fence the offending thread
                    end
                    // Increment the individual counters regardless of collisions
                    if (pc_overflow_err[i] && !prev_pc_ovf_err[i])            err_pc_ovf_t[i]      <= err_pc_ovf_t[i] + 1'b1;
                    if (micro_op_action_ovf_err[i] && !prev_uop_act_err[i])   err_uop_act_ovf_t[i] <= err_uop_act_ovf_t[i] + 1'b1;
                    if (micro_op_param_ovf_err[i] && !prev_uop_prm_err[i])    err_uop_prm_ovf_t[i] <= err_uop_prm_ovf_t[i] + 1'b1;
                    if (reg_overflow_err[i] && !prev_reg_ovf_err[i])          err_reg_ovf_t[i]     <= err_reg_ovf_t[i] + 1'b1;
                end
            end
            // -----------------------------------------------------------------
            // 2. Clear Handshaked Sub-Module Pulses on ACK
            // -----------------------------------------------------------------
            if (single_execute_ack_t0) single_execute_t0 <= 1'b0;
            if (single_execute_ack_t1) single_execute_t1 <= 1'b0;
            if (single_execute_ack_t2) single_execute_t2 <= 1'b0;

            if (cmd_buf_clr_ack_t0) cmd_buf_clr_t0 <= 1'b0;
            if (cmd_buf_clr_ack_t1) cmd_buf_clr_t1 <= 1'b0;
            if (cmd_buf_clr_ack_t2) cmd_buf_clr_t2 <= 1'b0;

            // -----------------------------------------------------------------
            // 3. Host Command Interface (Active when ui_in[2:0] == 3'b001)
            // -----------------------------------------------------------------
            if (module_selected) begin
                if (cmd_opcode == 4'b0000) begin
                    command_op_ack <= 1'b0;

                    if (!showing_errors) begin
                        case (controller_state)
                            // -------------------------------------------------
                            // Transmit or Receive 24-bit (3-Byte) PC (Reg ID 5)
                            // -------------------------------------------------
                            state_trn_pc, state_rcv_pc: begin
                                if (fpga_ack) begin
                                    if (bus_ready) begin
                                        bus_ready <= 1'b0;
                                        if (register_read_stage == 2'b10) begin
                                            register_read_stage <= 2'b00;
                                            register_address    <= 5'd0;
                                            controller_state    <= state_idle;
                                        end else begin
                                            register_read_stage <= register_read_stage + 1'b1;
                                        end
                                    end
                                end else begin
                                    if (!bus_ready) bus_ready <= 1'b1;
                                end
                            end

                            // -------------------------------------------------
                            // Stream Registers (Increment 0..31 until reg_handover_done)
                            // -------------------------------------------------
                            state_trn_reg, state_rcv_reg: begin
                                if (reg_handover_done[thread_select]) begin
                                    // Active thread signaled all register bytes are handed over
                                    bus_ready           <= 1'b0;
                                    register_address    <= 5'd0;
                                    register_read_stage <= 2'b00;
                                    controller_state    <= state_idle;
                                end else if (fpga_ack) begin
                                    if (bus_ready) begin
                                        bus_ready <= 1'b0;
                                        if (register_read_stage == 2'b10) begin
                                            // 3 bytes finished, move to the next 24-bit Register ID
                                            register_read_stage <= 2'b00;
                                            register_address    <= register_address + 1'b1;
                                        end else begin
                                            // Next byte chunk of the current register
                                            register_read_stage <= register_read_stage + 1'b1;
                                        end
                                    end
                                end else begin
                                    if (!bus_ready) bus_ready <= 1'b1;
                                end
                            end
                            // -------------------------------------------------
                            // Transmit 8-bit Macro-Op Command Buffer (0=Active, 1=Next)
                            // -------------------------------------------------
                            state_cmd_trn: begin
                                if (fpga_ack) begin
                                    if (bus_ready) begin
                                        bus_ready <= 1'b0;
                                        if (register_read_stage == 2'b01) begin
                                            register_read_stage <= 2'b00;
                                            register_address    <= 5'd0; //release thread from debug mode
                                            controller_state    <= state_idle;
                                        end else begin
                                            register_read_stage <= 2'b01;
                                        end
                                    end
                                end else begin
                                    if (!bus_ready) bus_ready <= 1'b1;
                                end
                            end
                            // -------------------------------------------------
                            // Command Buffer Clear Handshake Wait
                            // -------------------------------------------------
                            state_cmd_clr: begin
                                if ((thread_select == 2'd0 && cmd_buf_clr_ack_t0) ||
                                    (thread_select == 2'd1 && cmd_buf_clr_ack_t1) ||
                                    (thread_select == 2'd2 && cmd_buf_clr_ack_t2)) begin
                                    controller_state <= state_idle;
                                end
                            end

                            default: controller_state <= state_idle;
                        endcase
                    end
                end else if (command_op_ack == 1'b0 && controller_state == state_idle) begin
                    // ---------------------------------------------------------
                    // Decode Host Command Opcodes (Only when Idle & Ack is Low)
                    // ---------------------------------------------------------
                    case (cmd_opcode)
                        // 0001: Toggle Direct Instruction Mode for current thread
                        4'b0001: begin
                            direct_instruction[thread_select] <= ~direct_instruction[thread_select];
                        end

                        // 0010: Transmit PC of current thread TO FPGA (Table ID 5 = Program Counter)
                        4'b0010: begin
                            controller_state    <= state_trn_pc;
                            register_read_stage <= 2'b00;
                            bus_ready           <= 1'b0;
                            register_address    <= 5'd5;
                            is_read             <= 1'b1;
                            showing_errors      <= 1'b0;
                            error_id_select     <= 3'd0;
                        end

                        // 0011: Increment Thread Select (Blocked if showing_errors or busy)
                        4'b0011: begin
                            if (!showing_errors) begin
                                thread_select <= (thread_select == 2'b10) ? 2'b00 : (thread_select + 1'b1);
                            end
                        end

                        // 0100: Micro-Op Fence / Single-Step Execute on current thread
                        4'b0100: begin
                            if (microcode_fence[thread_select]) begin
                                case (thread_select)
                                    2'd0: if (!single_execute_ack_t0) single_execute_t0 <= 1'b1;
                                    2'd1: if (!single_execute_ack_t1) single_execute_t1 <= 1'b1;
                                    2'd2: if (!single_execute_ack_t2) single_execute_t2 <= 1'b1;
                                    default: ;
                                endcase
                            end else begin
                                microcode_fence[thread_select] <= 1'b1;
                            end
                        end

                        // 0101: Set Program Counter FROM FPGA (Fences thread microcode)
                        4'b0101: begin
                            controller_state               <= state_rcv_pc;
                            register_read_stage            <= 2'b00;
                            bus_ready                      <= 1'b0;
                            register_address               <= 5'd5;
                            is_read                        <= 1'b0;
                            showing_errors                 <= 1'b0;
                            error_id_select                <= 3'd0;
                            microcode_fence[thread_select] <= 1'b1;
                        end

                        // 0110: Command Buffer Dump (0=Active 8-bit macro-op, 1=Next 8-bit macro-op)
                        4'b0110: begin
                            controller_state    <= state_cmd_trn;
                            register_read_stage <= 2'b00;
                            register_address    <= 5'd31; // Reserved ID for macro-op buffer dump
                            bus_ready           <= 1'b0;
                            is_read             <= 1'b1;
                            showing_errors      <= 1'b0;
                            error_id_select     <= 3'd0;
                        end

                        // 0111: Register Dump TO FPGA (Starts at address 1, increments until reg_handover_done)
                        4'b0111: begin
                            controller_state    <= state_trn_reg;
                            register_read_stage <= 2'b00;
                            bus_ready           <= 1'b0;
                            register_address    <= 5'd1;
                            is_read             <= 1'b1;
                            showing_errors      <= 1'b0;
                            error_id_select     <= 3'd0;
                        end

                        // 1000: Unfence Microcode (Resumes continuous execution on current thread)
                        4'b1000: begin
                            microcode_fence[thread_select] <= 1'b0;
                        end

                        // 1001: Clear Command Buffer on current thread
                        4'b1001: begin
                            controller_state <= state_cmd_clr;
                            case (thread_select)
                                2'd0: cmd_buf_clr_t0 <= 1'b1;
                                2'd1: cmd_buf_clr_t1 <= 1'b1;
                                2'd2: cmd_buf_clr_t2 <= 1'b1;
                                default: ;
                            endcase
                        end

                        // 1010: Increment Error ID (0..5) ONLY while showing_errors is active
                        4'b1010: begin
                            if (showing_errors) begin
                                error_id_select <= (error_id_select == 3'd5) ? 3'd0 : (error_id_select + 1'b1);
                            end
                        end

                        // 1011: Register Write FROM FPGA (Starts at address 1, increments until reg_handover_done)
                        4'b1011: begin
                            controller_state    <= state_rcv_reg;
                            register_read_stage <= 2'b00;
                            bus_ready           <= 1'b0;
                            register_address    <= 5'd1;
                            is_read             <= 1'b0;
                            showing_errors      <= 1'b0;
                            error_id_select     <= 3'd0;
                        end

                        // 1100: Acknowledge & Clear New Error Alert Flag (uo_out[6]) without losing counts
                        4'b1100: begin
                            new_error_flag <= 1'b0;
                        end

                        // 1101: Toggle Showing Errors ON/OFF (Flipping OFF resets error_id_select to 0)
                        4'b1101: begin
                            if (showing_errors) begin
                                showing_errors  <= 1'b0;
                                error_id_select <= 3'd0;
                            end else begin
                                showing_errors  <= 1'b1;
                                error_id_select <= 3'd0;
                                new_error_flag  <= 1'b0;
                            end
                        end

                        // 1110 (Free Slot): Clear All Error Counters & Alert Flag for Selected Thread
                        4'b1110: begin
                            err_pc_ovf_t[thread_select]      <= 8'b0;
                            err_uop_act_ovf_t[thread_select] <= 8'b0;
                            err_uop_prm_ovf_t[thread_select] <= 8'b0;
                            err_reg_ovf_t[thread_select]     <= 8'b0;
                            new_error_flag                   <= 1'b0;
                        end

                        default: ;
                    endcase
                    command_op_ack <= 1'b1;
                end
            end
        end
    end

endmodule