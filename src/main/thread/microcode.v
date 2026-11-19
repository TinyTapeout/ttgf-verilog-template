module microcode #(
    parameter [3:0] PAD_NIBBLE = 4'hF //nibble value representing 'x' (padding)
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  macro_op_t0,
    input  wire [6:0]  microcode_action_offset_t0,
    input  wire [6:0]  microcode_param_offset_t0,
    input  wire        next_microop_t0,
    input  wire        flush_t0,
    output reg         data_ready_t0,
    output reg  [3:0]  action_t0,
    output reg  [3:0]  param0_t0,
    output reg  [3:0]  param1_t0,
    output reg  [3:0]  param2_t0,
    output reg  [6:0]  microcode_action_offset_t0_res,
    output reg  [6:0]  microcode_param_offset_t0_res,


    input  wire [7:0]  macro_op_t1,
    input  wire [6:0]  microcode_action_offset_t1,
    input  wire [6:0]  microcode_param_offset_t1,
    input  wire        next_microop_t1,
    input  wire        flush_t1,
    output reg         data_ready_t1,
    output reg  [3:0]  action_t1,
    output reg  [3:0]  param0_t1,
    output reg  [3:0]  param1_t1,
    output reg  [3:0]  param2_t1,
    output reg  [6:0]  microcode_action_offset_t1_res,
    output reg  [6:0]  microcode_param_offset_t1_res,


    input  wire [7:0]  macro_op_t2,
    input  wire [6:0]  microcode_action_offset_t2,
    input  wire [6:0]  microcode_param_offset_t2,
    input  wire        next_microop_t2,
    input  wire        flush_t2,
    output reg         data_ready_t2,
    output reg  [3:0]  action_t2,
    output reg  [3:0]  param0_t2,
    output reg  [3:0]  param1_t2,
    output reg  [3:0]  param2_t2,
    output reg  [6:0]  microcode_action_offset_t2_res,
    output reg  [6:0]  microcode_param_offset_t2_res,


    input  wire [7:0]  bus, //la controller uses this
    output reg  [11:0] address,
    output wire         cycle_threads,
    output wire        microcode_idle
);

    // 1. Sector Base Address Calculation
    //    0  to 31  (32 ops): 2-byte chunks -> Base = op * 2
    //    32 to 111 (80 ops): 4-byte chunks -> Base = 64  + (op - 32) * 4
    //    112 to 191(80 ops): 8-byte chunks -> Base = 384 + (op - 112) * 8
    function [9:0] calc_sector_base(input [7:0] macro_op);
        reg [7:0] rel_op;
        begin
            if (macro_op <= 8'd31) begin
                calc_sector_base = {4'b0000, macro_op[4:0], 1'b0};
            end else if (macro_op <= 8'd111) begin
                rel_op = macro_op - 8'd32;
                calc_sector_base = 10'd64 + ({2'b00, rel_op} << 2);
            end else if (macro_op <= 8'd191) begin
                rel_op = macro_op - 8'd112;
                calc_sector_base = 10'd384 + ({2'b00, rel_op} << 3);
            end else begin
                calc_sector_base = 10'd0; // Illegal fallback
            end
        end
    endfunction

    // // Helper: returns parameter counter increment based on command code
        // //   command == 4'b0000 -> +1
        // //   command == 4'b0001 -> +2
        // function [6:0] get_param_step(input [3:0] cmd, input [1:0] fallback_step);
        //     begin
        //         if (cmd == 4'b0000)
        //             get_param_step = 7'd1;
        //         else if (cmd == 4'b0001)
        //             get_param_step = 7'd2;
        //         else
        //             get_param_step = {5'd0, fallback_step};
        //     end
        // endfunction


    // 2. Per-Thread Request Tracking, Importance Counters & Cached Pair State
    reg        request_data   [0:2];
    reg [5:0]  importance     [0:2]; //max 64
    reg        has_cached_odd [0:2];
    reg [3:0]  cached_act_odd [0:2];
    reg [3:0]  cached_p0_odd  [0:2];
    reg [3:0]  cached_p1_odd  [0:2];
    reg [3:0]  cached_p2_odd  [0:2];
    wire [2:0] flush_vec = {flush_t2, flush_t1, flush_t0};


    //Arbiter from Halo 2
    reg [1:0] winning_thread;
    reg       any_request;

    always @(*) begin
        any_request    = request_data[0] | request_data[1] | request_data[2];
        winning_thread = 2'd0;

        if (request_data[0] &&
           (!request_data[1] || importance[0] >= importance[1]) &&
           (!request_data[2] || importance[0] >= importance[2])) begin
            winning_thread = 2'd0;
        end else if (request_data[1] &&
                    (!request_data[2] || importance[1] >= importance[2])) begin
            winning_thread = 2'd1;
        end else if (request_data[2]) begin
            winning_thread = 2'd2;
        end
    end

    // -------------------------------------------------------------------------
    // 4. Memory Interfacer FSM States & Working Registers
    // -------------------------------------------------------------------------
    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_WAIT_ACT  = 3'd1,
        ST_WAIT_PRM0 = 3'd2,
        ST_WAIT_PRM1 = 3'd3,
        ST_WAIT_PRM2 = 3'd4,
        ST_COMMIT    = 3'd5;

    reg [2:0]  state;
    reg [1:0]  active_t; //active thread
    reg [9:0]  active_base; // active read address
    reg [6:0]  cur_act_off; //current action offset
    reg [6:0]  cur_prm_off; //current param offset

    reg [7:0]  latched_act_byte; //latched data
    reg [7:0]  latched_prm0;
    reg [7:0]  latched_prm1;
    reg [7:0]  latched_prm2;
    reg [1:0]  extra_reads_needed;

    assign microcode_idle = !(request_data[0] | request_data[1] | request_data[2]) && (state == ST_IDLE);

    wire [7:0] sel_macro_op = (winning_thread == 2'd0) ? macro_op_t0 :
                              (winning_thread == 2'd1) ? macro_op_t1 : macro_op_t2;
    wire [6:0] sel_act_off  = (winning_thread == 2'd0) ? microcode_action_offset_t0_res :
                              (winning_thread == 2'd1) ? microcode_action_offset_t1_res :
                                                         microcode_action_offset_t2_res;
    wire [6:0] sel_prm_off  = (winning_thread == 2'd0) ? microcode_param_offset_t0_res :
                              (winning_thread == 2'd1) ? microcode_param_offset_t1_res :
                                                         microcode_param_offset_t2_res;


    wire [6:0] next_act_off = flush_vec[winning_thread] ?
                              ((winning_thread == 2'd0) ? microcode_action_offset_t0 :
                               (winning_thread == 2'd1) ? microcode_action_offset_t1 :
                                                          microcode_action_offset_t2) : sel_act_off;
    wire [6:0] next_prm_off = flush_vec[winning_thread] ?
                              ((winning_thread == 2'd0) ? microcode_param_offset_t0 :
                               (winning_thread == 2'd1) ? microcode_param_offset_t1 :
                                                          microcode_param_offset_t2) : sel_prm_off;
    // Combinational unpacking wires used in ST_COMMIT
    wire [3:0] op0_cmd = latched_act_byte[7:4];
    wire [3:0] op1_cmd = latched_act_byte[3:0];

    wire [3:0] raw_op0_p0 = latched_prm0[7:4];
    wire [3:0] raw_op0_p1 = (latched_prm0[3:0] == PAD_NIBBLE) ? 4'd0 : latched_prm0[3:0];
    wire [3:0] raw_op0_p2 = (latched_prm0[3:0] != PAD_NIBBLE &&
                             latched_prm1[7:4] != PAD_NIBBLE &&
                             latched_prm1[3:0] != PAD_NIBBLE) ? latched_prm1[7:4] : 4'd0;

    // Apply command 0000 / 0001 output masking rules for Op0:
    //   - 0000: only param0 is sent (param1 = 0, param2 = 0)
    //   - 0001: only param0 and param1 are sent (param2 = 0)
    wire [3:0] final_op0_p0 = raw_op0_p0;
    wire [3:0] final_op0_p1 = (op0_cmd == 4'b0000) ? 4'd0 : raw_op0_p1;
    wire [3:0] final_op0_p2 = (op0_cmd == 4'b0000 || op0_cmd == 4'b0001) ? 4'd0 : raw_op0_p2;

    integer i;
    
    //high whenever the FSM is idle and no thread is requesting an external fetch
    //(also stays high if a thread is only hitting straight cache)
    wire bus_needed = any_request && !(has_cached_odd[winning_thread] && !flush_vec[winning_thread]);
    assign cycle_threads = (state == ST_IDLE) && !bus_needed;
    //logik
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                          <= ST_IDLE;
            active_t                       <= 2'd0;
            active_base                    <= 10'd0;
            cur_act_off                    <= 7'd0;
            cur_prm_off                    <= 7'd0;
            address                        <= 12'd0;
            latched_act_byte               <= 8'd0;
            latched_prm0                   <= 8'd0;
            latched_prm1                   <= 8'd0;
            latched_prm2                   <= 8'd0;
            extra_reads_needed             <= 2'd0;

            data_ready_t0                  <= 1'b0;
            data_ready_t1                  <= 1'b0;
            data_ready_t2                  <= 1'b0;
            action_t0  <= 4'd0; param0_t0 <= 4'd0; param1_t0 <= 4'd0; param2_t0 <= 4'd0;
            action_t1  <= 4'd0; param0_t1 <= 4'd0; param1_t1 <= 4'd0; param2_t1 <= 4'd0;
            action_t2  <= 4'd0; param0_t2 <= 4'd0; param1_t2 <= 4'd0; param2_t2 <= 4'd0;

            microcode_action_offset_t0_res <= 7'd0;
            microcode_param_offset_t0_res  <= 7'd0;
            microcode_action_offset_t1_res <= 7'd0;
            microcode_param_offset_t1_res  <= 7'd0;
            microcode_action_offset_t2_res <= 7'd0;
            microcode_param_offset_t2_res  <= 7'd0;



            //synths into physical stuff
            for (i = 0; i < 3; i = i + 1) begin
                request_data[i]   <= 1'b0;
                importance[i]     <= 6'd0;
                has_cached_odd[i] <= 1'b0;
                cached_act_odd[i] <= 4'd0;
                cached_p0_odd[i]  <= 4'd0;
                cached_p1_odd[i]  <= 4'd0;
                cached_p2_odd[i]  <= 4'd0;
            end
        end else begin
            cycle_threads <= 1'b0;//goes up when no work is needed
            //removed toggle based request for traditional pulse based request
            if (flush_t0) begin
                microcode_action_offset_t0_res <= microcode_action_offset_t0;
                microcode_param_offset_t0_res  <= microcode_param_offset_t0;
                has_cached_odd[0]              <= 1'b0;
                data_ready_t0                  <= 1'b0;
                request_data[0]                <= 1'b1;
                importance[0]                  <= 6'd1;
            end else if (next_microop_t0) begin
                data_ready_t0                  <= 1'b0;
                request_data[0]                <= 1'b1;
            end

            if (flush_t1) begin
                microcode_action_offset_t1_res <= microcode_action_offset_t1;
                microcode_param_offset_t1_res  <= microcode_param_offset_t1;
                has_cached_odd[1]              <= 1'b0;
                data_ready_t1                  <= 1'b0;
                request_data[1]                <= 1'b1;
                importance[1]                  <= 6'd1;
            end else if (next_microop_t1) begin
                data_ready_t1                  <= 1'b0;
                request_data[1]                <= 1'b1;
            end

            if (flush_t2) begin
                microcode_action_offset_t2_res <= microcode_action_offset_t2;
                microcode_param_offset_t2_res  <= microcode_param_offset_t2;
                has_cached_odd[2]              <= 1'b0;
                data_ready_t2                  <= 1'b0;
                request_data[2]                <= 1'b1;
                importance[2]                  <= 6'd1;
            end else if (next_microop_t2) begin
                data_ready_t2                  <= 1'b0;
                request_data[2]                <= 1'b1;
            end

            // Increment importance for every waiting thread (saturating at 63)
            for (i = 0; i < 3; i = i + 1) begin
                if (request_data[i] && !(state == ST_COMMIT && active_t == i[1:0])) begin
                    if (importance[i] != 6'h3F)
                        importance[i] <= importance[i] + 6'd1;
                end
            end

            // -----------------------------------------------------------------
            // Memory Interfacer & Parameter Alignment State Machine
            // -----------------------------------------------------------------
            case (state)
                ST_IDLE: begin
                    if (any_request) begin
                        active_t <= winning_thread;

                        // Fast-path: serve cached 2nd micro-op (Op1) in 1 cycle
                        if (has_cached_odd[winning_thread] && !flush_vec[winning_thread]) begin
                            has_cached_odd[winning_thread] <= 1'b0;
                            request_data[winning_thread]   <= 1'b0;
                            importance[winning_thread]     <= 6'd0;
                            cycle_threads                  <= 1'b1;

                            case (winning_thread)
                                2'd0: begin
                                    action_t0     <= cached_act_odd[0];
                                    param0_t0     <= cached_p0_odd[0];
                                    param1_t0     <= (cached_act_odd[0] == 4'b0000) ? 4'd0 : cached_p1_odd[0];
                                    param2_t0     <= (cached_act_odd[0] == 4'b0000 || cached_act_odd[0] == 4'b0001) ? 4'd0 : cached_p2_odd[0];
                                    data_ready_t0 <= 1'b1;
                                end
                                2'd1: begin
                                    action_t1     <= cached_act_odd[1];
                                    param0_t1     <= cached_p0_odd[1];
                                    param1_t1     <= (cached_act_odd[1] == 4'b0000) ? 4'd0 : cached_p1_odd[1];
                                    param2_t1     <= (cached_act_odd[1] == 4'b0000 || cached_act_odd[1] == 4'b0001) ? 4'd0 : cached_p2_odd[1];
                                    data_ready_t1 <= 1'b1;
                                end
                                2'd2: begin
                                    action_t2     <= cached_act_odd[2];
                                    param0_t2     <= cached_p0_odd[2];
                                    param1_t2     <= (cached_act_odd[2] == 4'b0000) ? 4'd0 : cached_p1_odd[2];
                                    param2_t2     <= (cached_act_odd[2] == 4'b0000 || cached_act_odd[2] == 4'b0001) ? 4'd0 : cached_p2_odd[2];
                                    data_ready_t2 <= 1'b1;
                                end
                            endcase

                        end else begin
                            // Issue Action Byte Read from SRAM (Offset < 1024)
                            active_base <= calc_sector_base(sel_macro_op);

                            cur_act_off <= next_act_off;
                            cur_prm_off <= next_prm_off;
                            address     <= {2'b00, calc_sector_base(sel_macro_op)} + {5'b00000, next_act_off};
                            state       <= ST_WAIT_ACT;
                            state   <= ST_WAIT_ACT;
                        end
                    end
                end

                // Cycle 1: Latch Action Byte, Drive First Parameter Byte Address (+1024 offset)
                ST_WAIT_ACT: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        latched_act_byte <= bus;
                        address          <= 12'd1024 + {2'b00, active_base} + {5'b00000, cur_prm_off};
                        state            <= ST_WAIT_PRM0;
                    end
                end

                // Cycle 2: Latch First Param Byte (p_byte0)
                ST_WAIT_PRM0: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        latched_prm0 <= bus;
                        // Optimization: if both Op0 and Op1 are 0000 (1+1 = 1 byte 'p p'), no 2nd byte needed
                        if (latched_act_byte[7:4] == 4'b0000 && latched_act_byte[3:0] == 4'b0000 && bus[3:0] != PAD_NIBBLE) begin
                            extra_reads_needed <= 2'd1;
                            state              <= ST_COMMIT;
                        end else begin
                            address <= 12'd1024 + {2'b00, active_base} + {5'b00000, cur_prm_off + 7'd1};
                            state   <= ST_WAIT_PRM1;
                        end
                    end
                end

                // Cycle 3: Latch Second Param Byte (p_byte1) & Check if a 3rd Byte is Required
                ST_WAIT_PRM1: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        latched_prm1 <= bus;
                        if (bus[7:4] == PAD_NIBBLE) begin
                            extra_reads_needed <= 2'd3;
                            address            <= 12'd1024 + {2'b00, active_base} + {5'b00000, cur_prm_off + 7'd2};
                            state              <= ST_WAIT_PRM2;
                        end else begin
                            extra_reads_needed <= 2'd2;
                            state              <= ST_COMMIT;
                        end
                    end
                end

                // Cycle 4: Latch Third Param Byte (p_byte2)
                ST_WAIT_PRM2: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        latched_prm2 <= bus;
                        state        <= ST_COMMIT;
                    end
                end

                // Commit Stage: Output Op0, Cache Op1, and Advance Offsets Past the Pair
                ST_COMMIT: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        request_data[active_t]   <= 1'b0;
                        importance[active_t]     <= 6'd0;
                        cycle_threads            <= 1'b1;
                        has_cached_odd[active_t] <= 1'b1;
                        cached_act_odd[active_t] <= op1_cmd;

                        // Parameter Alignment Multiplexer for Cached Op1
                        if (extra_reads_needed == 2'd1) begin
                            // 1+1 = 1 byte (p p): Op0 got [7:4], Op1 gets [3:0]
                            cached_p0_odd[active_t] <= latched_prm0[3:0];
                            cached_p1_odd[active_t] <= 4'd0;
                            cached_p2_odd[active_t] <= 4'd0;
                        end else if (latched_prm0[3:0] == PAD_NIBBLE || op0_cmd == 4'b0000) begin
                            // First op has 1 parameter ('p x | ...')
                            if (latched_prm1[7:4] == PAD_NIBBLE) begin
                                // Pattern: 1+3 = 3 bytes (p x | x p | p p)
                                cached_p0_odd[active_t] <= latched_prm1[3:0];
                                cached_p1_odd[active_t] <= (op1_cmd == 4'b0000) ? 4'd0 : latched_prm2[7:4];
                                cached_p2_odd[active_t] <= (op1_cmd == 4'b0000 || op1_cmd == 4'b0001) ? 4'd0 : latched_prm2[3:0];
                            end else begin
                                // Pattern: 1+2 = 2 bytes (p x | p p)
                                cached_p0_odd[active_t] <= latched_prm1[7:4];
                                cached_p1_odd[active_t] <= (op1_cmd == 4'b0000) ? 4'd0 : latched_prm1[3:0];
                                cached_p2_odd[active_t] <= 4'd0;
                            end
                        end else begin
                            // First op starts with 'p p | ...' (2 or 3 parameters)
                            if (latched_prm1[7:4] == PAD_NIBBLE) begin
                                // Pattern: 2+2 = 3 bytes (p p | x p | p x)
                                cached_p0_odd[active_t] <= latched_prm1[3:0];
                                cached_p1_odd[active_t] <= (op1_cmd == 4'b0000) ? 4'd0 : latched_prm2[7:4];
                                cached_p2_odd[active_t] <= 4'd0;
                            end else if (latched_prm1[3:0] == PAD_NIBBLE || op0_cmd == 4'b0001) begin
                                // Pattern: 2+1 = 2 bytes (p p | p x)
                                cached_p0_odd[active_t] <= latched_prm1[7:4];
                                cached_p1_odd[active_t] <= 4'd0;
                                cached_p2_odd[active_t] <= 4'd0;
                            end else begin
                                // Pattern: 3+1 = 2 bytes (p p | p p)
                                cached_p0_odd[active_t] <= latched_prm1[3:0];
                                cached_p1_odd[active_t] <= 4'd0;
                                cached_p2_odd[active_t] <= 4'd0;
                            end
                        end

                        // Drive Thread Outputs for Op0 & Advance Offsets by Exact Bytes Consumed
                        case (active_t)
                            2'd0: begin
                                action_t0 <= op0_cmd;
                                param0_t0 <= final_op0_p0;
                                param1_t0 <= final_op0_p1;
                                param2_t0 <= final_op0_p2;
                                microcode_action_offset_t0_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t0_res  <= cur_prm_off + {5'd0, extra_reads_needed};
                                data_ready_t0 <= 1'b1;
                            end
                            2'd1: begin
                                action_t1 <= op0_cmd;
                                param0_t1 <= final_op0_p0;
                                param1_t1 <= final_op0_p1;
                                param2_t1 <= final_op0_p2;
                                microcode_action_offset_t1_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t1_res  <= cur_prm_off + {5'd0, extra_reads_needed};
                                data_ready_t1 <= 1'b1;
                            end
                            2'd2: begin
                                action_t2 <= op0_cmd;
                                param0_t2 <= final_op0_p0;
                                param1_t2 <= final_op0_p1;
                                param2_t2 <= final_op0_p2;
                                microcode_action_offset_t2_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t2_res  <= cur_prm_off + {5'd0, extra_reads_needed};
                                data_ready_t2 <= 1'b1;
                            end
                        endcase

                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule