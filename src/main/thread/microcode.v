`default_nettype none
module microcode(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  macro_op_t0,
    input  wire [7:0]  microcode_action_offset_t0,
    input  wire [7:0]  microcode_param_offset_t0,
    input  wire        next_microop_t0,
    input  wire        flush_t0,
    output reg         data_ready_t0,
    output reg  [3:0]  action_t0,
    output reg  [3:0]  param0_t0,
    output reg  [3:0]  param1_t0,
    output reg  [3:0]  param2_t0,

    input  wire [7:0]  macro_op_t1,
    input  wire [7:0]  microcode_action_offset_t1,
    input  wire [7:0]  microcode_param_offset_t1,
    input  wire        next_microop_t1,
    input  wire        flush_t1,
    output reg         data_ready_t1,
    output reg  [3:0]  action_t1,
    output reg  [3:0]  param0_t1,
    output reg  [3:0]  param1_t1,
    output reg  [3:0]  param2_t1,


    input  wire [7:0]  macro_op_t2,
    input  wire [7:0]  microcode_action_offset_t2,
    input  wire [7:0]  microcode_param_offset_t2,
    input  wire        next_microop_t2,
    input  wire        flush_t2,
    output reg         data_ready_t2,
    output reg  [3:0]  action_t2,
    output reg  [3:0]  param0_t2,
    output reg  [3:0]  param1_t2,
    output reg  [3:0]  param2_t2,


    input  wire [7:0]  bus, //la controller uses this
    output reg  [11:0] address,
    output wire         cycle_threads,
    output wire         microcode_idle
);

    reg  [6:0]  microcode_action_offset_t0_res;
    reg  [6:0]  microcode_param_offset_t0_res;
    
    reg  [6:0]  microcode_action_offset_t1_res;
    reg  [6:0]  microcode_param_offset_t1_res;
    
    reg  [6:0]  microcode_action_offset_t2_res;
    reg  [6:0]  microcode_param_offset_t2_res;
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
                calc_sector_base = 10'd0;
            end
        end
    endfunction

    function [11:0] calc_param_addr(input [9:0] base, input [6:0] prm_off);
        reg [12:0] raw_addr;
        begin
            // 1024 + (base * 3) + prm_off
            raw_addr = 13'd1024 + {3'b000, base} + {2'b00, base, 1'b0} + {6'b000000, prm_off};
            calc_param_addr = (raw_addr >= 13'd4000) ? 12'd3999 : raw_addr[11:0];
        end
    endfunction
    
    reg        request_data   [0:2];
    reg [3:0]  importance     [0:2]; //max 63 //should never be filled all the way up
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
    
    reg [3:0]  wait_cnt;

    //bus wait stuff : fixed 2-cycle min wait + early exit on != 8'hFF, 16-cycle timeout fallback
    wire early_exit   = (wait_cnt >= 4'd2) && (bus != 8'hFF);
    wire timed_out    = (wait_cnt == 4'd15);
    wire data_arrived = early_exit || timed_out;

//high whenever the FSM is idle and no thread is requesting an external fetch
    //(also stays high if a thread is only hitting straight cache)
    assign microcode_idle = !any_request && (state == ST_IDLE); //prevents cycling when still waiting for microcode
    wire   bus_needed     = any_request && !(has_cached_odd[winning_thread] && !flush_vec[winning_thread]);
    assign cycle_threads  = (state == ST_IDLE) && !bus_needed;




    wire [7:0] sel_macro_op = (winning_thread == 2'd0) ? macro_op_t0 :
                              (winning_thread == 2'd1) ? macro_op_t1 : macro_op_t2;
    wire [6:0] sel_act_off  = (winning_thread == 2'd0) ? microcode_action_offset_t0_res :
                              (winning_thread == 2'd1) ? microcode_action_offset_t1_res :
                                                         microcode_action_offset_t2_res;
    wire [6:0] sel_prm_off  = (winning_thread == 2'd0) ? microcode_param_offset_t0_res :
                              (winning_thread == 2'd1) ? microcode_param_offset_t1_res :
                                                         microcode_param_offset_t2_res;

//switched to optimise
    // wire [6:0] next_act_off = flush_vec[winning_thread] ?
    //                           ((winning_thread == 2'd0) ? microcode_action_offset_t0 :
    //                            (winning_thread == 2'd1) ? microcode_action_offset_t1 :
    //                                                       microcode_action_offset_t2) : sel_act_off;
    // wire [6:0] next_prm_off = flush_vec[winning_thread] ?
    //                           ((winning_thread == 2'd0) ? microcode_param_offset_t0 :
    //                            (winning_thread == 2'd1) ? microcode_param_offset_t1 :
    //                                                       microcode_param_offset_t2) : sel_prm_off;


//deterministic unpacking according to action opcode
    wire [3:0] op0_cmd = latched_act_byte[7:4];
    wire [3:0] op1_cmd = latched_act_byte[3:0];

    wire [1:0] n0 = (op0_cmd == 4'b0000) ? 2'd1 : (op0_cmd == 4'b0001) ? 2'd2 : 2'd3;
    wire [1:0] n1 = (op1_cmd == 4'b0000) ? 2'd1 : (op1_cmd == 4'b0001) ? 2'd2 : 2'd3;

    wire [2:0] total_nibs   = {1'b0, n0} + {1'b0, n1}; // Range: 2..6 nibbles
    wire [1:0] bytes_needed = (total_nibs <= 3'd2) ? 2'd1 :
                              (total_nibs <= 3'd4) ? 2'd2 : 2'd3;


    //nibble assignment
    wire [3:0] nib [0:5];
    assign nib[0] = latched_prm0[7:4];
    assign nib[1] = latched_prm0[3:0];
    assign nib[2] = latched_prm1[7:4];
    assign nib[3] = latched_prm1[3:0];
    assign nib[4] = latched_prm2[7:4];
    assign nib[5] = latched_prm2[3:0];

    // Op0 always starts at nib[0]
    wire [3:0] final_op0_p0 = nib[0];
    wire [3:0] final_op0_p1 = (n0 >= 2'd2) ? nib[1] : 4'd0;
    wire [3:0] final_op0_p2 = (n0 == 2'd3) ? nib[2] : 4'd0;

    // Op1 always starts immediately after Op0 at nib[n0]
    wire [3:0] final_op1_p0 = nib[n0];
    wire [3:0] final_op1_p1 = (n1 >= 2'd2) ? nib[{1'b0, n0} + 3'd1] : 4'd0;
    wire [3:0] final_op1_p2 = (n1 == 2'd3) ? nib[{1'b0, n0} + 3'd2] : 4'd0;

    // Select Op0 (even index) or Op1 (odd jump index) for immediate output in ST_COMMIT
    //fixes jump instruction problems
    wire       is_odd_jump  = cur_act_off[0];
    wire [3:0] commit_act   = is_odd_jump ? op1_cmd      : op0_cmd;
    wire [3:0] commit_p0    = is_odd_jump ? final_op1_p0 : final_op0_p0;
    wire [3:0] commit_p1    = is_odd_jump ? final_op1_p1 : final_op0_p1;
    wire [3:0] commit_p2    = is_odd_jump ? final_op1_p2 : final_op0_p2;


    reg [2:0] pending_flush;
    wire [7:0] raw_act_win = (winning_thread == 2'd0) ? microcode_action_offset_t0 :
                         (winning_thread == 2'd1) ? microcode_action_offset_t1 : microcode_action_offset_t2;
    wire [7:0] raw_prm_win = (winning_thread == 2'd0) ? microcode_param_offset_t0 :
                            (winning_thread == 2'd1) ? microcode_param_offset_t1 : microcode_param_offset_t2;

    wire [6:0] conv_act_win, conv_prm_win;
    ternary_to_unsigned_binary_logic_convert_4t u_conv_act (.in_trits(raw_act_win), .out_bin(conv_act_win));
    ternary_to_unsigned_binary_logic_convert_4t u_conv_prm (.in_trits(raw_prm_win), .out_bin(conv_prm_win));

    
    wire use_flush_win = flush_vec[winning_thread] || pending_flush[winning_thread];
    wire [6:0] next_act_off = use_flush_win ? conv_act_win : sel_act_off;
    wire [6:0] next_prm_off = use_flush_win ? conv_prm_win : sel_prm_off;

    integer i;
    
    
    //logik
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_flush <= 3'b000;
            state                          <= ST_IDLE;
            active_t                       <= 2'd0;
            // active_base                    <= 10'd0;
            // cur_act_off                    <= 7'd0;
            // cur_prm_off                    <= 7'd0;
            address                        <= 12'd0;
            // latched_act_byte               <= 8'd0;
            // latched_prm0                   <= 8'd0;
            // latched_prm1                   <= 8'd0;
            // latched_prm2                   <= 8'd0;
            wait_cnt                       <= 4'd0;

            data_ready_t0                  <= 1'b0;
            data_ready_t1                  <= 1'b0;
            data_ready_t2                  <= 1'b0;

            // action_t0  <= 4'd0; param0_t0 <= 4'd0; param1_t0 <= 4'd0; param2_t0 <= 4'd0;
            // action_t1  <= 4'd0; param0_t1 <= 4'd0; param1_t1 <= 4'd0; param2_t1 <= 4'd0;
            // action_t2  <= 4'd0; param0_t2 <= 4'd0; param1_t2 <= 4'd0; param2_t2 <= 4'd0;

            microcode_action_offset_t0_res <= 7'd0;
            microcode_param_offset_t0_res  <= 7'd0;
            microcode_action_offset_t1_res <= 7'd0;
            microcode_param_offset_t1_res  <= 7'd0;
            microcode_action_offset_t2_res <= 7'd0;
            microcode_param_offset_t2_res  <= 7'd0;

            for (i = 0; i < 3; i = i + 1) begin
                request_data[i]   <= 1'b0;
                importance[i]     <= 4'd0;
                has_cached_odd[i] <= 1'b0;
                // cached_act_odd[i] <= 4'd0;
                // cached_p0_odd[i]  <= 4'd0;
                // cached_p1_odd[i]  <= 4'd0;
                // cached_p2_odd[i]  <= 4'd0;
            end
        end else begin
            
            //removed toggle based request for traditional pulse based request
            if (flush_t0) begin
                microcode_action_offset_t0_res <= microcode_action_offset_t0;
                microcode_param_offset_t0_res  <= microcode_param_offset_t0;
                has_cached_odd[0]              <= 1'b0;
                data_ready_t0                  <= 1'b0;
                request_data[0]                <= 1'b1;
                importance[0]                  <= 4'd1;
                pending_flush[0]               <= 1'b1;
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
                importance[1]                  <= 4'd1;
                pending_flush[1]               <= 1'b1;
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
                importance[2]                  <= 4'd1;
                pending_flush[2]               <= 1'b1;
            end else if (next_microop_t2) begin
                data_ready_t2                  <= 1'b0;
                request_data[2]                <= 1'b1;
            end

            //importance inc for every waiting thread (saturating at 63)
            for (i = 0; i < 3; i = i + 1) begin
                if (request_data[i] && !(state == ST_COMMIT && active_t == i[1:0])) begin
                    if (importance[i] != 4'b1111)
                        importance[i] <= importance[i] + 4'd1;
                end
            end

            // -----------------------------------------------------------------
            // Memory Interfacer & Parameter Alignment State Machine
            // -----------------------------------------------------------------
            case (state)
                ST_IDLE: begin
                    wait_cnt <= 4'd0;
                    if (any_request) begin
                        active_t <= winning_thread;
                        // Fast-path: serve cached 2nd micro-op (Op1) in 1 cycle
                        if (has_cached_odd[winning_thread] && !use_flush_win) begin
                            has_cached_odd[winning_thread] <= 1'b0;
                            request_data[winning_thread]   <= 1'b0;
                            importance[winning_thread]     <= 4'd0;
                            

                            case (winning_thread)
                                2'd0: begin
                                    //aligned odd even reads
                                    action_t0                      <= cached_act_odd[0];
                                    param0_t0                      <= cached_p0_odd[0];
                                    param1_t0                      <= cached_p1_odd[0];
                                    param2_t0                      <= cached_p2_odd[0];
                                    microcode_action_offset_t0_res <= microcode_action_offset_t0_res + 7'd1;
                                    data_ready_t0                  <= 1'b1;
                                end
                                2'd1: begin
                                    action_t1                      <= cached_act_odd[1];
                                    param0_t1                      <= cached_p0_odd[1];
                                    param1_t1                      <= cached_p1_odd[1];
                                    param2_t1                      <= cached_p2_odd[1];
                                    microcode_action_offset_t1_res <= microcode_action_offset_t1_res + 7'd1;
                                    data_ready_t1                  <= 1'b1;
                                end
                                2'd2: begin
                                    action_t2                      <= cached_act_odd[2];
                                    param0_t2                      <= cached_p0_odd[2];
                                    param1_t2                      <= cached_p1_odd[2];
                                    param2_t2                      <= cached_p2_odd[2];
                                    microcode_action_offset_t2_res <= microcode_action_offset_t2_res + 7'd1;
                                    data_ready_t2                  <= 1'b1;
                                end
                            endcase

                        end else begin
                            // Issue Action Byte Read from SRAM (Offset < 1024)
                            active_base <= calc_sector_base(sel_macro_op);
                            cur_act_off <= next_act_off;
                            cur_prm_off <= next_prm_off;
                            address     <= {2'b00, calc_sector_base(sel_macro_op)} + {6'b000000, next_act_off[6:1]};
                            state       <= ST_WAIT_ACT;
                            pending_flush[winning_thread] <= 1'b0;
                        end
                    end
                end

                ST_WAIT_ACT: begin
                    if (flush_vec[active_t]) begin
                        wait_cnt <= 4'd0;
                        state    <= ST_IDLE;
                    end else if (!data_arrived) begin
                        wait_cnt <= wait_cnt + 4'd1;// Hold address steady for FPGA BRAM + pad delay
                    end else begin
                        wait_cnt         <= 4'd0;
                        latched_act_byte <= bus;
                        address          <= calc_param_addr(active_base, cur_prm_off);
                        state            <= ST_WAIT_PRM0;
                    end
                end
    
                ST_WAIT_PRM0: begin
                    if (flush_vec[active_t]) begin
                        wait_cnt <= 4'd0;
                        state <= ST_IDLE;
                    end else if (!data_arrived) begin
                       wait_cnt <= wait_cnt + 4'd1;// Hold address steady for FPGA BRAM + pad delay
                    end else begin
                        wait_cnt <= 4'd0;
                        latched_prm0 <= bus;
                        if (bytes_needed == 2'd1) begin
                            state <= ST_COMMIT;
                        end else begin
                            address <= calc_param_addr(active_base, cur_prm_off + 7'd1);
                            state   <= ST_WAIT_PRM1;
                        end
                    end
                end

                //removed next byte read
                ST_WAIT_PRM1: begin
                    if (flush_vec[active_t]) begin
                        wait_cnt <= 4'd0;
                        state <= ST_IDLE;
                    end else if (!data_arrived) begin
                        wait_cnt <= wait_cnt + 4'd1;// Hold address steady for FPGA BRAM + pad delay
                    end else begin
                        wait_cnt <= 4'd0;
                        latched_prm1 <= bus;
                        if (bytes_needed == 2'd2) begin
                            state <= ST_COMMIT;
                        end else begin
                            address <= calc_param_addr(active_base, cur_prm_off + 7'd2);
                            state   <= ST_WAIT_PRM2;
                        end
                    end
                end

                //3rd param
                ST_WAIT_PRM2: begin
                    if (flush_vec[active_t]) begin
                        wait_cnt <= 4'd0;
                        state <= ST_IDLE;
                    end else if (!data_arrived) begin
                        wait_cnt <= wait_cnt + 4'd1; // Hold address steady for FPGA BRAM + pad delay
                    end else begin
                        wait_cnt <= 4'd0;
                        latched_prm2 <= bus;
                        state        <= ST_COMMIT;
                    end
                end

                //this stage needs no bus waits
                // Commit Stage: Output Op0, Cache Op1, and Advance Offsets Past the Pair
                ST_COMMIT: begin
                    if (flush_vec[active_t]) begin
                        state <= ST_IDLE;
                    end else begin
                        request_data[active_t]   <= 1'b0;
                        importance[active_t]     <= 4'd0;

                        // Only cache Op1 if we started on an even micro-op
                        has_cached_odd[active_t] <= ~is_odd_jump;
                        cached_act_odd[active_t] <= op1_cmd;
                        cached_p0_odd[active_t]  <= final_op1_p0;
                        cached_p1_odd[active_t]  <= final_op1_p1;
                        cached_p2_odd[active_t]  <= final_op1_p2;
                        //this is because odd micro ops should not be cached

                        //REMOVED : logic necessary for padding
                        case (active_t)
                            2'd0: begin
                                action_t0                      <= commit_act;
                                param0_t0                      <= commit_p0;
                                param1_t0                      <= commit_p1;
                                param2_t0                      <= commit_p2;
                                microcode_action_offset_t0_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t0_res  <= cur_prm_off + {5'd0, bytes_needed};
                                data_ready_t0                  <= 1'b1;
                            end
                            2'd1: begin
                                action_t1                      <= commit_act;
                                param0_t1                      <= commit_p0;
                                param1_t1                      <= commit_p1;
                                param2_t1                      <= commit_p2;
                                microcode_action_offset_t1_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t1_res  <= cur_prm_off + {5'd0, bytes_needed};
                                data_ready_t1                  <= 1'b1;
                            end
                            2'd2: begin
                                action_t2                      <= commit_act;
                                param0_t2                      <= commit_p0;
                                param1_t2                      <= commit_p1;
                                param2_t2                      <= commit_p2;
                                microcode_action_offset_t2_res <= cur_act_off + 7'd1;
                                microcode_param_offset_t2_res  <= cur_prm_off + {5'd0, bytes_needed};
                                data_ready_t2                  <= 1'b1;
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