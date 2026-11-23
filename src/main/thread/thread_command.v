/*
checklist
1: all valid addresses when doing read or write op should check for aliases first
2: make sure there are no deadlocks
3: listen to thread overlord
4: double check command intake is working
5: microcode too
6: make sure yosys doesn't optimise stuff away (very important!!!!)
7: reset single clock values
8: check command intake again
9: verify all interactions with other modules
10: check aliasing
11: check program counter functionality
12: check command first loading and microcode parsing
13: check micro op stages
14: ensure compliance
15: check non blocking and blocking assignments
16: verify state machine
17: check timing delays and verify everything is ok
18: probably simulate on an fpga
19: from 18, i'll need an even bigger wrapper that does the work for me
20: ladder reg aliasing (just the 12 trit ones)
21: am buffer default states MUST be h55 (to make sure that accidental mask uses don't erase things) 
22: make sure read commands work to transmit asic data to fpga
23: remember direct execution exists
24: as an extra layer of security, make sure is_command is used to modulate command intake
25: common read write wires.
26: memory read write stuff
27. double triple check the command decodings of xxx0
28. fpga generally is at microcode so remember to make it switch before sending anything
29. check if the single use masks are respected
30. there is no mask active reg. just push in h55 and call it a day
31. it should always be 0 : no mask, 1 : use mask
32. tritwise works on 1 trit
33. something something spi
34. direct execution its a thing
35. alias write is kind of old, need fixing
36. make sure the cycle threads command and micro stuff is being called
*/

`default_nettype none
module thread_command(
    input wire clk,
    input wire rst_n,
    //from the microcode parsers
        output wire [7:0] macro_op,
        output wire [7:0] action_counter,
        output wire [7:0] param_counter,
        output reg flush,
        input wire [3:0] command,
        input wire [3:0] param_0,
        input wire [3:0] param_1,
        input wire [3:0] param_2,
        output reg  command_req_new, // remember this is not a strobe!
        input  wire command_data_sent,

        // input [6:0] microcode_action_offset_t0, //up to 128 but since its 4 trit to convert to this it's more like 81
        //input [6:0] microcode_param_offset_t0,
        //.flush(),//this forces the microcode handler to pause, flush cache and read from action position & param position, as well as macro op position
        // output wire ,
        /*
        when new command loaded, pull flush to make the microcode parser read new macro op and new param and action counter
        then we are done
        wait for command data sent, then the actual command evaluation can begin
        command req new is pulled once a micro op is finished
        */

    //alu interface
        output  reg [2:0] alu_command,
        output  reg [2:0] tritwise_command,
        output  reg [2:0] tritshift_tritwise_command, //trit shifts up to 3 front and back
        output  reg alu_command_ready,
        input   wire alu_res_ready,
        //4 trit
        output reg [7:0] alu_t4_a,
        output reg [7:0] alu_t4_b,
        output reg [7:0] alu_t4_c,
        output reg [1:0] alu_t1_a,
        output reg [1:0] alu_t1_b,
        output reg alu_prio_inc,
        //results here
        input wire [7:0] alu_t4_o,
        input wire [1:0] alu_t1_o,
        input wire [15:0] alu_multiply_o,
        output reg [3:0] hog_timer,

    //overlord interface
        input   wire [7:0] bus_data_in,
        input   wire data_valid,
        input   wire data_done,
        output  reg request_set_pc,
        output  reg request_memory_access,
        output  reg is_write_memory,
        output  reg [23:0] addr_out,
        input   wire [23:0] d_in,
        output  reg [23:0] d_out,
        output  reg pipe_full_rej,
        output  reg request_new_cmd,



    //debug core components
    //when profiling core loads info, it comes from d_in and leaves using d_out
        input   wire [1:0] register_read_stage,       // Byte chunk (0>1>2)
        input   wire [4:0] register_address,          // Flat 5-bit address (0..31) incremented per handover 
        //if reg address is not zero, read is active
        input   wire is_read,                   // 1 = ASIC transmits to FPGA, 0 = ASIC receives from FPGA
        output  wire reg_handover_done,         // Pulled high by active thread when entire register dump finishes
        output  reg [7:0] dbg_dout,
        input   wire [7:0] dbg_din,    
        input   wire       microcode_fence, //pauses execute after 1 micro op
        input   wire       direct_instr,    //skips the buffer; always hold 1 instruction
        input   wire       single_execute, //basically take 1 step
        output  reg       single_execute_ack,
        input   wire       cmd_buf_clr, //clear command buffers
        output  reg       cmd_buf_clr_ack,

    //error
        output wire pc_overflow,
        output wire a_ctr_overflow,
        output wire p_ctr_overflow,
        output wire reg_overflow,
        output wire [3:0] offending_id,

    //bus comms
    input  wire        bus_is_cmd,      //1 = macro-op byte, 0 = 24-bit param byte
    input  wire [2:0]  bus_param_idx,   //target param register index (from ui_in[7:5])
    input  wire [1:0]  bus_byte_phase,  //incoming byte chunk (00, 01, 10)
    output reg         request_show_reg,//triggers req_type 3'b001 (0001:1101)
    output reg  [15:0] ext_addr_out    // 8-trit (16-bit) high memory address (0001:1100)
    //the actual bytes come in through bus_data_phase

    // //spi
    // output reg  [11:0] spi_tx_data,      // 12 bits: 1 for +1 (2'b01), 0 otherwise (nti)
    // output reg         spi_tx_start,     // Held high until spi_done
    // output reg  [3:0]  spi_clk_div,      // 1..15 clock divider
    // output reg         spi_div_we,       // 1-cycle strobe when 0001:1111 updates divider
    // input  wire        spi_busy,
    // input  wire        spi_done
);

reg       reg_ovf_r;
reg [3:0] offending_id_r;

//moved below
//shared read ports
    reg [3:0] rd_addr_12t;
    reg [2:0] rd_addr_4t_a, rd_addr_4t_b, rd_addr_4t_c;
    reg [2:0] rd_addr_1t_a, rd_addr_1t_b;


    wire addr4t_a_hits_hold = (hold_reg_addr == 4'd1 && rd_addr_4t_a <= 3'd2) ||
                          (hold_reg_addr == 4'd2 && rd_addr_4t_a >= 3'd3 && rd_addr_4t_a <= 3'd5) ||
                          (hold_reg_addr == 4'd3 && rd_addr_4t_a >= 3'd6);
    wire addr4t_b_hits_hold = (hold_reg_addr == 4'd1 && rd_addr_4t_b <= 3'd2) ||
                            (hold_reg_addr == 4'd2 && rd_addr_4t_b >= 3'd3 && rd_addr_4t_b <= 3'd5) ||
                            (hold_reg_addr == 4'd3 && rd_addr_4t_b >= 3'd6);
    wire addr4t_c_hits_hold = (hold_reg_addr == 4'd1 && rd_addr_4t_c <= 3'd2) ||
                            (hold_reg_addr == 4'd2 && rd_addr_4t_c >= 3'd3 && rd_addr_4t_c <= 3'd5) ||
                            (hold_reg_addr == 4'd3 && rd_addr_4t_c >= 3'd6);

    reg hold_stall;
    
    //shared read ports
    wire [23:0] shared_rd_12t = read_any_reg_12t(
        rd_addr_12t, using_macroop_1, alias_addr_12t, am_buff_12t,
        sp0_trib0, sp0_trib1, sp0_trib2, sp1_trib0, sp1_trib1, sp1_trib2,
        spf_trib0, spf_trib1, spf_trib2, mac_a, mac_b, mac_d,
        pc_r_data, gp_reg_6, gp_reg_7, gp_reg_8,
        cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
        cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
    );

    wire [7:0] shared_rd_4t_a = read_any_reg_4t(
        rd_addr_4t_a, alias_active_4t, alias_addr_4t, am_buff_4t,
        sp0_trib0, sp0_trib1, sp0_trib2, sp1_trib0, sp1_trib1, sp1_trib2,
        spf_trib0, spf_trib1
    );

    wire [7:0] shared_rd_4t_b = read_any_reg_4t(
        rd_addr_4t_b, alias_active_4t, alias_addr_4t, am_buff_4t,
        sp0_trib0, sp0_trib1, sp0_trib2, sp1_trib0, sp1_trib1, sp1_trib2,
        spf_trib0, spf_trib1
    );

    wire [7:0] shared_rd_4t_c = read_any_reg_4t(
        rd_addr_4t_c, alias_active_4t, alias_addr_4t, am_buff_4t,
        sp0_trib0, sp0_trib1, sp0_trib2, sp1_trib0, sp1_trib1, sp1_trib2,
        spf_trib0, spf_trib1
    );

    wire [1:0] shared_rd_1t_a = read_any_reg_1t(
        rd_addr_1t_a, alias_active_1t, alias_addr_1t, am_buff_1t,
        spf_flag0, spf_flag1, spf_flag2, spf_flag3,
        spf_flag4, spf_flag5, spf_flag6, spf_flag7
    );

    wire [1:0] shared_rd_1t_b = read_any_reg_1t(
        rd_addr_1t_b, alias_active_1t, alias_addr_1t, am_buff_1t,
        spf_flag0, spf_flag1, spf_flag2, spf_flag3,
        spf_flag4, spf_flag5, spf_flag6, spf_flag7
    );

//data transmit
localparam [1:0]
    TX_SHOW_REG  = 2'd0, // req_type 001: Send 24-bit d_out
    TX_SET_PC    = 2'd1, // req_type 110: Send 24-bit pc_r_data (via d_out)
    TX_MEM_READ  = 2'd2, // req_type 100: Send 24-bit addr_out (+ ext_addr_out), receive 24-bit d_in
    TX_MEM_WRITE = 2'd3; // req_type 101: Send 24-bit addr_out (+ ext_addr_out) + 24-bit d_out

//macro op governing section and state machine
        reg [15:0] macroop_register;
        reg using_macroop_1;//remember to only flip after macro op 1 finishes

        // Expose active macro-op to the microcode parser
        assign macro_op = using_macroop_1 && !direct_instr ? macroop_register[15:8] : macroop_register[7:0];

        // Thread buffer occupancy states
        localparam  THREAD_EMPTY  = 2'b00, // Neither slot has a valid command
                    THREAD_FULL   = 2'b01, // Both slots occupied
                    THREAD_EMPTY0 = 2'b10, // Slot 0 empty, Slot 1 active
                    THREAD_EMPTY1 = 2'b11; // Slot 1 empty, Slot 0 active
        reg [1:0]   threadstate = THREAD_EMPTY;

//moved to higher level in order to widen bus
        //parameter registers
        reg [23:0] cmd0_reg_0;
        reg [23:0] cmd0_reg_1;
        reg [23:0] cmd0_reg_2;
        reg [23:0] cmd0_reg_3;
        reg [23:0] cmd0_reg_4;
        reg [23:0] cmd0_reg_5;

        
        reg [23:0] cmd1_reg_0;
        reg [23:0] cmd1_reg_1;
        reg [23:0] cmd1_reg_2;
        reg [23:0] cmd1_reg_3;
        reg [23:0] cmd1_reg_4;
        reg [23:0] cmd1_reg_5;

        reg [23:0] gp_reg_6;
        reg [23:0] gp_reg_7;
        reg [23:0] gp_reg_8;
        //also the program counter here

        //when this entire command is done,
        //increment pc
        //load in registers (this can be done WHILE another command is being excuted)
        //profit...
        reg        pc_inc_enable;

        wire [23:0] pc_r_data;
        reg [23:0] pc_w_data;
        
        
        reg pc_w_enable;
        
        wire pc_r_enable;
        

        ternary_pc_12t program_counter (
        .clk(clk),
        .rst_n(rst_n),
        .inc_enable(pc_inc_enable),
        .w_enable(pc_w_enable),    // High to overwrite PC with w_data (Jump/Branch)
        .r_enable(pc_r_enable),    // High to drive PC onto r_data bus
        .w_data(pc_w_data),      // 12-trit (24-bit) branch target input
        .r_data(pc_r_data),      // 12-trit (24-bit) gated read output
        .overflow(pc_overflow)     // Pulses high when wrapping from all +1s to all -1s
        );

// scratchpads
        reg [7:0] sp0_trib0;
        reg [7:0] sp0_trib1;
        reg [7:0] sp0_trib2;
        
        reg [7:0] sp1_trib0;
        reg [7:0] sp1_trib1;
        reg [7:0] sp1_trib2;
        
        reg [1:0] spf_flag0;
        reg [1:0] spf_flag1;
        reg [1:0] spf_flag2;
        reg [1:0] spf_flag3;
        reg [1:0] spf_flag4;
        reg [1:0] spf_flag5;
        reg [1:0] spf_flag6;
        reg [1:0] spf_flag7;

        wire [7:0] spf_trib0;
        assign spf_trib0 = {spf_flag3, spf_flag2, spf_flag1, spf_flag0};
        wire [7:0] spf_trib1;
        assign spf_trib1 = {spf_flag7, spf_flag6, spf_flag5, spf_flag4};

        reg  [7:0] spf_trib2; // cannot be accessed normally. basically just to keep the data there to prevent losing it

        reg [7:0] mac_a;
        reg [7:0] mac_b;
        reg [7:0] mac_c;
        reg [7:0] mac_d;
        reg [1:0] mac_carry;

        reg [7:0] param_ctr;
        reg [7:0] action_ctr;

        reg [1:0] carry_reg;

wire [7:0] mac_sum;
wire [1:0] mac_c0, mac_c1, mac_c2, mac_cout;

bt_fa_1t u_fa0 (
    .a    (mac_b[1:0]),
    .b    (mac_c[1:0]),
    .cin  (mac_carry),
    .sum  (mac_sum[1:0]),
    .cout (mac_c0)
);

bt_fa_1t u_fa1 (
    .a    (mac_b[3:2]),
    .b    (mac_c[3:2]),
    .cin  (mac_c0),
    .sum  (mac_sum[3:2]),
    .cout (mac_c1)
);

bt_fa_1t u_fa2 (
    .a    (mac_b[5:4]),
    .b    (mac_c[5:4]),
    .cin  (mac_c1),
    .sum  (mac_sum[5:4]),
    .cout (mac_c2)
);

bt_fa_1t u_fa3 (
    .a    (mac_b[7:6]),
    .b    (mac_c[7:6]),
    .cin  (mac_c2),
    .sum  (mac_sum[7:6]),
    .cout (mac_cout)
);



assign reg_handover_done = (register_address == 5'd25);

//aliased buffers / masks
    //stands for alias mask buffer
        reg  [23:0] am_buff_12t;
        reg  [7:0] am_buff_4t;
        reg  [1:0] am_buff_1t;
        reg [3:0] alias_addr_12t; // the 0000 state is to mark it to be used as a mask
        reg [2:0] alias_addr_4t;
        reg [2:0] alias_addr_1t;
        reg alias_active_1t; //since 1 t has a valid 0 address, one more is needed
        reg alias_active_4t; //since 4 t has a valid 0 address, one more is needed

        reg mask_12t_singleuse;
        reg mask_4t_singleuse;
        reg mask_1t_singleuse;

    //micro operations
        reg ret_micro;
        reg [23:0] cmd_param_reg;
        reg [2:0]  cmd_param_id;
        reg        cmd_we;
        reg [1:0]  op_read_state; // 00: idle, 01: byte 0, 10: byte 1, 11: byte 2 & write

reg can_execute_this_clock = 1'b1;
reg [3:0]  exec_cmd_lat, exec_p0_lat, exec_p1_lat, exec_p2_lat;
reg [7:0]  alu_res_4t_lat;
reg [1:0]  alu_res_1t_lat;
reg [15:0] alu_res_mul_lat;
reg [3:0]  mac_ovf_target_12t; // Set by 1param:0001
reg [3:0]  hold_reg_addr;      // Set by 1param:0100
reg        hold_reg_active;

reg execute_done = 1'b0;

//i should find a use for them later

reg pc_tx_pending; // Reset to 1'b0 in if (!rst_n)

assign pc_r_enable = (register_address == 5'd5) ||
                     (rd_addr_12t == 4'd5) ||
                     pc_tx_pending;
//assign pc_r_enable = (register_address == 5'd5) || (r_enable && t_addr_in == 4'd5);

wire collision;
assign collision = check_reg_collision(
    (write_allow && do_housekeeping && (alias_addr_12t >= 4'd6)),
    alias_addr_12t, using_macroop_1, 1'b1, ~using_macroop_1,
    cmd_we,cmd_param_id
);

assign reg_overflow   = reg_ovf_r;
assign offending_id   = offending_id_r;
assign a_ctr_overflow = (action_ctr > 8'd80);
assign p_ctr_overflow = (param_ctr  > 8'd80);

assign action_counter = action_ctr;
assign param_counter = param_ctr;

reg do_housekeeping = 1'b0;

reg [1:0] execute_stage = 2'b00;
reg loading_second_slot = 1'b0;
reg [23:0] tmp_12t_lat;

reg write_allow = 1'b1;  


//declarations
reg       direct_exec_active;
reg       direct_exec_ready;
reg [3:0] direct_cmd, direct_p0, direct_p1, direct_p2;

//muxed execution inputs (use exec_* in rd_addr_* mux and Stage 0/1/2)
wire [3:0] exec_cmd   = direct_exec_ready ? direct_cmd : command;
wire [3:0] exec_p0    = direct_exec_ready ? direct_p0  : param_0;
wire [3:0] exec_p1    = direct_exec_ready ? direct_p1  : param_1;
wire [3:0] exec_p2    = direct_exec_ready ? direct_p2  : param_2;
wire       exec_valid = direct_exec_ready || (command_data_sent && (threadstate != THREAD_EMPTY));


reg [23:0] debug_reg_cache = 24'b0;
reg pipe_empty = 1'b0;

reg cmd_target_bank;

reg [4:0]  prev_reg_addr;
reg [1:0]  prev_reg_stage;
reg        prev_is_read;
wire [23:0] dbg_wdata_24b = {dbg_din, debug_reg_cache[15:0]};

always @(*) begin
        rd_addr_12t  = 4'd0;
        rd_addr_4t_a = 3'd0;
        rd_addr_4t_b = 3'd0;
        rd_addr_4t_c = 3'd0;
        rd_addr_1t_a = 3'd0;
        rd_addr_1t_b = 3'd0;

        if (register_address != 5'd0 && register_address < 5'd15) begin
            rd_addr_12t = register_address[3:0];
        end else begin
            case (exec_cmd)
                4'b0001: begin // 1-Param Ops
                    rd_addr_12t  = exec_p1;
                    rd_addr_4t_a = exec_p1[3:1];
                    rd_addr_1t_a = exec_p1[3:1];
                end

                4'b0010: begin // 2-Param Ops
                    rd_addr_12t  = (execute_stage == 2'b00) ? exec_p1 : exec_p2;

                    rd_addr_4t_a = (exec_p0 == 4'b1111) ? exec_p1[2:0] : exec_p1[3:1];
                    rd_addr_4t_b = (exec_p0 == 4'b1111) ? exec_p2[2:0] : exec_p2[3:1];
                    rd_addr_1t_a = exec_p1[3:1];
                    rd_addr_1t_b = exec_p2[3:1];
                end

                // --- Unified xxx0 3-Parameter Ops ---
                4'b0011,    // Add / Sub:      param_0[3:1]=A, exec_p1[3:1]=B
                4'b0100:    // Reworked MAC:   param_0[3:1]=A, exec_p1[3:1]=B
                begin
                    rd_addr_4t_a = exec_p0[3:1];
                    rd_addr_4t_b = exec_p1[3:1];
                end

                4'b0101: begin // Add3 (Packed: aaab bbcc cddd)
                    rd_addr_4t_a = exec_p0[3:1];
                    rd_addr_4t_b = {exec_p0[0], exec_p1[3:2]};
                    rd_addr_4t_c = {exec_p1[1:0], exec_p2[3]};
                end

                4'b0110: begin // Trit Shift: param_0[3:1]=A (4t), exec_p1[3:1]=amt (1t)
                    rd_addr_4t_a = exec_p0[3:1];
                    rd_addr_1t_a = exec_p1[3:1];
                end

                4'b0111: begin // Single-Instruction Conditional: param_0[3:1]=eval (1t)
                    rd_addr_1t_a = exec_p0[3:1];
                end

                4'b1100: begin // 12t<->4t (exec_p1[0]=0) or 4t<->1t (exec_p1[0]=1)
                    rd_addr_12t  = exec_p0;
                    rd_addr_4t_a = exec_p1[0] ? exec_p0[3:1] : exec_p1[3:1];
                    rd_addr_1t_a = exec_p1[3:1];
                end
                4'b1000, 4'b1001, 4'b1010: begin // Tritwise 1..3: A=exec_p0[3:1], B=exec_p1[3:1]
                    rd_addr_1t_a = exec_p0[3:1];
                    rd_addr_1t_b = exec_p1[3:1];
                end
                default: ;
            endcase
        end
    end

always @(*) begin
        hold_stall = 1'b0;
        if (hold_reg_active && (hold_reg_addr != 4'd0)) begin
            case (exec_cmd)
                4'b0001: begin
                    case (exec_p0)
                        4'b0100, 4'b1010: hold_stall = 1'b0; // Don't stall hold-set or TX_MEM_READ
                        4'b0101, 4'b0110, 4'b1100, 4'b1101, 4'b1110: hold_stall = (rd_addr_12t == hold_reg_addr);
                        4'b0111, 4'b1001: hold_stall = addr4t_a_hits_hold;
                        4'b1000:          hold_stall = (hold_reg_addr == 4'd3);
                        default:          hold_stall = 1'b0;
                    endcase
                end
                4'b0010: begin
                    case (exec_p0)
                        4'b0000, 4'b0010, 4'b1001, 4'b1110: hold_stall = (exec_p1 == hold_reg_addr);
                        4'b0101:          hold_stall = (exec_p1 == hold_reg_addr) || (exec_p2 == hold_reg_addr);
                        4'b0100:          hold_stall = addr4t_a_hits_hold;
                        4'b0110, 4'b1111: hold_stall = addr4t_a_hits_hold || addr4t_b_hits_hold;
                        4'b0011, 4'b0111: hold_stall = (hold_reg_addr == 4'd3);
                        default:          hold_stall = 1'b0;
                    endcase
                end
                4'b0011, 4'b0100: hold_stall = addr4t_a_hits_hold || addr4t_b_hits_hold;
                4'b0101:          hold_stall = addr4t_a_hits_hold || addr4t_b_hits_hold || addr4t_c_hits_hold;
                4'b0110:          hold_stall = addr4t_a_hits_hold || (hold_reg_addr == 4'd3);
                4'b0111, 4'b1000: hold_stall = (hold_reg_addr == 4'd3);
                4'b1100:          hold_stall = (rd_addr_12t == hold_reg_addr) || addr4t_a_hits_hold || (hold_reg_addr == 4'd3);
                default:          hold_stall = 1'b0;
            endcase
        end
    end


//register section
//12 trit read
function [23:0] read_any_reg_12t;
    input [3:0]  addr;
    input        sel_cmd1;
    // Alias check inputs
    input [3:0]  alias_addr_12t;
    input [23:0] am_buff_12t;
    // Scratchpads, Flags (with spf_trib2), Ladder
    input [7:0]  sp0_t0, sp0_t1, sp0_t2;
    input [7:0]  sp1_t0, sp1_t1, sp1_t2;
    input [7:0]  spf_t0, spf_t1, spf_t2;
    
    //ladder
    input [7:0] mac_a, mac_b,mac_d; // Incoming product:   | D | C |
    // PC & Secondary Registers
    input [23:0] pc_val;
    input [23:0] gp6, gp7, gp8;
    input [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    input [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    begin
        if (addr == 4'd0) begin
            read_any_reg_12t = 24'hAAAAAA;
        end else if (alias_addr_12t == addr) begin
            // Active 12t alias intercept
            read_any_reg_12t = am_buff_12t;
        end else begin
            case (addr)
                4'd1: read_any_reg_12t = {sp0_t2, sp0_t1, sp0_t0};
                4'd2: read_any_reg_12t = {sp1_t2, sp1_t1, sp1_t0};
                4'd3: read_any_reg_12t = {spf_t2, spf_t1, spf_t0};
                4'd4: read_any_reg_12t = {mac_d, mac_b, mac_a};
                default: read_any_reg_12t = read_secondary_reg(
                    1'b1, addr, using_macroop_1, pc_val,
                    gp6, gp7, gp8,
                    c0_0, c0_1, c0_2, c0_3, c0_4, c0_5,
                    c1_0, c1_1, c1_2, c1_3, c1_4, c1_5
                );
            endcase
        end
    end
endfunction

// task write_any_reg_12t;
//     input use_mask;
//     input        write_allow;
//     input [3:0]  addr;
//     input        sel_cmd1;
//     // Alias check inputs
//     inout [3:0]  alias_addr_12t;
//     inout [23:0] am_12t;
//     input [23:0] data_12t;
//     output pc_wen;
//     inout [23:0] pc_val;
//     // Scratchpads, Flags (with spf_trib2), Ladder
//     inout [7:0]  sp0_t0, sp0_t1, sp0_t2;
//     inout [7:0]  sp1_t0, sp1_t1, sp1_t2;
//     inout [7:0]  spf_t2;
//     inout [1:0] f0, f1, f2, f3, f4, f5, f6, f7;
//     //ladder
//     inout [7:0] mac_a,mac_b,mac_d; // Incoming product:   | D | C |
//     // PC & Secondary Registers
//     inout [23:0] gp6, gp7, gp8;
//     inout [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
//     inout [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    
//     reg [23:0] wdata;
//     begin
//         if(write_allow) begin
//             wdata = (alias_addr_12t == 4'd0 && use_mask) ? apply_mask_12t(data_12t, am_12t) : data_12t;
//             case (addr)
//                 4'd1: {sp0_t2, sp0_t1, sp0_t0} <= wdata;
//                 4'd2: {sp1_t2, sp1_t1, sp1_t0} <= wdata;
//                 4'd3: {spf_t2, f7, f6, f5, f4, f3, f2, f1, f0} <= wdata;
//                 4'd4: {mac_d, mac_b, mac_a} <= wdata;
//                 default: write_secondary_reg(
//                     1'b1, addr, sel_cmd1, use_mask, alias_addr_12t, am_12t, data_12t, pc_wen, pc_val,
//                     gp6, gp7, gp8,
//                     c0_0, c0_1, c0_2, c0_3, c0_4, c0_5,
//                     c1_0, c1_1, c1_2, c1_3, c1_4, c1_5
//                 );
//             endcase
//             if (alias_addr_12t == addr) begin
//                 alias_addr_12t <= 4'd0;
//                 am_12t         <= 24'h555555;
//             end
//         end
//     end
// endtask

//masking funcs
function [1:0] apply_mask_1t;
    input [1:0] val;
    input [1:0] m;
    reg out_ovr;
    begin
        out_ovr       = val[1] | m[1];
        apply_mask_1t = {out_ovr, (~out_ovr) & (val[0] ~^ m[0])};
    end
endfunction //shrunked

    // function [1:0] apply_mask_1t;
    //     input [1:0] val;
    //     input [1:0] m;
    //     begin
    //         case (m)
    //             2'b01: apply_mask_1t = (val == 2'b11) ? 2'b10 : val; // * (+1)
    //             2'b00: begin                                         // * (-1)
    //                 case (val)
    //                     2'b00:   apply_mask_1t = 2'b01;
    //                     2'b01:   apply_mask_1t = 2'b00;
    //                     default: apply_mask_1t = 2'b10;
    //                 endcase
    //             end
    //             default: apply_mask_1t = 2'b10;                      // * (0)
    //         endcase
    //     end
    // endfunction

function [7:0] apply_mask_4t;
    input [7:0] val;
    input [7:0] m;
    begin
        apply_mask_4t = {
            apply_mask_1t(val[7:6], m[7:6]),
            apply_mask_1t(val[5:4], m[5:4]),
            apply_mask_1t(val[3:2], m[3:2]),
            apply_mask_1t(val[1:0], m[1:0])
        };
    end
endfunction

function [23:0] apply_mask_12t;
    input [23:0] val;
    input [23:0] m;
    begin
        apply_mask_12t = {
            apply_mask_4t(val[23:16], m[23:16]),
            apply_mask_4t(val[15:8],  m[15:8]),
            apply_mask_4t(val[7:0],   m[7:0])
        };
    end
endfunction

task write_secondary_reg;
    input        w_allow;
    input [3:0]  addr;
    input        sel_c1;
    input [23:0] wdata;
    begin
        if (w_allow) begin
            case (addr)
                4'd5:  begin
                    pc_w_data   <= wdata;
                    pc_w_enable <= 1'b1;
                    on_pc_modified;
                end
                4'd6:  gp_reg_6 <= wdata;
                4'd7:  gp_reg_7 <= wdata;
                4'd8:  gp_reg_8 <= wdata;
                4'd9:  if (sel_c1) cmd1_reg_0 <= wdata; else cmd0_reg_0 <= wdata;
                4'd10: if (sel_c1) cmd1_reg_1 <= wdata; else cmd0_reg_1 <= wdata;
                4'd11: if (sel_c1) cmd1_reg_2 <= wdata; else cmd0_reg_2 <= wdata;
                4'd12: if (sel_c1) cmd1_reg_3 <= wdata; else cmd0_reg_3 <= wdata;
                4'd13: if (sel_c1) cmd1_reg_4 <= wdata; else cmd0_reg_4 <= wdata;
                4'd14: if (sel_c1) cmd1_reg_5 <= wdata; else cmd0_reg_5 <= wdata;
                default: ;
            endcase
        end
    end
endtask

task write_any_reg_12t;
    input        use_mask;
    input        w_allow;
    input [3:0]  addr;
    input        sel_c1;
    input [23:0] data_12t;
    reg   [23:0] wdata;
    begin
        if (w_allow) begin
            wdata = (alias_addr_12t == 4'd0 && use_mask) ? apply_mask_12t(data_12t, am_buff_12t) : data_12t;
            case (addr)
                4'd1: {sp0_trib2, sp0_trib1, sp0_trib0} <= wdata;
                4'd2: {sp1_trib2, sp1_trib1, sp1_trib0} <= wdata;
                4'd3: {spf_trib2, spf_flag7, spf_flag6, spf_flag5, spf_flag4,
                                  spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= wdata;
                4'd4: {mac_d, mac_b, mac_a} <= wdata;
                default: write_secondary_reg(1'b1, addr, sel_c1, wdata);
            endcase
            if (alias_addr_12t == addr && addr != 4'd0) begin
                alias_addr_12t <= 4'd0;
                am_buff_12t    <= 24'h555555;
            end
        end
    end
endtask

function [23:0] read_secondary_reg;
    input        r_en;
    input [3:0]  addr;
    input        sel_cmd1;
    input [23:0] pc_val;
    input [23:0] gp6, gp7, gp8;
    input [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    input [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    begin
        if (!r_en) begin
            read_secondary_reg = 24'hAAAAAA;
        end else begin
            case (addr)
                4'd5:  read_secondary_reg = pc_val;
                4'd6:  read_secondary_reg = gp6;
                4'd7:  read_secondary_reg = gp7;
                4'd8:  read_secondary_reg = gp8;
                4'd9:  read_secondary_reg = sel_cmd1 ? c1_0 : c0_0;
                4'd10: read_secondary_reg = sel_cmd1 ? c1_1 : c0_1;
                4'd11: read_secondary_reg = sel_cmd1 ? c1_2 : c0_2;
                4'd12: read_secondary_reg = sel_cmd1 ? c1_3 : c0_3;
                4'd13: read_secondary_reg = sel_cmd1 ? c1_4 : c0_4;
                4'd14: read_secondary_reg = sel_cmd1 ? c1_5 : c0_5;
                default: read_secondary_reg = 24'hAAAAAA;
            endcase
        end
    end
endfunction

// task write_secondary_reg;
//     input        write_allow;
//     input [3:0]  addr;
//     input        sel_cmd1;
//     input use_mask;
//     inout [3:0] alias_addr_12t;
//     inout [23:0] am_12t;
//     input [23:0] data_12t;
//     output pc_wen;
//     inout [23:0] pc_val;
//     inout [23:0] gp6, gp7, gp8;
//     inout [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
//     inout [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;

//     reg [23:0] wdata;
//     begin

//         if(write_allow) begin
//             wdata = (alias_addr_12t == 4'd0 && use_mask) ? apply_mask_12t(data_12t, am_12t) : data_12t;
//             case (addr)
//                 4'd5:  begin
//                     pc_val = wdata;
//                     pc_wen = 1'b1;
//                 end
//                 4'd6: gp6 = wdata;
//                 4'd7: gp7 = wdata;
//                 4'd8: gp8 = wdata;
//                 default: begin 
//                     if(sel_cmd1) begin
//                         case (addr)
//                             4'd9:  c1_0 = wdata;
//                             4'd10: c1_1 = wdata;
//                             4'd11: c1_2 = wdata;
//                             4'd12: c1_3 = wdata;
//                             4'd13: c1_4 = wdata;
//                             4'd14: c1_5 = wdata;
//                         endcase
//                     end
//                     else begin
//                         case (addr)
//                             4'd9:  c0_0 = wdata;
//                             4'd10: c0_1 = wdata;
//                             4'd11: c0_2 = wdata;
//                             4'd12: c0_3 = wdata;
//                             4'd13: c0_4 = wdata;
//                             4'd14: c0_5 = wdata;
//                         endcase
//                     end
//                 end
//             endcase
//             if (alias_addr_12t == addr) begin
//                 alias_addr_12t = 4'd0;
//                 am_12t         = 24'h555555;
//             end
//         end
//     end
// endtask

task write_any_reg_4t;
    input       use_mask;
    input       w_allow;
    input [2:0] addr_4t;
    input [7:0] data_4t;
    reg   [7:0] wdata;
    begin
        if (w_allow) begin
            wdata = (!alias_active_4t && use_mask) ? apply_mask_4t(data_4t, am_buff_4t) : data_4t;
            case (addr_4t)
                3'd0: sp0_trib0 <= wdata;
                3'd1: sp0_trib1 <= wdata;
                3'd2: sp0_trib2 <= wdata;
                3'd3: sp1_trib0 <= wdata;
                3'd4: sp1_trib1 <= wdata;
                3'd5: sp1_trib2 <= wdata;
                3'd6: {spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= wdata;
                3'd7: {spf_flag7, spf_flag6, spf_flag5, spf_flag4} <= wdata;
            endcase
            if (alias_active_4t && (alias_addr_4t == addr_4t)) begin
                alias_active_4t <= 1'b0;
                am_buff_4t      <= 8'h55;
            end
        end
    end
endtask

function [7:0] read_any_reg_4t;
    input [2:0] addr_4t;
    input       alias_active_4t;
    input [2:0] alias_addr_4t;
    input [7:0] am_buff_4t;
    input [7:0] sp0_t0, sp0_t1, sp0_t2;
    input [7:0] sp1_t0, sp1_t1, sp1_t2;
    input [7:0] spf_t0, spf_t1;
    begin
        if (alias_active_4t && (alias_addr_4t == addr_4t)) begin
            read_any_reg_4t = am_buff_4t;
        end else begin
            case (addr_4t)
                3'd0: read_any_reg_4t = sp0_t0;
                3'd1: read_any_reg_4t = sp0_t1;
                3'd2: read_any_reg_4t = sp0_t2;
                3'd3: read_any_reg_4t = sp1_t0;
                3'd4: read_any_reg_4t = sp1_t1;
                3'd5: read_any_reg_4t = sp1_t2;
                3'd6: read_any_reg_4t = spf_t0;
                3'd7: read_any_reg_4t = spf_t1;
            endcase
        end
    end
endfunction

    // task write_any_reg_4t;
    //     input use_mask;
    //     input write_allow;
    //     input [2:0] addr_4t;
    //     inout       alias_active_4t;
    //     input [2:0] alias_addr_4t;
    //     input [7:0] data_4t;

    //     inout [7:0] am_4t;
    //     inout [7:0] sp0_t0, sp0_t1, sp0_t2;
    //     inout [7:0] sp1_t0, sp1_t1, sp1_t2;
    //     inout [1:0] f0, f1, f2, f3, f4, f5, f6, f7;


    //     reg [7:0] wdata;
    //     begin
    //         if(write_allow) begin
    //             wdata = (!alias_active_4t && use_mask) ? apply_mask_4t(data_4t, am_4t) : data_4t;
    //             case (addr_4t)
    //                 3'd0: sp0_t0 = wdata;
    //                 3'd1: sp0_t1 = wdata;
    //                 3'd2: sp0_t2 = wdata;
    //                 3'd3: sp1_t0 = wdata;
    //                 3'd4: sp1_t1 = wdata;
    //                 3'd5: sp1_t2 = wdata;
    //                 3'd6: {f3,f2,f1,f0} = wdata;
    //                 3'd7: {f7,f6,f5,f4} = wdata;
    //             endcase
    //             if (alias_active_4t && (alias_addr_4t == addr_4t)) begin
    //                 alias_active_4t = 1'b0;
    //                 am_4t      = 8'h55;
    //             end
    //         end
    //     end
    // endtask

//read 1t func
function [1:0] read_any_reg_1t;
    input [2:0] addr_1t;
    input       alias_active_1t;
    input [2:0] alias_addr_1t;
    input [1:0] am_buff_1t;
    input [1:0] f0, f1, f2, f3, f4, f5, f6, f7;
    begin
        if (alias_active_1t && (alias_addr_1t == addr_1t)) begin
            read_any_reg_1t = am_buff_1t;
        end else begin
            case (addr_1t)
                3'd0: read_any_reg_1t = f0;
                3'd1: read_any_reg_1t = f1;
                3'd2: read_any_reg_1t = f2;
                3'd3: read_any_reg_1t = f3;
                3'd4: read_any_reg_1t = f4;
                3'd5: read_any_reg_1t = f5;
                3'd6: read_any_reg_1t = f6;
                3'd7: read_any_reg_1t = f7;
            endcase
        end
    end
endfunction

// task write_any_reg_1t;
    //     input use_mask;
    //     input write_allow;
    //     input [2:0] addr_1t;
    //     input [1:0] data_1t;
    //     inout       alias_active_1t;
    //     input [2:0] alias_addr_1t;
    //     inout [1:0] am_1t;
    //     inout [1:0] f0, f1, f2, f3, f4, f5, f6, f7;

    //     reg [1:0] wdata;
    //     begin
    //         if(write_allow)begin
    //             wdata = (!alias_active_1t && use_mask) ? apply_mask_1t(data_1t, am_1t) : data_1t;
    //             case (addr_1t)
    //                     3'd0: f0 = wdata;
    //                     3'd1: f1 = wdata;
    //                     3'd2: f2 = wdata;
    //                     3'd3: f3 = wdata;
    //                     3'd4: f4 = wdata;
    //                     3'd5: f5 = wdata;
    //                     3'd6: f6 = wdata;
    //                     3'd7: f7 = wdata;
    //             endcase

    //             if (alias_active_1t && (alias_addr_1t == addr_1t)) begin
    //                 alias_active_1t = 1'b0;
    //                 am_1t = 2'b01;
    //             end
    //         end
    //     end
    // endtask

task write_any_reg_1t;
    input       use_mask;
    input       w_allow;
    input [2:0] addr_1t;
    input [1:0] data_1t;
    reg   [1:0] wdata;
    begin
        if (w_allow) begin
            wdata = (!alias_active_1t && use_mask) ? apply_mask_1t(data_1t, am_buff_1t) : data_1t;
            case (addr_1t)
                3'd0: spf_flag0 <= wdata;
                3'd1: spf_flag1 <= wdata;
                3'd2: spf_flag2 <= wdata;
                3'd3: spf_flag3 <= wdata;
                3'd4: spf_flag4 <= wdata;
                3'd5: spf_flag5 <= wdata;
                3'd6: spf_flag6 <= wdata;
                3'd7: spf_flag7 <= wdata;
            endcase
            if (alias_active_1t && (alias_addr_1t == addr_1t)) begin
                alias_active_1t <= 1'b0;
                am_buff_1t      <= 2'b01;
            end
        end
    end
endtask

function check_reg_collision;
    input       w_en;
    input [3:0] t_addr;
    input       sel_cmd1;
    input       ovr_en;
    input       ovr_sel;
    input       pipe_w_en;
    input [2:0] pipe_addr;
    reg         t_bank, bin_bank, t_is_cmd, bin_is_cmd, addr_match;
    begin
        t_bank     = ovr_en ? ovr_sel : sel_cmd1;
        bin_bank   = ~sel_cmd1;
        t_is_cmd   = (t_addr >= 4'd9 && t_addr <= 4'd14);
        bin_is_cmd = (pipe_addr >= 3'd1 && pipe_addr <= 3'd6);
        addr_match = ({1'b0, pipe_addr} + 4'd8) == t_addr;

        check_reg_collision = pipe_w_en && w_en && bin_is_cmd && t_is_cmd &&
                              (bin_bank == t_bank) && addr_match;
    end
endfunction

task update_secondary_regs;
    // Thread / ALU write inputs
    input        w_en;
    input [3:0]  t_addr;
    input [23:0] trit_in;
    input        sel_cmd1;
    input        ovr_en;
    input        ovr_sel;
    // Command pipe inputs
    input        pipe_w_en;
    input [2:0]  pipe_addr;
    input [23:0] bin_trit_in;
    // Register states (inout so they update in place)
    inout [23:0] gp6, gp7, gp8;
    inout [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    inout [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    reg          t_bank;
    reg          bin_bank;
    begin
        t_bank   = ovr_en ? ovr_sel : sel_cmd1;
        bin_bank = ~sel_cmd1;

        // 1. Thread / ALU Write
        if (w_en) begin
            case (t_addr)
                4'd6:  gp6 = trit_in;
                4'd7:  gp7 = trit_in;
                4'd8:  gp8 = trit_in;
                4'd9:  if (t_bank) c1_0 = trit_in; else c0_0 = trit_in;
                4'd10: if (t_bank) c1_1 = trit_in; else c0_1 = trit_in;
                4'd11: if (t_bank) c1_2 = trit_in; else c0_2 = trit_in;
                4'd12: if (t_bank) c1_3 = trit_in; else c0_3 = trit_in;
                4'd13: if (t_bank) c1_4 = trit_in; else c0_4 = trit_in;
                4'd14: if (t_bank) c1_5 = trit_in; else c0_5 = trit_in;
            endcase
        end

        // 2. Command Pipe Write (Executed second -> overrides Thread write on collision)
        if (pipe_w_en) begin
            if (bin_bank) begin
                case (pipe_addr)
                    3'd1: c1_0 = bin_trit_in;
                    3'd2: c1_1 = bin_trit_in;
                    3'd3: c1_2 = bin_trit_in;
                    3'd4: c1_3 = bin_trit_in;
                    3'd5: c1_4 = bin_trit_in;
                    3'd6: c1_5 = bin_trit_in;
                endcase
            end else begin
                case (pipe_addr)
                    3'd1: c0_0 = bin_trit_in;
                    3'd2: c0_1 = bin_trit_in;
                    3'd3: c0_2 = bin_trit_in;
                    3'd4: c0_3 = bin_trit_in;
                    3'd5: c0_4 = bin_trit_in;
                    3'd6: c0_5 = bin_trit_in;
                endcase
            end
        end
    end
endtask

function [23:0] read_debug_ext;
    input [4:0]  addr;
    input        active_bank; // Pass using_macroop_1 so ~active_bank reads the OTHER bank
    // Inactive command bank registers
    input [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    input [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    // Alias mask buffers & addresses
    input [23:0] am_12t;
    input [7:0]  am_4t;
    input [1:0]  am_1t;
    input [3:0]  addr_12t;
    input [2:0]  addr_4t;
    input [2:0]  addr_1t;
    input        act_4t;
    input        act_1t;
    // Counters, carry, and macro-op state
    input [7:0]  act_ctr;
    input [7:0]  prm_ctr;
    input [1:0]  carry;
    input [15:0] macroop_reg;
    begin
        case (addr)
            // 5'd15 .. 5'd20: Other Macro-Op's 6 Parameter Registers (~active_bank)
            5'd15: read_debug_ext = (~active_bank) ? c1_0 : c0_0;
            5'd16: read_debug_ext = (~active_bank) ? c1_1 : c0_1;
            5'd17: read_debug_ext = (~active_bank) ? c1_2 : c0_2;
            5'd18: read_debug_ext = (~active_bank) ? c1_3 : c0_3;
            5'd19: read_debug_ext = (~active_bank) ? c1_4 : c0_4;
            5'd20: read_debug_ext = (~active_bank) ? c1_5 : c0_5;

            // 5'd21: 12-trit alias mask buffer (full 24 bits)
            5'd21: read_debug_ext = am_12t;

            // 5'd22: [23:16]=alias_addr_12t | [15:8]=am_buff_4t | [7:0]={2'b0, am_buff_1t, alias_active_4t, alias_addr_4t}
            5'd22: read_debug_ext = {4'b0, addr_12t, am_4t, 2'b0, am_1t, act_4t, addr_4t};

            // 5'd23: [23:16]={2'b0, carry_reg, alias_active_1t, alias_addr_1t} | [15:8]=action_ctr | [7:0]=param_ctr
            5'd23: read_debug_ext = {2'b0, carry, act_1t, addr_1t, act_ctr, prm_ctr};

            // 5'd24: [23:16]=8'b0 | [15:0]=macroop_register
            5'd24: read_debug_ext = {8'b0, macroop_reg};

            default: read_debug_ext = 24'hAAAAAA;
        endcase
    end
endfunction

// Convert 12 Override Ternary trits -> 12 SPI bits (1 for +1 [2'b01], 0 otherwise)
function [11:0] trits12_to_spi12;
    input [23:0] t12;
    integer k;
    begin
        for (k = 0; k < 12; k = k + 1)
            trits12_to_spi12[k] = (t12[2*k +: 2] == 2'b01);
    end
endfunction

task start_bus_tx;
    input [1:0]  tx_mode;
    input [23:0] addr_val;
    input [23:0] data_val;
    begin
        addr_out      <= addr_val;
        d_out         <= data_val;
        execute_stage <= 2'b01; // Wait for data_done in Stage 1
        execute_done  <= 1'b0;

        case (tx_mode)
            TX_SHOW_REG:  request_show_reg      <= 1'b1;
            TX_SET_PC:    request_set_pc        <= 1'b1;
            TX_MEM_READ:  begin
                request_memory_access <= 1'b1;
                is_write_memory       <= 1'b0;
            end
            TX_MEM_WRITE: begin
                request_memory_access <= 1'b1;
                is_write_memory       <= 1'b1;
            end
        endcase
    end
endtask

task on_pc_modified;
    begin
        pc_tx_pending <= 1'b1;
        // Drop the next latched macro-op (keep the currently executing slot)
        if (threadstate == THREAD_FULL) begin
            threadstate   <= using_macroop_1 ? THREAD_EMPTY0 : THREAD_EMPTY1;
            pipe_full_rej <= 1'b0;
        end
        loading_second_slot <= 1'b0;
    end
endtask

task execute_alias_write;
    input active;
    input is_ret_carryover; // Pass ret_micro here!
    reg   alias_bank;
    begin
        alias_bank = is_ret_carryover ? ~using_macroop_1 : using_macroop_1;

        if (active) begin
            // 12t alias commit
            if (alias_addr_12t != 4'd0) begin
                if (write_allow) begin
                    case (alias_addr_12t)
                        4'd1: {sp0_trib2, sp0_trib1, sp0_trib0} <= am_buff_12t;
                        4'd2: {sp1_trib2, sp1_trib1, sp1_trib0} <= am_buff_12t;
                        4'd3: {spf_trib2, spf_flag7, spf_flag6, spf_flag5, spf_flag4,
                                          spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= am_buff_12t;
                        4'd4: {mac_d, mac_b, mac_a} <= am_buff_12t;
                        default: write_secondary_reg(1'b1, alias_addr_12t, alias_bank, am_buff_12t);
                    endcase
                end
                alias_addr_12t <= 4'd0;
                am_buff_12t    <= 24'h555555;
            end

            // 4t alias commit
            if (alias_active_4t) begin
                if (write_allow) begin
                    case (alias_addr_4t)
                        3'd0: sp0_trib0 <= am_buff_4t;
                        3'd1: sp0_trib1 <= am_buff_4t;
                        3'd2: sp0_trib2 <= am_buff_4t;
                        3'd3: sp1_trib0 <= am_buff_4t;
                        3'd4: sp1_trib1 <= am_buff_4t;
                        3'd5: sp1_trib2 <= am_buff_4t;
                        3'd6: {spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= am_buff_4t;
                        3'd7: {spf_flag7, spf_flag6, spf_flag5, spf_flag4} <= am_buff_4t;
                    endcase
                end
                alias_active_4t <= 1'b0;
                am_buff_4t      <= 8'h55;
            end

            // 1t alias commit
            if (alias_active_1t) begin
                if (write_allow) begin
                    case (alias_addr_1t)
                        3'd0: spf_flag0 <= am_buff_1t;
                        3'd1: spf_flag1 <= am_buff_1t;
                        3'd2: spf_flag2 <= am_buff_1t;
                        3'd3: spf_flag3 <= am_buff_1t;
                        3'd4: spf_flag4 <= am_buff_1t;
                        3'd5: spf_flag5 <= am_buff_1t;
                        3'd6: spf_flag6 <= am_buff_1t;
                        3'd7: spf_flag7 <= am_buff_1t;
                    endcase
                end
                alias_active_1t <= 1'b0;
                am_buff_1t      <= 2'b01;
            end
        end

        // Background command-pipe parameter commit (overrides alias on collision)
        if (cmd_we) begin
            write_secondary_reg(1'b1, {1'b0, cmd_param_id} + 4'd8, cmd_target_bank, cmd_param_reg);
            if (hold_reg_active && (cmd_target_bank == using_macroop_1) && (({1'b0, cmd_param_id} + 4'd8) == hold_reg_addr)) begin
                hold_reg_active <= 1'b0;
            end
        end
    end
endtask

always @(posedge clk or negedge rst_n) begin
    //this is the main loop of the system
    if (!rst_n) begin
        //control & state        
        prev_reg_addr  <= 5'd0;
        prev_reg_stage <= 2'b00;
        prev_is_read   <= 1'b0;
        // spi_tx_data    <= 12'd0;
        // spi_tx_start   <= 1'b0;
        // spi_clk_div    <= 4'd4;
        // spi_div_we     <= 1'b0;
        write_allow <= 1'b1;
        threadstate        <= THREAD_EMPTY;
        macroop_register   <= 16'b0;
        using_macroop_1    <= 1'b0;
        op_read_state      <= 2'b00;
        cmd_param_id       <= 3'd1;      // 1-indexed to match commandpipe_address (1..6)
        //cmd_param_reg      <= 24'hAAAAAA;
        ret_micro          <= 1'b0;
        execute_done       <= 1'b0;
        execute_stage      <= 2'b00;
        command_req_new    <= 1'b0;
        // pulses and handshakes
        cmd_buf_clr_ack    <= 1'b0;
        single_execute_ack <= 1'b0;
        pc_inc_enable       <= 1'b0;
        pc_w_enable        <= 1'b0;
        //pc_w_data          <= 24'hAAAAAA;
        cmd_we             <= 1'b0;
        pipe_empty         <= 1'b1;
        pipe_full_rej      <= 1'b0;
        //debug_reg_cache    <= 24'hAAAAAA;
        //dbg_dout           <= 8'hAA;

        //aliases
        action_ctr         <= 8'b0;
        param_ctr          <= 8'b0;
        alias_addr_12t     <= 4'b0;
        alias_addr_4t      <= 3'b0;
        alias_addr_1t      <= 3'b0;
        alias_active_4t    <= 1'b0;
        alias_active_1t    <= 1'b0;
        am_buff_12t        <= 24'h555555;
        am_buff_4t         <= 8'h55;
        am_buff_1t         <= 2'b01;

        //uhhhh
        loading_second_slot <= 1'b0;

        //top reggies
        // {sp0_trib2, sp0_trib1, sp0_trib0} <= 24'hAAAAAA;
        // {sp1_trib2, sp1_trib1, sp1_trib0} <= 24'hAAAAAA;
        // {spf_flag7, spf_flag6, spf_flag5, spf_flag4,
        //  spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= 16'hAAAA;
        // {mac_a,mac_b,mac_c,mac_d} <= 32'hAAAAAAAA;
        mac_carry <= 2'b10;
        // gp_reg_6    <= 24'hAAAAAA; gp_reg_7     <= 24'hAAAAAA; gp_reg_8     <= 24'hAAAAAA;
        // cmd0_reg_0  <= 24'hAAAAAA; cmd0_reg_1   <= 24'hAAAAAA; cmd0_reg_2   <= 24'hAAAAAA;
        // cmd0_reg_3  <= 24'hAAAAAA; cmd0_reg_4   <= 24'hAAAAAA; cmd0_reg_5   <= 24'hAAAAAA;
        // cmd1_reg_0  <= 24'hAAAAAA; cmd1_reg_1   <= 24'hAAAAAA; cmd1_reg_2   <= 24'hAAAAAA;
        // cmd1_reg_3  <= 24'hAAAAAA; cmd1_reg_4   <= 24'hAAAAAA; cmd1_reg_5   <= 24'hAAAAAA;


        // //execution state resets
        // alu_res_4t_lat <= 8'hAA;
        // alu_res_1t_lat <= 2'b10;
        // alu_res_mul_lat <= 16'hAAAA;
        // hold_reg_addr <= 4'b0;
        hold_reg_active <= 1'b0;

        hog_timer             <= 4'd0;
        request_show_reg      <= 1'b0;
        request_set_pc        <= 1'b0;
        request_memory_access <= 1'b0;
        is_write_memory       <= 1'b0;
        ext_addr_out          <= 16'hAAAA; // 8 trits of canonical zero
        direct_exec_active <= 1'b0;
        direct_exec_ready <= 1'b0;
        cmd_target_bank <= 1'b0;
        pc_tx_pending <= 1'b0; alu_command_ready <= 1'b0; request_new_cmd <= 1'b0; flush <= 1'b0; carry_reg <= 2'b10;
    end else begin
        execute_done <= 1'b0;
        can_execute_this_clock = 1'b0; // non blocking temp var
        pc_inc_enable          <= 1'b0;
        pc_w_enable           <= 1'b0;
        cmd_we                <= 1'b0;
        pipe_empty            <= (threadstate == THREAD_EMPTY);
        command_req_new       <= 1'b0;
        alu_prio_inc          <= 1'b0;
        flush <= 1'b0;
        hog_timer <= 4'd0;
        do_housekeeping = 1'b0;

        // spi_div_we     <= 1'b0;
        prev_reg_addr  <= register_address;
        prev_reg_stage <= register_read_stage;
        prev_is_read   <= is_read;
        //this block executes beforehand to do... something    
            if(cmd_buf_clr) begin
                //this wipes the macro ops and puts the bus at empty
                threadstate     <= THREAD_EMPTY;
                macroop_register <= 16'b0;
                using_macroop_1 <= 1'b0;
                //do not the pc and clear action ctr and param ctr
                action_ctr      <= 8'b00000000;
                param_ctr      <= 8'b00000000;
                //direct clear flag
                {spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= 8'hAA;
                {spf_flag7, spf_flag6, spf_flag5, spf_flag4} <= 8'hAA;
                cmd_buf_clr_ack <= 1'b1;
                direct_exec_active <= 1'b0;
                direct_exec_ready <= 1'b0;
                // spi_tx_start <= 1'b0;
                hold_reg_active <= 1'b0;
            end
            else begin
                cmd_buf_clr_ack <= 1'b0; // Ensure ack clears when cmd_buf_clr drops
                can_execute_this_clock = 1'b0;
                if(register_address != 5'd0) begin
                    if(is_read) begin
                        //valid read address
                        // if its 5 its the program counter
                        //anything else goes like normal
                        //triple read stages
                        // Valid debug read address (1 to 14) -> triple read stages
                        if (register_address < 5'd15) begin
                            // Directly read any 12-trit register using the unified function
                            // (Note: PC read enable pc_r_enable should be high when register_address == 5'd5)
                            debug_reg_cache <= read_any_reg_12t(
                                register_address[3:0], using_macroop_1, // or sel_cmd1 (see Note 2)
                                alias_addr_12t,
                                am_buff_12t,
                                sp0_trib0, sp0_trib1, sp0_trib2,
                                sp1_trib0, sp1_trib1, sp1_trib2,
                                spf_trib0, spf_trib1, spf_trib2,
                                mac_a,mac_b,mac_d,
                                pc_r_data, gp_reg_6, gp_reg_7, gp_reg_8,
                                cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
                                cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
                            );
                        end else begin

                            //first read the other macro op's parameters (6 of them)
                            //then read the alias mask buffers
                            //read action and param counters 8bits each
                            //carry reg 2 bits
                            //transmit macro op registers 16 bits
                            debug_reg_cache <= read_debug_ext(
                                register_address, using_macroop_1,
                                cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
                                cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5,
                                am_buff_12t, am_buff_4t, am_buff_1t,
                                alias_addr_12t, alias_addr_4t, alias_addr_1t,
                                alias_active_4t, alias_active_1t,
                                action_ctr, param_ctr, carry_reg, macroop_register
                            );
                        end
                        case (register_read_stage)
                            2'b00: dbg_dout <= debug_reg_cache[7:0];
                            2'b01: dbg_dout <= debug_reg_cache[15:8];
                            2'b10: dbg_dout <= debug_reg_cache[23:16];
                            default: dbg_dout <= 8'hAA;
                        endcase

                        if (register_address == 5'd31) begin
                            if (register_read_stage[0] == 1'b0) begin
                                // Stage 0: Active 8-bit macro-op
                                dbg_dout <= using_macroop_1 ? macroop_register[15:8] : macroop_register[7:0];
                            end else begin
                                // Stage 1: Next queued 8-bit macro-op
                                dbg_dout <= using_macroop_1 ? macroop_register[7:0]  : macroop_register[15:8];
                            end
                        end
                    end else begin
                        //debug read
                        if (prev_reg_addr != 5'd0 && !prev_is_read) begin
                            if (prev_reg_stage == 2'b00 && register_read_stage == 2'b01) begin
                                debug_reg_cache[7:0] <= dbg_din;
                            end else if (prev_reg_stage == 2'b01 && register_read_stage == 2'b10) begin
                                debug_reg_cache[15:8] <= dbg_din;
                            end else if (prev_reg_stage == 2'b10 && register_read_stage == 2'b00) begin
                                if (prev_reg_addr < 5'd15) begin
                                    write_any_reg_12t(1'b0, 1'b1, prev_reg_addr[3:0], using_macroop_1, dbg_wdata_24b);
                                    if (hold_reg_active && (prev_reg_addr[3:0] == hold_reg_addr)) hold_reg_active <= 1'b0;
                                end else if (prev_reg_addr <= 5'd20) begin
                                    write_secondary_reg(1'b1, prev_reg_addr[3:0] - 4'd6, ~using_macroop_1, dbg_wdata_24b);
                                end else begin
                                    case (prev_reg_addr)
                                        5'd21: am_buff_12t <= dbg_wdata_24b;
                                        5'd22: begin
                                            alias_addr_12t  <= dbg_wdata_24b[19:16];
                                            am_buff_4t      <= dbg_wdata_24b[15:8];
                                            am_buff_1t      <= dbg_wdata_24b[5:4];
                                            alias_active_4t <= dbg_wdata_24b[3];
                                            alias_addr_4t   <= dbg_wdata_24b[2:0];
                                        end
                                        5'd23: begin
                                            carry_reg       <= dbg_wdata_24b[21:20];
                                            alias_active_1t <= dbg_wdata_24b[19];
                                            alias_addr_1t   <= dbg_wdata_24b[18:16];
                                            action_ctr      <= dbg_wdata_24b[15:8];
                                            param_ctr       <= dbg_wdata_24b[7:0];
                                        end
                                        5'd24: macroop_register <= dbg_wdata_24b[15:0];
                                        default: ;
                                    endcase
                                end
                            end
                        end
                    end
                end
                else begin
                    //if theres a fence
                    if (prev_reg_addr == 5'd5 && !prev_is_read &&
                        prev_reg_stage == 2'b10 && register_read_stage == 2'b00) begin
                        write_any_reg_12t(1'b0, 1'b1, 4'd5, using_macroop_1, dbg_wdata_24b);
                    end

                    if(microcode_fence) begin
                        //if theres a single execute 
                        //handshake
                        if(single_execute ) begin    
                            if(!single_execute_ack) begin
                                single_execute_ack <= 1'b1;
                                //we can execute normally for 1 micro op
                                can_execute_this_clock = 1'b1;
                                //do something here
                            end
                        end else begin
                            single_execute_ack <= 1'b0;
                        end
                    end else begin
                        //no fence or anything 
                        //act normal
                        can_execute_this_clock = 1'b1;
                    end
                end
            end


        if(can_execute_this_clock) begin //for the core is allowed to do anything at all this needs to be 1
            if(direct_instr) begin
                //this bypasses the usual things, locking it to only operate at command 0
                //this also shuts down the ability for microcode to request more commands, potentially breaking some things
                //thus it should be used with specialised microcode for testing only
                //unfinished?
            end
            
            //logical execution section
                if(ret_micro) begin 
                    ret_micro    <= 1'b0;
                    pc_inc_enable <= 1'b1; // Finishing a command increments the thread PC
                    //resets parameter program counter
                    param_ctr <= 8'b00000000;
                    action_ctr <= 8'b00000000;
                    //yes it is intentional that we can override it later
                    hold_reg_active <= 1'b0;

                    //remind me to move this to a seperate func
                    //housekeeping section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    //best clock for housekeeping btw
                    //make sure that the aliased buffers that write into the command parameter addresses are written into the new command parameter space
                    //excellent for doing carry over stuff, which is intended behaviour
                    //here is a special case where if a param is aliased it should swap it over to the next macro op command param space 
                    do_housekeeping = 1'b1;
                    //end of housekeeping
                end
                else begin
                    //either idling or working

                    pc_inc_enable <= 1'b0;
                    if(threadstate == THREAD_EMPTY && !direct_exec_ready && execute_stage == 2'b00 && !pc_tx_pending && !execute_done) begin
                        //literally idle
                        pipe_empty <= 1'b1;
                        //do housekeeping here
                        do_housekeeping = 1'b1;
                        //twiddle thumbs or something idk
                    end
                    else begin
                        //working hard or hardly working?
                        //housekeeping here writes to the same register set
                        //if microop is in, request another one if it is non branching
                        //for branching, finish executing first then request microcode
                        //if no micro op or no alu, do nothing

                        //check if the logic leading up to micro op handling is sound

                        if (pc_tx_pending && execute_stage == 2'b00) begin
                            pc_tx_pending <= 1'b0;
                            start_bus_tx(TX_SET_PC, 24'hAAAAAA, pc_r_data); // Drives d_out + request_set_pc, waits for data_done in Stage 1
                        end
                        else if(execute_done) begin // execute done refers to the micro instruction
                            //request new from microcode
                            //if there was a jump previously, use a flush and make sure action counter and param counter are visible
                            // then command_req_new is inverted to show new request
                            if (direct_exec_ready) begin
                                direct_exec_ready <= 1'b0; // Consumed direct micro-op; don't poke microcode
                            end else begin
                                command_req_new   <= 1'b1;
                            end
                            execute_done <= 1'b0;
                        end else if(exec_valid && execute_stage == 2'b0) begin

                            if (hold_stall) begin
                                do_housekeeping = 1'b1; // Stall until hold_reg_addr is written
                            end else begin
                            //read command input
                            execute_done <= 1'b0;
                            //for some single clock stuff you can non block it back to 1 later
                            
                            if(exec_cmd == 4'b0000) begin
                                // 0 parameters
                                execute_done <= 1'b1; //auto sets to 1 
                                case(exec_p0)
                                    4'b0001: begin
                                        //mac ladder flush
                                        if(write_allow) begin
                                            {mac_a,mac_b,mac_c,mac_d} <= 32'hAAAAAAAA;
                                            mac_carry <= 2'b10;
                                        end
                                    end
                                    4'b0010: begin
                                        //clear carry
                                        if(write_allow) begin
                                            carry_reg <= 2'b10;
                                        end
                                    end
                                    4'b0011: begin
                                        //reset mask/alias
                                        if(write_allow) begin
                                            am_buff_12t <= 24'h555555;
                                            am_buff_4t <= 8'h55;
                                            am_buff_1t <= 2'b01;
                                            alias_active_1t <= 1'b0;
                                            alias_active_4t <= 1'b0;
                                            alias_addr_12t <= 4'd0;
                                        end
                                    end
                                    4'b0100: begin
                                        //return
                                        ret_micro <= 1'b1;
                                        flush <= 1'b1;
                                        action_ctr <= 8'd0;
                                        param_ctr  <= 8'd0;
                                        //maybe unfinished
                                        //purge all cached microcode on the handler side

                                    end
                                    4'b0101: begin
                                        //disable write
                                        write_allow <= 1'b0;
                                    end
                                    4'b0110: begin
                                        //enable write
                                        write_allow <= 1'b1; //do not clear on a clockly basis 
                                    end
                                    4'b0111: begin
                                        //clear scratchpads
                                        if(write_allow) begin
                                            {sp0_trib2, sp0_trib1, sp0_trib0} <= 24'hAAAAAA;
                                            {sp1_trib2, sp1_trib1, sp1_trib0} <= 24'hAAAAAA;
                                            if (alias_addr_12t == 4'd1 || alias_addr_12t == 4'd2) alias_addr_12t <= 4'd0;
                                            if (alias_active_4t && alias_addr_4t <= 3'd5)         alias_active_4t <= 1'b0;
                                        end
                                    end
                                    4'b1000: begin
                                        //clear general purpose registers
                                        if(write_allow) begin
                                            gp_reg_6 <= 24'hAAAAAA;
                                            gp_reg_7 <= 24'hAAAAAA;
                                            gp_reg_8 <= 24'hAAAAAA;
                                            if (alias_addr_12t >= 4'd6 && alias_addr_12t <= 4'd8) alias_addr_12t <= 4'd0;
                                        end
                                    end
                                    4'b1001: begin

                                        //we need request next command here
                                        //when pulled high, this should prompt to cycle threads 
                                        //stall until next macro command is in
                                        //to prevent a permanent stall, do nothing if direct instr is up
                                        if (!direct_instr && (threadstate != THREAD_FULL)) begin
                                            request_new_cmd <= 1'b1;
                                            execute_stage   <= 2'b01; // Wait in Stage 1
                                            execute_done    <= 1'b0;
                                        end else begin
                                            execute_done    <= 1'b1;  // Already full or direct_instr active -> continue
                                        end
                                    end
                                    4'b1010: begin
                                        //alu priority step up, housekeep
                                        alu_prio_inc    <= 1'b1; // 1-cycle pulse
                                        do_housekeeping  = 1'b1;
                                        //this needs to be implemented on all sides
                                    end
                                    4'b1011: begin
                                        //program counter increment
                                        pc_inc_enable <= 1'b1;
                                        on_pc_modified;
                                        // start_bus_tx(TX_SET_PC, 24'hAAAAAA, pc_r_data);
                                        //blocking or non blocking here?
                                        //remember to always request a pc update when we change it inside here to prevent desync after each macro op and also for situations like these
                                    end
                                    4'b1100: begin
                                        //mac pump (destinationless)
                                        if (write_allow) begin
                                            mac_a <= mac_c;
                                            mac_b <= mac_d;
                                        end
                                    end
                                    4'b1101: begin
                                        //mac add but just takes whatever is in spot c and throws it onto b, clearing c in the process
                                        if (write_allow) begin
                                            mac_b     <= mac_sum;
                                            mac_carry <= mac_cout;
                                            mac_c     <= 8'hAA; // Ternary 0
                                        end 
                                    end

                                    default: ;
                                endcase
                            end else if(exec_cmd == 4'b0001) begin 
                                execute_done <= 1'b1;
                                // 1 parameter
                                case(exec_p0)
                                    4'b0001: begin
                                        //mac read with imm
                                        //due to space limitations, this is constrained to 4 4t scratchpads, which are the 3 in sp1 and flag
                                        /*
                                        task write_any_reg_4t;
                                        input use_mask;
                                        input write_allow;
                                        input [2:0] addr_4t;
                                        inout       alias_active_4t;
                                        input [2:0] alias_addr_4t;
                                        input [7:0] data_4t;

                                        inout [7:0] am_4t;
                                        output [7:0] sp0_t0, sp0_t1, sp0_t2;
                                        output [7:0] sp1_t0, sp1_t1, sp1_t2;
                                        output [7:0] spf_t0, spf_t1;

                                        reg [7:0] wdata;
                                        begin                             


                                        input       use_mask;
                                            input       w_allow;
                                            input [2:0] addr_4t;
                                            input [7:0] data_4t;
                                            reg   [7:0] wdata;
                                            begin
                                        */

                                        case(exec_p1[1:0])
                                            2'b00:write_any_reg_4t(1'b0,write_allow,{1'b0, exec_p1[3:2]} + 2'd3,mac_a);
                                            2'b01:write_any_reg_4t(1'b0,write_allow,{1'b0, exec_p1[3:2]} + 2'd3,mac_b);
                                            2'b10:write_any_reg_4t(1'b0,write_allow,{1'b0, exec_p1[3:2]} + 2'd3,mac_c);
                                            2'b11:write_any_reg_4t(1'b0,write_allow,{1'b0, exec_p1[3:2]} + 2'd3,mac_d);
                                        endcase
                                    end
                                    4'b0010: begin
                                        //mac pump. param is overflow destination
                                        if(write_allow) begin
                                            write_any_reg_4t(exec_p1[0],1'b1,exec_p1[3:1],mac_a);
                                            mac_a <= mac_c;
                                            mac_b <= mac_d;
                                        end
                                    end
                                    4'b0011: begin
                                        //clear 4 trits of scratch/mac
                                        //4 trit chunk address
                                        // Xxx0: clear scratchpads
                                        // Xxx1: clear mac ladder section
                                        if(write_allow) begin
                                            if(exec_p1[0]) begin
                                                case(exec_p1[3:1])
                                                3'b000: mac_a <= 8'hAA;
                                                3'b001: mac_b <= 8'hAA;
                                                3'b010: mac_c <= 8'hAA;
                                                3'b011: mac_d <= 8'hAA;
                                                endcase
                                            end
                                            else begin
                                                write_any_reg_4t(1'b0,1'b1,exec_p1[3:1],8'hAA);
                                                if(alias_active_4t && alias_addr_4t == exec_p1[3:1]) begin
                                                    //clear alias
                                                    alias_active_4t <= 1'b0;
                                                    am_buff_4t <= 8'h55;
                                                end
                                            end
                                        end
                                    end
                                    4'b0100: begin
                                        //hold until register change
                                        //when a micro op with that register being used is reached, it must make sure the register is updated BEFORE it can proceed
                                        //12t register address
                                        hold_reg_addr   <= exec_p1;
                                        hold_reg_active <= 1'b1;
                                    end
                                    4'b0101,4'b0110: begin
                                        if(alias_addr_12t == 4'd0) begin
                                            if(write_allow) begin
                                                am_buff_12t <= shared_rd_12t;
                                                mask_12t_singleuse <= (exec_p0 == 4'b0101);//singleuse
                                            end
                                        end
                                        else begin
                                            do_housekeeping = 1'b1;
                                            execute_stage <= 2'b01;
                                            execute_done <= 1'b0;
                                            //housekeep to clear aliases first
                                        end
                                            //remember for masks if aliasing is active remember they share the same registers so clear the aliases first
                                            //set single use mask for 12 trits
                                            //the value at that 12t address will be copied into the mask buffer
                                            //param 1 is the addresss of the 12 trits that will be copied into the mask
                                    end
                                        // 4'b0110: begin
                                        //     if(write_allow) begin
                                        //         if(alias_addr_12t != 4'd0) begin
                                        //             am_buff_12t <= read_any_reg_12t(
                                        //                 exec_p1, using_macroop_1,
                                        //                 alias_addr_12t,
                                        //                 am_buff_12t,
                                        //                 sp0_trib0, sp0_trib1, sp0_trib2,
                                        //                 sp1_trib0, sp1_trib1, sp1_trib2,
                                        //                 spf_trib0, spf_trib1, spf_trib2,
                                        //                 ladder_trib0, ladder_trib1, ladder_trib2,
                                        //                 pc_r_data, gp_reg_6, gp_reg_7, gp_reg_8,
                                        //                 cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
                                        //                 cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
                                        //             );
                                        //         end
                                        //         else begin
                                        //             //housekeep to clear aliases first
                                        //             //if write enable is off, it should still run through
                                        //         end
                                        //         mask_12t_singleuse <= 1'b0;
                                        //         //mask will not clear automatically for 12 trits
                                        //         //the value at that 12t address will be copied into the mask buffer
                                        //         //param 1 is the addresss of the 12 trits that will be copied into the mask
                                        //     end
                                        // end
                                    4'b0111: begin
                                        //4t mask
                                        if(!alias_active_4t) begin
                                                if(write_allow) begin
                                                am_buff_4t <= shared_rd_4t_a;
                                                //xxx0 : single use
                                                //xxx1 : will not clear
                                                mask_4t_singleuse <= ~exec_p1[0];
                                            end
                                        end else begin
                                            //housekeep
                                            do_housekeeping = 1'b1;
                                            execute_stage <= 2'b01;
                                            execute_done <= 1'b0;
                                        end
                                    end
                                    4'b1000: begin
                                        //1t mask
                                        if(!alias_active_1t) begin
                                            if(write_allow) begin
                                                am_buff_1t <= shared_rd_1t_a;
                                                //xxx0 : single use
                                                //xxx1 : will not clear
                                                mask_1t_singleuse <= ~exec_p1[0];
                                            end
                                        end else begin
                                            //housekeep
                                            do_housekeeping = 1'b1;
                                            execute_stage <= 2'b01;
                                            execute_done <= 1'b0;
                                        end
                                    end
                                    4'b1001: begin
                                        //hop micro op 4 trit address
                                        //xxx0 : action
                                        //xxx1 : param
                                        //xxx is the address of the 4 trit chunk
                                        if(write_allow) begin
                                            if(exec_p1[0] == 1'b0) begin
                                                //xxx0 : action
                                                action_ctr <= shared_rd_4t_a;
                                            end else begin
                                                //xxx1 : param
                                                param_ctr <= shared_rd_4t_a;
                                            end
                                            flush <= 1'b1;
                                        end
                                    end
                                    4'b1010: begin
                                        //read 12 trits of memory to that address (the memory address is overwritten from there)
                                        //this works to provide memory read writes
                                        //this needs to work in conjunction with the memory bus
                                        //need 2 finish
                                        start_bus_tx(TX_MEM_READ, shared_rd_12t, 24'hAAAAAA);
                                    end
                                    4'b1011: begin
                                        //alu hogging
                                        //for big operations that need alu max priority on all processes
                                        //param 1 is a binary number up to 16
                                        hog_timer <= exec_p1;
                                    end
                                    4'b1100: begin
                                        //save to the extended memory address (so you can get up to 9.7GiB of addressable ram)
                                        //this gets sent to the thread manager and saved there
                                        //each thread has their own little memory address
                                        if (write_allow) ext_addr_out <= shared_rd_12t[15:0];
                                    end
                                    4'b1101: begin
                                        //show register data on bus
                                        //param 1 is reg addr
                                        start_bus_tx(TX_SHOW_REG, 24'hAAAAAA, shared_rd_12t);
                                        // d_out <= shared_rd_12t; request_show_reg <= 1'b1; execute_stage <= 2'b01; execute_done <= 1'b0;
                                    end
                                    4'b1110: begin
                                        //use the singular spi to do stuff i guess
                                        //param 1 is reg addr for spi, read as 24 bits
                                        //remember to add ownership stuff. send data to the thread control
                                        //placeholder
                                        execute_done <= 1'b1;
                                        execute_stage <= 2'b00;
                                        // // Latch 12 trits (1 for +1, 0 otherwise) and request SPI shift
                                        // execute_done <= 1'b0;
                                        // if (!spi_busy) begin
                                        //     spi_tx_data   <= trits12_to_spi12(shared_rd_12t);
                                        //     spi_tx_start  <= 1'b1;
                                        //     execute_stage <= 2'b01;
                                        // end
                                    end
                                    4'b1111: begin
                                        //disabled due to lack of pins
                                        // // Set SPI clock division (1..15 of input clock)
                                        // if (write_allow) begin
                                        //     spi_clk_div <= (exec_p1 == 4'd0) ? 4'd1 : exec_p1;
                                        //     spi_div_we  <= 1'b1;
                                        // end
                                    end
                                endcase
                            end 
                            else if(exec_cmd == 4'b0010) begin 
                                // 2 parameters
                                case(exec_p0)
                                    4'b0000,4'b0010,4'b1110: begin
                                        //register A copy to register B (12 trits) without mask
                                        //param1 regA addr
                                        //param2 regB addr
                                        //we are strapped for addresses so this will have to do
                                        tmp_12t_lat   <= shared_rd_12t; //latch source before Stage 1
                                        execute_stage <= 2'b01;
                                        //register A copy to register B (12 trits) with mask
                                        //param1 regA addr
                                        //param2 regB addr
                                    end
                                    //by collapsing all of the commands to deal with them later, we need not deal with them now
                                    4'b0001, 4'b0011, 4'b0100, 4'b1101,4'b1100: begin
                                        if (alias_addr_12t != 4'd0) do_housekeeping = 1'b1;
                                        execute_stage <= 2'b01;
                                    end

                                    /*

                                        4'b0001: begin
                                            //1 trit reg write
                                            //param1 xx00, xx is the immediate value, 01 means normal register space, 10 means that write into carry reg
                                            //param2 xxx0, xxx is the address, and 1 here means the mask is used
                                            execute_stage <= 2'b01;
                                        end
                                        4'b0011: begin
                                            //1 trit reg move
                                            //param 1 (addr from) xxx0 : use mask
                                            //param 1 (addr from) xxx1 : skip mask
                                            //param 2 (addr to)   xxx0
                                            execute_stage <= 2'b01;
                                        end
                                        4'b0100: begin
                                            //4 trit reg move
                                            //param 1 (addr from) xxx0 : use mask
                                            //param 1 (addr from) xxx1 : skip mask
                                            //param 2 (addr to)   xxx0
                                            execute_stage <= 2'b01;
                                        end
                                    */
                                    4'b0101: begin
                                        //due to the shoddy construction of things, you cant swap and mask
                                        //reg swap 12t
                                        //param 1 and 2 : addresses of A and B
                                        if (alias_addr_12t != 4'd0) do_housekeeping = 1'b1;
                                        tmp_12t_lat   <= shared_rd_12t;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b0110: begin
                                        //reg swap 4t
                                        //param 1 and 2 : addresses of A and B
                                        if (alias_active_4t) do_housekeeping = 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b0111: begin
                                        //reg swap 1t
                                        //param 1 and 2 : addresses of A and B
                                        if (alias_active_1t) do_housekeeping = 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b1000: begin
                                        //read extended register (the extended register is basically just ram stored on another fpga)
                                        //param 1 Local reg addr 12T
                                        //param 2 Extended reg addr 12t
                                        start_bus_tx(TX_MEM_READ, {16'hAAAA, exec_p2, 4'hA}, 24'hAAAAAA);
                                    end
                                    4'b1001: begin
                                        //write extended register
                                        //param 1 Local reg addr 12T
                                        //param 2 Extended reg addr 12t
                                        if (write_allow) start_bus_tx(TX_MEM_WRITE, {16'hAAAA, exec_p2, 4'hA}, shared_rd_12t);
                                        else             execute_done <= 1'b1;
                                        //if (write_allow)
                                        //     start_bus_tx(TX_MEM_WRITE, mem_addr_24b, mem_data_24b);
                                        // else
                                        //     execute_done <= 1'b1;
                                    end
                                    4'b1010: begin
                                        //hop parameter counter using immediate
                                        if (write_allow) begin
                                            param_ctr <= {exec_p1, exec_p2};
                                            flush     <= 1'b1;
                                        end
                                        execute_done <= 1'b1;
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
                                    end
                                    4'b1011: begin
                                        //hop action counter using immediate
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
                                        if (write_allow) begin
                                            action_ctr <= {exec_p1, exec_p2};
                                            flush      <= 1'b1;
                                        end
                                        execute_done <= 1'b1;
                                    end
                                    /*
                                    4'b1100: begin
                                        //Generate mask
                                        //param 1
                                        // 0 << error
                                        // 1-3 4 trit mask at positions
                                        // 4-15 1 trit mask

                                        //param 2
                                        //1 trit immediate value and 1 bit is how long it stays, 1 bit is unused
                                    end
                                    4'b1101: begin
                                        //generate value
                                        //param1 : 1 trit fill value, 2 bits to distinguish 00 = 12 trits 01 = 4 trits 10 = 1 trit
                                        //param2 target addresses
                                        //for only 3 bit addresses, the last bit can be anythhing: xxx0 / xxx1 are both ok 
                                    end
                                    4'b1110: begin
                                        //write 12 trits to the next command space
                                        //param 1: ADDR from
                                        //param 2: ADDR to 
                                    end
                                    */
                                    4'b1111: begin
                                        //mac mul without pumping
                                        //OR mac move
                                        alu_command       <= 3'd2;
                                        alu_t4_a      <= shared_rd_4t_a;
                                        alu_t4_b      <= shared_rd_4t_b;
                                        alu_command_ready <= 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    default: execute_done <= 1'b1;
                                endcase
                            end
                            else begin
                                case(exec_cmd)
                                    4'b0011: begin
                                        // trit add/subtract using the alu
                                        /*
                                        Operand A address 4 trit
                                        xxx0: add
                                        xxx1: sub
                                        Operand B address 4 trit
                                        xxx0:regular store
                                        xxx1:invert store

                                        Result store address 4 trit
                                        xxx0: skip mask
                                        xxx1: masked
                                        */
                                        alu_command       <= 3'd1;
                                        alu_t4_a      <= shared_rd_4t_a;
                                        alu_t4_b      <= param_0[0] ? apply_mask_4t(shared_rd_4t_b, 8'h00) : shared_rd_4t_b;
                                        alu_t1_a      <= carry_reg;
                                        alu_command_ready <= 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b0100: begin
                                        //reworked mac
                                        /*
                                        multiply accumulate and pump
                                        0 : dont use mask 
                                        1 : use mask
                                        param 1 Operand A address 4 trit aaa0
                                        param 2 Operand B address 4 trit bbb0
                                        param 3 operand C address 4 trit ccc0
                                        */
                                        alu_command       <= 3'd2;
                                        alu_t4_a      <= (param_0[0] && !alias_active_4t) ? apply_mask_4t(shared_rd_4t_a, am_buff_4t) : shared_rd_4t_a;
                                        alu_t4_b      <= (exec_p1[0] && !alias_active_4t) ? apply_mask_4t(shared_rd_4t_b, am_buff_4t) : shared_rd_4t_b;
                                        alu_command_ready <= 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b0101: begin
                                        //add 3 A + B + C = D
                                        /*
                                        param 1 Operand A address 4 trit aaab
                                        param 2 Operand B address 4 trit bbcc
                                        param 3 operand C address 4 trit cddd
                                        result stored at address D
                                        */
                                        alu_command       <= 3'd6;
                                        alu_t4_a      <= shared_rd_4t_a;
                                        alu_t4_b      <= shared_rd_4t_b;
                                        alu_t4_c      <= shared_rd_4t_c;
                                        alu_t1_a      <= carry_reg;
                                        alu_command_ready <= 1'b1;
                                        execute_stage <= 2'b01;
                                    end
                                    4'b0110: begin
                                        //trit shift
                                        /*
                                        remember to modify alu to support this
                                        address A (original) : xxx , address B (shift amt): yyy (yyy is the 1 trit register address (here the balanced ternary instead is more like 1,2,3))
                                        xxx0yyy0 : local, no mask
                                        xxx0yyy1 : overflow/underflow , depending on shift dir , no mask
                                        xxx1yyy0 : use mask, local
                                        xxx1yyy1 : over/underflow, use mask 
                                        //the mask is applied to the thing before shifting

                                        Result address (4 trits) zzz0 : shift towards infinity, zzz1 : shift towards 0
                                        */
                                        /*
                                        alu_command       <= 3'd7;
                                        alu_t4_a      <= (param_0[0] && !alias_active_4t) ? apply_mask_4t(shared_rd_4t_a, am_buff_4t) : shared_rd_4t_a;
                                        alu_t1_a      <= shared_rd_1t_a;
                                        tritshift_tritwise_command <= {dir, amt[1:0]};
                                        alu_command_ready <= 1'b1;
                                        execute_stage <= 2'b01;
                                        */
                                        alu_command <= 3'd7;
                                        alu_t4_a    <= (exec_p0[0] && !alias_active_4t)
                                                    ? apply_mask_4t(shared_rd_4t_a, am_buff_4t)
                                                    : shared_rd_4t_a;

                                        // Window select: local -> 2'b10 (Center); ovf/udf -> 2'b00 (Left) if dir=inf, 2'b01 (Right) if dir=0
                                        alu_t1_a    <= exec_p1[0] ? {1'b0, exec_p2[0]} : 2'b10;

                                        // {dir (1=left/inf, 0=right/0), amt (2'b00->1, 2'b10/11->2, 2'b01->3)}
                                        tritshift_tritwise_command <= {
                                            ~exec_p2[0],
                                            shared_rd_1t_a[1] ? 2'd2 : (shared_rd_1t_a[0] ? 2'd3 : 2'd1)
                                        };

                                        alu_command_ready <= 1'b1;
                                        execute_stage     <= 2'b01;
                                    end
                                    4'b0111: begin
                                        /*
                                        single instruction conditional
                                        param 0:
                                        xxx is 1 trit logical evaluation address
                                        xxx1: execute if equal
                                        xxx0: execute if unequal
                                        param 1&2 action rom addresses are aaa and param rom is ppp
                                        aaap ppt0
                                        t0 here is an immediate trit, where it checks if the evaluation address equals to this and writes the action and params when equal or unequal
                                        use override encoding 00 : -1 , 01 :  1 , 10|11 : 0
                                        */
                                        if (write_allow) begin
                                            if (param_0[0] == (shared_rd_1t_a == ((exec_p2[1:0] == 2'b11) ? 2'b10 : exec_p2[1:0]))) begin
                                                action_ctr <= {5'b0, exec_p1[3:1]};
                                                param_ctr  <= {5'b0, exec_p1[0], exec_p2[3:2]};
                                                flush      <= 1'b1;
                                            end
                                        end
                                        execute_done <= 1'b1;
                                    end
                                        // 4'b1000: begin
                                        //     //tritwise 1
                                        //     //params 012 , a against b , store value at c
                                        //     /*
                                        //         aaab bbb ccc0 : skip mask
                                        //         aaab bbb ccc1 : use mask
                                        //     */
                                        // end
                                        // 4'b1001: begin
                                        //     //tritwise 2
                                        //     //params 012 , a against b , store value at c
                                        //     /*
                                        //         aaab bbb ccc0 : skip mask
                                        //         aaab bbb ccc1 : use mask
                                        //     */
                                        // end
                                        // 4'b1010: begin
                                        //     //tritwise 3
                                        //     //params 012 , a against b , store value at c
                                        //     /*
                                        //         aaab bbb ccc0 : skip mask
                                        //         aaab bbb ccc1 : use mask
                                        //     */
                                        // end
                                    
                                    4'b1000: begin
                                        case ({param_0[0], exec_p1[0]})
                                            2'b00:   alu_command <= 3'd3; // Tritwise 1
                                            2'b01:   alu_command <= 3'd4; // Tritwise 2
                                            2'b10:   alu_command <= 3'd5; // Tritwise 3
                                            default: alu_command <= 3'd3; // Spare
                                        endcase
                                        tritwise_command  <= {exec_p0[0], exec_p1[0], exec_p2[0]}; // Or {exec_p0[0], exec_p1[0], exec_p2[0]} for 8 sub-ops
                                        alu_t1_a          <= shared_rd_1t_a;
                                        alu_t1_b          <= shared_rd_1t_b;
                                        alu_command_ready <= 1'b1;
                                        execute_stage     <= 2'b01;
                                    end                                       
                                    4'b1011: begin
                                        /*
                                        4 trit imm conversion
                                            param 012
                                            t3t2 t1t0 xxx0 : no mask
                                            t3t2 t1t0 xxx0 : use mask
                                            xxx is store address
                                        */
                                        execute_stage <= 2'b01;
                                    end
                                    4'b1100: begin
                                        /*
                                        12t to 4t transfer or 4t to 1t
                                            param 0: 12t addr
                                            param 1: 4/1t addr (if the lsb is 0, its a 12-4 trit; if the lsb is 1, its a 4-1)
                                            param 2:Which trit to copy (2 bits) + mask result bit + direction bit
                                        */
                                        execute_stage <= 2'b01;
                                    end
                                    //we have 3 left over. maybe one day it can do something
                                    default: execute_done <= 1'b1;
                                endcase
                            end

                            //important! remember to increment based on command

                            //nevermind we removed this
                                // if(command == 4'b0000) begin
                                //     // 0 parameters
                                //     p_pc_inc = 1'b1;
                                //     p_pc_inc_amt = 2'b01;
                                // end else if(command == 4'b0001) begin 
                                //     // 1 parameter
                                //     p_pc_inc = 1'b1;
                                //     p_pc_inc_amt = 2'b10;
                                // end 
                                // else begin
                                //     p_pc_inc = 1'b1;
                                //     p_pc_inc_amt = 2'b11;
                                // end

                            //housekeeping stage
                            /*
                            when the alu needs to write to the registers, 
                            housekeeping should not take place at all in order to avoid collisions
                            in order to achieve this, just don't wipe the alias addresses
                            and turn off the we and re to all registers
                            since the alu handles writing and blocking assignments will prioritise the last one
                            */
                            //for data to be written, check if the command can skip data masking then either mask or write
                            //data masking stage
                            //write stage
                            end
                        end else if(exec_valid) begin
                            //this is for multi stage execution
                            //at most you have 4 stages because reasons (its harder to mess up in 4 cycles)
                            //during this time you can still request microcode
                            //this is for the stall but i havent done the rest of it

                            case (execute_stage)
                                2'b01: begin
                                    //wait until next command comes in
                                    if (exec_cmd == 4'b0000 && exec_p0 == 4'b1001) begin
                                        if (threadstate == THREAD_FULL || direct_instr) begin
                                            request_new_cmd <= 1'b0;
                                            execute_stage   <= 2'b00;
                                            execute_done    <= 1'b1;
                                        end
                                    // end else if (spi_tx_start) begin
                                    //     if (spi_done) begin
                                    //         spi_tx_start  <= 1'b0;
                                    //         execute_stage <= 2'b00;
                                    //         execute_done  <= 1'b1;
                                    //     end
                                    end
                                    else if (request_show_reg || request_set_pc || request_memory_access) begin
                                        //bus wait
                                        if (data_done) begin
                                            if (request_memory_access && !is_write_memory && write_allow) begin
                                                /*
                                                    input use_mask;
                                                    input        write_allow;
                                                    input [3:0]  addr;
                                                    input        sel_cmd1;
                                                    // Alias check inputs
                                                    inout [3:0]  alias_addr_12t;
                                                    inout [23:0] am_12t;
                                                    input [23:0] data_12t;
                                                    output pc_wen;
                                                    output [23:0] pc_val;
                                                    // Scratchpads, Flags (with spf_trib2), Ladder
                                                    output [7:0]  sp0_t0, sp0_t1, sp0_t2;
                                                    output [7:0]  sp1_t0, sp1_t1, sp1_t2;
                                                    output [7:0]  spf_t0, spf_t1, spf_t2;
                                                    
                                                    //ladder
                                                    output [7:0] mac_a, mac_b,mac_d; // Incoming product:   | D | C |
                                                    // PC & Secondary Registers
                                                    output [23:0] gp6, gp7, gp8;
                                                    output [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
                                                    output [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
                                                    
                                                    reg [23:0] wdata;


                                                    task write_any_reg_12t;
                                                        input        use_mask;
                                                        input        w_allow;
                                                        input [3:0]  addr;
                                                        input        sel_c1;
                                                        input [23:0] data_12t;
                                                        reg   [23:0] wdata;
                                                        begin
                                                    */
                                                write_any_reg_12t(1'b0,1'b1,exec_p1,using_macroop_1,d_in);
                                                if (hold_reg_active && (exec_p1 == hold_reg_addr)) begin
                                                    hold_reg_active <= 1'b0;                                                    
                                                end
                                            end
                                            request_show_reg      <= 1'b0;
                                            request_set_pc        <= 1'b0;
                                            request_memory_access <= 1'b0;
                                            is_write_memory       <= 1'b0;
                                            execute_stage         <= 2'b00;
                                            execute_done          <= 1'b1;
                                        end
                                    end 
                                    else if ((exec_cmd == 4'b0010 && exec_p0 == 4'b1111) ||
                                        exec_cmd == 4'b0011 || exec_cmd == 4'b0100 || exec_cmd == 4'b0101 ||
                                        exec_cmd == 4'b0110 || exec_cmd == 4'b1000 || exec_cmd == 4'b1001 || exec_cmd == 4'b1010) begin
                                        //alu op section
                                        if (alu_res_ready) begin
                                            alu_command_ready  <= 1'b0; //release ALU lock
                                            alu_res_4t_lat <= alu_t4_o;
                                            alu_res_1t_lat <= alu_t1_o;
                                            if (exec_cmd == 4'b0010 && exec_p0 == 4'b1111) begin
                                                if (write_allow) begin
                                                    if (exec_p1[3]) begin //mac seed -> B,A
                                                        mac_a <= alu_multiply_o[7:0]; mac_b <= alu_multiply_o[15:8]; mac_carry <= 2'b10;
                                                    end else begin        //mac mul  -> D,C
                                                        mac_c <= alu_multiply_o[7:0]; mac_d <= alu_multiply_o[15:8];
                                                    end
                                                end
                                                execute_stage <= 2'b00;
                                                execute_done  <= 1'b1;
                                            end else if (exec_cmd == 4'b0100) begin
                                                if (write_allow) begin
                                                    mac_c <= alu_multiply_o[7:0];
                                                    mac_d <= alu_multiply_o[15:8];
                                                end
                                                execute_stage <= 2'b10; //mac B+C & pump in stage 2
                                            end else begin
                                                execute_stage <= 2'b10; //masked writeback in stage 2
                                            end
                                        end
                                    end else if (exec_cmd == 4'b0001) begin
                                    //after housekeeping mask stuffs
                                        if (write_allow) begin
                                            case (exec_p0)
                                                4'b0101, 4'b0110: begin
                                                    am_buff_12t        <= shared_rd_12t;
                                                    //mask_12t_active    <= 1'b1;
                                                    mask_12t_singleuse <= (exec_p0 == 4'b0101);
                                                end
                                                4'b0111: begin
                                                    am_buff_4t        <= shared_rd_4t_a;
                                                    //mask_4t_active    <= 1'b1;
                                                    mask_4t_singleuse <= ~exec_p1[0];
                                                end
                                                4'b1000: begin
                                                    am_buff_1t        <= shared_rd_1t_a;
                                                    //mask_1t_active    <= 1'b1;
                                                    mask_1t_singleuse <= ~exec_p1[0];
                                                end
                                                default: ;
                                            endcase
                                        end
                                        execute_stage <= 2'b00;
                                        execute_done  <= 1'b1;

                                    // 1E. 2-Parameter Non-ALU Commits (Copies, Moves, Swaps, Imm, Gen)
                                    end else if (exec_cmd == 4'b0010) begin
                                        if (write_allow) begin
                                            case (exec_p0)
                                                4'b0000,// 12t Copy (no mask)
                                                4'b0010: begin// 12t Copy (masked)

                                                    //i thinked write any reg resolves am

                                                    /*
                                                    task write_any_reg_12t;
                                                        input        use_mask;
                                                        input        w_allow;
                                                        input [3:0]  addr;
                                                        input        sel_c1;
                                                        input [23:0] data_12t;
                                                        reg   [23:0] wdata;
                                                        begin
                                                    */
                                                    write_any_reg_12t(exec_p0[1],1'b1,exec_p2,using_macroop_1,tmp_12t_lat);
                                                    if (exec_p0[1] && mask_12t_singleuse) am_buff_12t <= 24'h555555;
                                                end
                                                4'b0001: begin // 1t Imm Write
                                                    if (exec_p1[1:0] == 2'b10)
                                                        //write to carry reg
                                                        carry_reg <= (exec_p2[0] && !alias_active_1t) ? apply_mask_1t(exec_p1[3:2], am_buff_1t) : exec_p1[3:2];
                                                    else
                                                        /*
                                                        input use_mask;
                                                        input write_allow;
                                                        input [2:0] addr_1t;
                                                        input [1:0] data_1t;
                                                        inout       alias_active_1t;
                                                        input [2:0] alias_addr_1t;
                                                        inout [1:0] am_1t;
                                                        output [1:0] f0, f1, f2, f3, f4, f5, f6, f7;

                                                        reg [1:0] wdata;

                                                        task write_any_reg_1t;
                                                            input       use_mask;
                                                            input       w_allow;
                                                            input [2:0] addr_1t;
                                                            input [1:0] data_1t;
                                                            reg   [1:0] wdata;
                                                        */
                                                        //
                                                        write_any_reg_1t(exec_p2[0],1'b1,exec_p2[3:1], exec_p1[3:2]);
                                                    if (exec_p2[0] && mask_1t_singleuse) am_buff_1t <= 2'b01;//set to * by 1
                                                end
                                                4'b0011: begin // 1t mv (exec_p1[0]=1: use mask, =0: skip mask)
                                                    write_any_reg_1t(exec_p1[0],1'b1,exec_p2[3:1], shared_rd_1t_a);
                                                    if (exec_p1[0] && mask_1t_singleuse) am_buff_1t <= 2'b01;//set to * by 1
                                                end
                                                4'b0100: begin // 4t Move (exec_p1[0] =1: use mask, =0: skip mask)

                                                    //mask,wallow,
                                                    write_any_reg_4t(exec_p1[0],1'b1,exec_p2[3:1],shared_rd_4t_a);
                                                    if (exec_p1[0] && mask_4t_singleuse) am_buff_4t <= 8'h55;//set to * by 1
                                                end
                                                4'b0101: begin // 12t Swap (A=exec_p1 written FIRST, B=exec_p2 aliased)
                                                    am_buff_12t    <= tmp_12t_lat;
                                                    alias_addr_12t <= exec_p2;
                                                    write_any_reg_12t(1'b0,1'b1,exec_p1,using_macroop_1,shared_rd_12t);
                                                    //stage 1 shared_rd_12t reads exec_p2
                                                end
                                                4'b0110: begin // 4t Swap (A=exec_p1[3:1] written FIRST, B=exec_p2[3:1] aliased)
                                                    //yes i know there is no mask, but convention guys
                                                    am_buff_4t      <= shared_rd_4t_a;
                                                    alias_addr_4t   <= exec_p2[3:1];
                                                    alias_active_4t <= 1'b1;
                                                    //due to the shoddy nature of this construct, YES YOU CAN SWAP SOMETHING WITH ITSELF
                                                    //why? idk
                                                    write_any_reg_4t(1'b0,1'b1,exec_p1[3:1],shared_rd_4t_b);
                                                end
                                                4'b0111: begin // 1t Swap (A=exec_p1[2:0] written FIRST, B=exec_p2[2:0] aliased)
                                                    am_buff_1t      <= shared_rd_1t_a;
                                                    alias_addr_1t   <= exec_p2[3:1];
                                                    alias_active_1t <= 1'b1;
                                                    write_any_reg_1t(1'b0,1'b1,exec_p1[3:1],shared_rd_1t_b);
                                                end
                                                4'b1101: begin // Generate Value (fill 12t/4t/1t with exec_p1[3:2])
                                                    case (exec_p1[1:0])
                                                        2'b00: write_any_reg_12t(1'b0,1'b1,exec_p2,using_macroop_1,{12{exec_p1[3:2]}});
                                                        2'b01: write_any_reg_4t(1'b0,1'b1,exec_p2[3:1],{4{exec_p1[3:2]}});
                                                        2'b10: write_any_reg_1t(1'b0,1'b1,exec_p2[3:1],exec_p1[3:2]);
                                                        default: ;
                                                    endcase
                                                end

                                                4'b1100: begin // Generate Mask into am_buff_12t
                                                    // exec_p1: 0=error, 1..3=4t trib fill, 4..15=single trit pos (0..11)
                                                    // exec_p2[3:2]=trit value, exec_p2[1]=0 single-use / 1 persistent
                                                    if (exec_p1 == 4'd0) begin
                                                        reg_ovf_r      <= 1'b1;
                                                        offending_id_r <= exec_cmd;
                                                    end else begin
                                                        am_buff_12t        <= 24'h555555;
                                                        mask_12t_singleuse <= ~exec_p2[1];
                                                        case (exec_p1)
                                                            4'd1: am_buff_12t[7:0]   <= {4{exec_p2[3:2]}};
                                                            4'd2: am_buff_12t[15:8]  <= {4{exec_p2[3:2]}};
                                                            4'd3: am_buff_12t[23:16] <= {4{exec_p2[3:2]}};
                                                            default: am_buff_12t[(exec_p1 - 4'd4)*2 +: 2] <= exec_p2[3:2];
                                                        endcase
                                                    end
                                                end
                                                4'b1110: begin // Write 12t to NEXT command parameter space (~using_macroop_1)
                                                    write_any_reg_12t(1'b0, 1'b1, exec_p2, ~using_macroop_1, tmp_12t_lat);
                                                end
                                                default: ;
                                            endcase
                                        end
                                        execute_stage <= 2'b00;
                                        execute_done  <= 1'b1;
                                    // 1F. 4-Trit imm load (1011) and slice transfer (1100)
                                    end else begin
                                        if (write_allow && exec_cmd == 4'b1011) begin
                                            write_any_reg_4t(exec_p2[0],1'b1,exec_p2[3:1],{exec_p0, exec_p1});
                                            if (exec_p2[0] && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                        end else if (write_allow && exec_cmd == 4'b1100) begin
                                            if (!exec_p1[0]) begin // 12t <-> 4t
                                                if (!exec_p2[0]) begin // Extract 4t from 12t
                                                    write_any_reg_4t(exec_p2[1], 1'b1, exec_p1[3:1],
                                                        (exec_p2[3:2] == 2'd0) ? shared_rd_12t[7:0]  :
                                                        (exec_p2[3:2] == 2'd1) ? shared_rd_12t[15:8] : shared_rd_12t[23:16]);
                                                    if (exec_p2[1] && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                                end else begin         // Insert 4t into 12t
                                                    write_any_reg_12t(exec_p2[1], 1'b1, exec_p0, using_macroop_1,
                                                        (exec_p2[3:2] == 2'd0) ? {shared_rd_12t[23:8],  shared_rd_4t_a} :
                                                        (exec_p2[3:2] == 2'd1) ? {shared_rd_12t[23:16], shared_rd_4t_a, shared_rd_12t[7:0]} :
                                                                                {shared_rd_4t_a, shared_rd_12t[15:0]});
                                                    if (exec_p2[1] && mask_12t_singleuse) am_buff_12t <= 24'h555555;
                                                end
                                            end else begin         // 4t <-> 1t
                                                if (!exec_p2[0]) begin // Extract 1t from 4t
                                                    write_any_reg_1t(exec_p2[1], 1'b1, exec_p1[3:1],
                                                        shared_rd_4t_a[exec_p2[3:2]*2 +: 2]);
                                                    if (exec_p2[1] && mask_1t_singleuse) am_buff_1t <= 2'b01;
                                                end else begin         // Insert 1t into 4t
                                                    write_any_reg_4t(exec_p2[1], 1'b1, exec_p0[3:1],
                                                        (exec_p2[3:2] == 2'd0) ? {shared_rd_4t_a[7:2], shared_rd_1t_a} :
                                                        (exec_p2[3:2] == 2'd1) ? {shared_rd_4t_a[7:4], shared_rd_1t_a, shared_rd_4t_a[1:0]} :
                                                        (exec_p2[3:2] == 2'd2) ? {shared_rd_4t_a[7:6], shared_rd_1t_a, shared_rd_4t_a[3:0]} :
                                                                                {shared_rd_1t_a, shared_rd_4t_a[5:0]});
                                                    if (exec_p2[1] && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                                end
                                            end
                                        end
                                        execute_stage <= 2'b00;
                                        execute_done  <= 1'b1;
                                    end
                                    
                                end

                                2'b10: begin
                                    if (exec_cmd == 4'b0100) begin
                                        // mac: kick A out (masked by exec_p2[0]), A <= B+C, B <= D
                                        if (write_allow) begin
                                            write_any_reg_4t(exec_p2[0],1'b1,exec_p2[3:1],mac_a);
                                            mac_a     <= mac_sum;
                                            mac_b     <= mac_d;
                                            mac_carry <= mac_cout;
                                            if ((exec_p0[0] || exec_p1[0] || exec_p2[0]) && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                        end
                                    end else if (write_allow) begin
                                        case (exec_cmd)
                                            // 0011:Add/Sub — exec_p1[0]=invert store, exec_p2[3:1]=dest, exec_p2[0]=mask
                                            4'b0011: begin
                                                carry_reg <= alu_res_1t_lat;
                                                write_any_reg_4t(exec_p2[0],1'b1,exec_p2[3:1],exec_p1[0] ? apply_mask_4t(alu_res_4t_lat, 8'h00) : alu_res_4t_lat);
                                                if (exec_p2[0] && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                            end

                                            // 0101:Add3 — dest = exec_p2[2:0]
                                            4'b0101: begin
                                                carry_reg <= alu_res_1t_lat;
                                                write_any_reg_4t(1'b0,1'b1,exec_p2[2:0],alu_res_4t_lat);
                                            end

                                            // 0110:Trit Shift — dest = exec_p2[3:1] (mask was pre-applied in Stage 0)
                                            4'b0110: begin
                                                //ts is frying me
                                                write_any_reg_4t(1'b0,1'b1,exec_p2[3:1],alu_res_4t_lat);
                                                if (exec_p0[0] && mask_4t_singleuse) am_buff_4t <= 8'h55;
                                            end

                                            // 1000:tritwise — dest = exec_p2[3:1], mask = exec_p2[0]
                                            4'b1000, 4'b1001, 4'b1010: begin
                                                write_any_reg_1t(exec_p2[0], 1'b1, exec_p2[3:1], alu_res_1t_lat);
                                                if (exec_p2[0] && mask_1t_singleuse) am_buff_1t <= 2'b01;
                                            end
                                            default: ;
                                        endcase
                                    end
                                    execute_stage <= 2'b00;
                                    execute_done  <= 1'b1;
                                end


                                //free space!
                                default: begin
                                    execute_stage <= 2'b00;
                                    execute_done  <= 1'b1;
                                end
                            endcase

                            // if (command == 4'b0000 && param_0 == 4'b1001) begin
                            //     if (threadstate == THREAD_FULL || direct_instr) begin
                            //         request_new_cmd <= 1'b0;
                            //         execute_stage   <= 2'b00;
                            //         execute_done    <= 1'b1;
                            //     end
                            // end
                            // //stage 1
                            // if (request_show_reg || request_set_pc || request_memory_access) begin
                            //     if (data_done) begin
                            //         // If this was a Memory Read (100), commit the 24-bit d_in back to the target 12t reg
                            //         if (request_memory_access && !is_write_memory && write_allow) begin
                            //             write_reg_12t(exec_p1, d_in);
                            //         end

                            //         request_show_reg      <= 1'b0;
                            //         request_set_pc        <= 1'b0;
                            //         request_memory_access <= 1'b0;
                            //         is_write_memory       <= 1'b0;
                            //         execute_stage         <= 2'b00;
                            //         execute_done          <= 1'b1;
                            //     end
                            // end
                        end else begin
                            //Housekeeping
                            do_housekeeping = 1'b1;
                        end
                    end
                end

            // macro op acquisition section

            //command in section; check if direct execution mode has been locked
            if (ret_micro) begin
                ret_micro <= 1'b0;
                // open for business
                if (pipe_full_rej) begin
                    // Queued buffer is ready! flip active buffer and start its microcode immediately.
                    // remember on fpga side it should always keep input is command high and have a macro op ready at all times
                    //normal, shows that it is ready for data!
                    using_macroop_1 <= ~using_macroop_1;
                    pipe_full_rej   <= 1'b0;
                    //the state handles itself here
                    threadstate     <= using_macroop_1 ? THREAD_EMPTY1 : THREAD_EMPTY0;
                    action_ctr      <= 8'h00; // Canonical ternary 0
                    param_ctr       <= 8'h00;
                    flush           <= 1'b1;  // 1-cycle pulse to microcode fetcher
                end else begin
                    // No queued macro-op waiting -> pipe is now completely empty
                    pipe_empty          <= 1'b1;
                    threadstate         <= THREAD_EMPTY;
                    loading_second_slot <= 1'b0;
                end
            end

            /* from module ports
                input   wire stall,
                input   wire [7:0] bus_data_in,
                input   wire data_valid,
                input   wire data_done,
                output  wire request_set_pc,
                output  wire request_memory_access,
                output  wire is_write_memory,
                output  wire [23:0] pc_out,
                output  wire [23:0] addr_out,
                input   wire [23:0] d_in,  //data in as part of a memory read write is also 12 trits 
                output  wire [23:0] d_out, //data out is directly 12 trits
                output  wire pipe_empty,
                output  wire pipe_full_rej,
                output  wire request_new_cmd, //set to high when that one micro op


                //bus comms
                input  wire        bus_is_cmd,      //1 = macro-op byte, 0 = 24-bit param byte
                input  wire [2:0]  bus_param_idx,   //target param register index (from ui_in[7:5])
                input  wire [1:0]  bus_byte_phase,  //incoming byte chunk (00, 01, 10)
                output reg         request_show_reg,//triggers req_type 3'b001 (0001:1101)
                output reg  [15:0] ext_addr_out     // 8-trit (16-bit) high memory address (0001:1100)

                the overlord is the thread manager. can i reuse some ports for the same functionality?            
            */

            // if(ret_micro) begin
                //         if(pipe_full_rej) begin
                //             //normal, shows that it is ready for data!                
                //             if(!data_valid) begin //if there is no data in
                //                 pipe_full_rej = 1'b0;
                //                 opstate = op_wait_cmd;
                //                 //the state handles itself here
                //                 if(using_macroop_1) begin
                //                     threadstate = THREAD_EMPTY1;
                //                 end
                //                 else begin
                //                     threadstate = THREAD_EMPTY0;
                //                 end
                //             end
                //             else begin
                //                 if(bus_is_cmd) begin
                //                     if(using_macroop_1) begin
                //                         //thus macro op 0 shoulde be used at this moment
                //                         //writes into macro_op 1
                //                         macroop_register[15:8] <= bus_data_in;
                //                         threadstate = THREAD_EMPTY1;
                //                     end
                //                     else begin
                //                         macroop_register[7:0] <= bus_data_in;
                //                         threadstate = THREAD_EMPTY0;
                //                     end

                //                     op_read_state = 2'b01;
                //                     //if there is incoming data, then it is treated as the first incoming data
                //                     using_macroop_1 <= ~using_macroop_1; //non blocking to execute at the end of the cycle
                //                 end
                //             end
                //         end
                //         else begin
                //             //the pipe is fully empty
                //             //warn
                //             if(!data_valid) begin //if there is no data in
                //                 pipe_empty = 1'b1;
                //                 //pull pipe empty
                //                 threadstate = THREAD_EMPTY;
                //                 //the state handles itself here
                //             end
                //             else begin //if there is incoming data, then it is treated as the first incoming data
                //                 macroop_register[7:0] <= bus_data_in; //if fully empty then you should of course fill from command 0
                //                 using_macroop_1 <= 1'b0;
                //                 op_read_state <= 2'b01;
                //             end
                //         end
                //         //ret micro is now 0
                // end


            /*
            //do i need these sections below or are they handled in the logic already?
            //these are outside of returning micro because they can keep executing / reading the second command while the device is stil ticking
            if (data_valid) begin
                case (op_read_state)
                    2'b01: begin
                        cmd_param_reg[7:0] <= bus_data_in;
                        op_read_state      <= 2'b10;
                    end
                    2'b10: begin
                        cmd_param_reg[15:8] <= bus_data_in;
                        op_read_state       <= 2'b11;
                    end
                    2'b11: begin
                        cmd_param_reg[23:16] <= bus_data_in; // Captures the 3rd byte!
                        op_read_state        <= 2'b01;
                        cmd_param_id         <= cmd_param_id + 1'b1;
                        cmd_we               <= 1'b1;        // 1-cycle pulse (defaulted to 0 at top)
                    end
                    default: op_read_state <= 2'b00;
                endcase
            end
            //1 full 24 bit param is ready, push into the registers, increment cmd_param_id
            //make sure that this happens all within a clock cycle because the bus data waits for no handshakes

            if(data_done) begin
                
                //make sure it works with whatever is happenigs
                op_read_state <= 2'b00;
                cmd_param_id  <= 3'd1;
                // on the overlord side, make sure that when data done is sent, the bus is still transmitting properly,
                // cmd_param_id is not changed from last clock, data valid is low
                
                //if the pipe was empty, you can start executing command 0
                //if it was not, pull pipe full rej
                //either way something interesting is gonna happen
                //pull cmd write enable to false
                if(threadstate == THREAD_EMPTY) begin
                    //something here i forgot
                    //integrate with microcode module to start streaming micro ops
                    //so first go up to the microcode, display macro op, wait until its ready
                    threadstate <= THREAD_EMPTY1;
                    using_macroop_1 <= 0;
                    param_ctr <= 8'd0;
                    action_ctr <= 8'd0;
                    command_req_new <= 1'b1;
                    flush <= 1'b1;
                end 
                else begin
                    //set to full    
                    threadstate <= THREAD_FULL;
                    pipe_full_rej <= 1'b1;
                end
            end
            */

        end
        else begin
            do_housekeeping = 1'b0;
        end
        //param and command streaming 
        //protocol: ui_in[7:5] counts down 6..1 for params; 0 = safe dock / 0-param
        //when a new command is loaded in, the commander should start presubmitting microcode requests
        //even when it's still receiving the rest of the params
        //as well as check if its 11xxxxxx, indicating that the streamed data should be used as a micro op
        //
        //for direct execution, its loaded into a separate buffer, splitting the data into 11aaaa-- bbbbcccc dddd----
        if (data_valid && !pipe_full_rej) begin
            if (bus_is_cmd) begin
                //8-Bit Macro-Op Byte (1-byte transfer, phase 00 -> 11) ---
                if (bus_data_in[7:6] == 2'b11) begin
                    // Direct Execution Mode: 11aaaa--
                    direct_exec_active <= 1'b1;
                    direct_cmd         <= bus_data_in[5:2];
                end else begin
                    direct_exec_active <= 1'b0;

                    if (threadstate == THREAD_EMPTY && !direct_exec_ready) begin
                        // Pipe empty -> load Slot 0 and pre-fetch microcode immediately
                        macroop_register[7:0] <= bus_data_in;
                        using_macroop_1       <= 1'b0;
                        pipe_empty            <= 1'b0;
                        threadstate           <= THREAD_EMPTY1;
                        loading_second_slot   <= 1'b0;
                        action_ctr            <= 8'h00;
                        param_ctr             <= 8'h00;
                        flush                 <= 1'b1; // Pre-fetch while params stream in
                    end else begin
                        // Load inactive slot (opposite of using_macroop_1)
                        if (using_macroop_1)
                            macroop_register[7:0]  <= bus_data_in;
                        else
                            macroop_register[15:8] <= bus_data_in;

                        if (bus_param_idx == 3'd0) begin
                            // 0-param macro-op in 2nd slot -> pipe is immediately full
                            threadstate         <= THREAD_FULL;
                            pipe_full_rej       <= 1'b1;
                            loading_second_slot <= 1'b0;
                        end else begin
                            loading_second_slot <= 1'b1;
                        end
                    end
                end
            end else begin
                //param
                if (direct_exec_active) begin
                    //split 11aaaa-- | bbbbcccc | dddd----
                    if (bus_byte_phase == 2'b00) begin
                        direct_p0 <= bus_data_in[7:4];
                        direct_p1 <= bus_data_in[3:0];
                    end else if (bus_byte_phase == 2'b01) begin
                        direct_p2         <= bus_data_in[7:4];
                        direct_exec_ready <= 1'b1;
                    end

                end else if (bus_param_idx != 3'd0) begin
                    // Ignore bus_param_idx == 3'd0 (safe dock)
                    case (bus_byte_phase)
                        2'b00: cmd_param_reg[7:0]   <= bus_data_in;
                        2'b01: cmd_param_reg[15:8]  <= bus_data_in;
                        2'b10: begin
                            cmd_param_reg[23:16] <= bus_data_in;
                            cmd_param_id         <= bus_param_idx; // 6 down to 1
                            cmd_we               <= 1'b1;          // 1-cycle strobe (next clk commits)
                            cmd_target_bank <= loading_second_slot ? ~using_macroop_1 : using_macroop_1;
                            // Final parameter (index 1) of the 2nd slot finished -> pipe full
                            if (bus_param_idx == 3'd1 && loading_second_slot) begin
                                threadstate         <= THREAD_FULL;
                                pipe_full_rej       <= 1'b1;
                                loading_second_slot <= 1'b0;
                            end
                        end
                        default: ;
                    endcase
                end
            end
        end

        //housekeeping
        execute_alias_write(do_housekeeping && can_execute_this_clock, ret_micro);
        //housekeep here?
    end
end



endmodule