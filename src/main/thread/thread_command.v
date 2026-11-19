
module thread_command(
    input wire clk,
    input wire rst_n,
    //from the microcode parsers
        output wire [7:0] macro_op,
        output wire [7:0] action_counter,
        output wire [7:0] param_counter,
        output wire flush,
        input wire [3:0] command,
        input wire [3:0] param_0,
        input wire [3:0] param_1,
        input wire [3:0] param_2,
        output reg  command_req_new, // remember this is a strobe!
        input  wire command_data_sent,

        // input [6:0] microcode_action_offset_t0, //up to 128 but since its 4 trit to convert to this it's more like 81
        //input [6:0] microcode_param_offset_t0,
        //.flush(),//this forces the microcode handler to pause, flush cache and read from action position & param position, as well as macro op position
        // output wire ,


    //alu interface
        output  wire [2:0] alu_command,
        output  wire [2:0] tritwise_command,
        output  wire [2:0] tritshift_tritwise_command, //trit shifts up to 3 front and back
        output  wire cmd_ready,
        input   wire res_ready_in,
        //4 trit. replace these with binary coding
        inout [3:0] t4_a_out,
        inout [3:0] t4_b_out,
        inout [3:0] t4_c_out,
        inout       t1_a_out,
        inout       t1_b_out,

        //results here
        inout [3:0] t4_in,
        inout       t1_in,
        inout [7:0] multiply_in,

    //overlord interface
        input   wire stall,
        input   wire [7:0] bus_data_in,
        input   wire data_valid,
        input   wire data_done,
        output  wire request_set_pc,
        output  wire request_memory_access,
        output  wire is_write_memory,
        output  wire [23:0] pc_out,
        output  wire [23:0] addr_out,
        input   wire [23:0] d_in,
        output  wire [23:0] d_out,
        output  wire pipe_empty,
        output  wire pipe_full_rej,
        output  wire request_new_cmd,



    //debug core components
    //when profiling core loads info, it comes from d_in and leaves using d_out
        input   wire [1:0] register_read_stage,       // Byte chunk (0>1>2)
        input   wire [4:0] register_address,          // Flat 5-bit address (0..31) incremented per handover 
        //if reg address is not zero, read is active
        input   wire is_read,                   // 1 = ASIC transmits to FPGA, 0 = ASIC receives from FPGA
        output  wire reg_handover_done,         // Pulled high by active thread when entire register dump finishes
        output  wire [7:0] dbg_dout,
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
        output wire [3:0] offending_id
);



//macro op governing section and state machine
        reg [15:0] macroop_register;
        reg using_macroop_1;//remember to only flip after macro op 1 finishes

        // Expose active macro-op to the microcode parser

        assign macro_op = using_macroop_1 && !direct_instr ? macroop_register[15:8] : macroop_register[7:0];
        assign sel_cmd1 = using_macroop_1;

        // Request a new command from Overlord whenever pipeline isn't full and not currently streaming
        assign request_new_cmd = !pipe_full_rej && (op_read_state == 2'b00);        
        // Thread buffer occupancy states
        localparam  THREAD_EMPTY  = 2'b00, // Neither slot has a valid command
                    THREAD_FULL   = 2'b01, // Both slots occupied
                    THREAD_EMPTY0 = 2'b10, // Slot 0 empty, Slot 1 active
                    THREAD_EMPTY1 = 2'b11; // Slot 1 empty, Slot 0 active
        reg [1:0]   threadstate = THREAD_EMPTY;

//register section

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
        wire        pc_inc_enable;


        wire [23:0] pc_r_data;
        
        // PC Write Enable: High only when w_enable is high AND the target address is 5
        wire pc_w_enable = w_enable && (t_addr_in == 4'd5);
        // PC Read Enable: High only when r_enable is high AND the target address is 5
        wire pc_r_enable = r_enable && (t_addr_in == 4'd5);

        ternary_pc_12t program_counter (
        .clk(clk),
        .rst_n(rst_n),
        .inc_enable(pc_inc_enable),
        .w_enable(pc_w_enable),    // High to overwrite PC with w_data (Jump/Branch)
        .r_enable(pc_r_enable),    // High to drive PC onto r_data bus
        .w_data(trit_in),      // 12-trit (24-bit) branch target input
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


        reg [7:0] ladder_trib0;
        reg [7:0] ladder_trib1;
        reg [7:0] ladder_trib2;
        reg [1:0] ladder_offset;

        reg [7:0] param_ctr;
        reg [7:0] action_ctr;

//12 trit read
function [23:0] read_any_reg_12t;
    input [3:0]  addr;
    input        sel_cmd1;
    // Scratchpad 0 & 1 Tribbles
    input [7:0]  sp0_t0, sp0_t1, sp0_t2;
    input [7:0]  sp1_t0, sp1_t1, sp1_t2;
    // Flags (8 x 2-bit)
    input [1:0]  f0, f1, f2, f3, f4, f5, f6, f7;
    // Ladder Tribbles
    input [7:0]  lad_t0, lad_t1, lad_t2;
    // PC & Secondary Registers (5-14)
    input [23:0] pc_val;
    input [23:0] gp6, gp7, gp8;
    input [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    input [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    begin
        case (addr)
            4'd0: read_any_reg_12t = 24'b0;
            4'd1: read_any_reg_12t = {sp0_t2, sp0_t1, sp0_t0};
            4'd2: read_any_reg_12t = {sp1_t2, sp1_t1, sp1_t0};
            4'd3: read_any_reg_12t = {8'b0, f7, f6, f5, f4, f3, f2, f1, f0};
            4'd4: read_any_reg_12t = {lad_t2, lad_t1, lad_t0};
            default: begin
                read_any_reg_12t = read_secondary_reg(
                    1'b1, addr, sel_cmd1, pc_val,
                    gp6, gp7, gp8,
                    c0_0, c0_1, c0_2, c0_3, c0_4, c0_5,
                    c1_0, c1_1, c1_2, c1_3, c1_4, c1_5
                );
            end
        endcase
    end
endfunction

//4t read func
function [7:0] read_any_reg_4t;
    input [2:0] addr_4t;
    input [7:0] sp0_t0, sp0_t1, sp0_t2;
    input [7:0] sp1_t0, sp1_t1, sp1_t2;
    input [7:0] spf_t0, spf_t1; //packed flag wires
    begin
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
endfunction

//read 1t func
function [1:0] read_any_reg_1t;
    input [2:0] addr_1t;
    input [1:0] f0, f1, f2, f3, f4, f5, f6, f7;
    begin
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
endfunction


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

    //micro operations
        reg ret_micro;
        reg [23:0] cmd_param_reg;
        reg [2:0]  cmd_param_id;
        reg        cmd_we;
        reg [1:0]  op_read_state; // 00: idle, 01: byte 0, 10: byte 1, 11: byte 2 & write

task execute_alias_write;
    // Control & Alias State Inputs
    input        write_allow;
    input        using_macroop_1;
    input        sel_cmd1;
    // 12-trit Alias Inputs
    input [3:0]  addr_12t_in;
    input [23:0] buff_12t;
    // 4-trit Alias Inputs
    input        active_4t_in;
    input [2:0]  addr_4t;
    input [7:0]  buff_4t;
    // 1-trit Alias Inputs
    input        active_1t_in;
    input [2:0]  addr_1t;
    input [1:0]  buff_1t;
    // Command Pipe Background Inputs
    input        pipe_w_en;
    input [2:0]  pipe_addr;
    input [23:0] bin_trit_in;

    //Next-state alias flags & PC write strobe
    output reg [3:0]  addr_12t_next;
    output reg        active_4t_next;
    output reg        active_1t_next;
    output reg        pc_w_en;
    output reg [23:0] pc_w_data;

    //Direct Top-Level Registers
    inout [7:0]  sp0_t0, sp0_t1, sp0_t2;
    inout [7:0]  sp1_t0, sp1_t1, sp1_t2;
    inout [1:0]  f0, f1, f2, f3, f4, f5, f6, f7;
    inout [7:0]  lad_t0, lad_t1, lad_t2;
    inout [23:0] gp6, gp7, gp8;
    inout [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    inout [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;

    // Internal control signals for secondary registers
    reg sec_w_en;
    reg ovr_en;
    reg ovr_sel;

    begin
        // Hold current alias flags by default; clear single-cycle strobes
        addr_12t_next  = addr_12t_in;
        active_4t_next = active_4t_in;
        active_1t_next = active_1t_in;

        pc_w_en        = 1'b0;
        pc_w_data      = buff_12t;
        sec_w_en       = 1'b0;
        ovr_en         = 1'b0;
        ovr_sel        = 1'b0;

        if (write_allow) begin
            
            if (addr_12t_in != 4'b0000) begin
                if (addr_12t_in >= 4'd5) begin
                    if (addr_12t_in == 4'd5) begin
                        pc_w_en   = 1'b1;
                        pc_w_data = buff_12t;
                    end else begin
                        sec_w_en = 1'b1;
                        ovr_en   = 1'b1;
                        ovr_sel  = ~using_macroop_1;
                    end
                end else begin
                    case (addr_12t_in)
                        4'd1: {sp0_t2, sp0_t1, sp0_t0} = buff_12t;
                        4'd2: {sp1_t2, sp1_t1, sp1_t0} = buff_12t;
                        4'd3: {f7, f6, f5, f4, f3, f2, f1, f0} = buff_12t[15:0];
                        4'd4: {lad_t2, lad_t1, lad_t0} = buff_12t;
                    endcase
                end
                addr_12t_next = 4'b0000;
            end

            if (active_4t_in) begin
                active_4t_next = 1'b0;
                case (addr_4t)
                    3'd0: sp0_t0 = buff_4t;
                    3'd1: sp0_t1 = buff_4t;
                    3'd2: sp0_t2 = buff_4t;
                    3'd3: sp1_t0 = buff_4t;
                    3'd4: sp1_t1 = buff_4t;
                    3'd5: sp1_t2 = buff_4t;
                    3'd6: {f3, f2, f1, f0} = buff_4t; // spf_trib0
                    3'd7: {f7, f6, f5, f4} = buff_4t; // spf_trib1
                endcase
            end

            if (active_1t_in) begin
                active_1t_next = 1'b0;
                case (addr_1t)
                    3'd0: f0 = buff_1t;
                    3'd1: f1 = buff_1t;
                    3'd2: f2 = buff_1t;
                    3'd3: f3 = buff_1t;
                    3'd4: f4 = buff_1t;
                    3'd5: f5 = buff_1t;
                    3'd6: f6 = buff_1t;
                    3'd7: f7 = buff_1t;
                endcase
            end
        end

        update_secondary_regs(
            sec_w_en, addr_12t_in, buff_12t, sel_cmd1, ovr_en, ovr_sel,
            pipe_w_en, pipe_addr, bin_trit_in,
            gp6, gp7, gp8,
            c0_0, c0_1, c0_2, c0_3, c0_4, c0_5,
            c1_0, c1_1, c1_2, c1_3, c1_4, c1_5
        );
    end
endtask

reg can_execute_this_clock = 1'b1;

always @(posedge clk or negedge rst_n) begin
    //this is the main loop of the system
    if(!rst_n) begin
    end else begin
        
        //this block executes beforehand to do... something    
            if(cmd_buf_clr) begin
                //this wipes the macro ops and puts the bus at empty
                threadstate = THREAD_EMPTY;
                macroop_register <= 16'b0;
                using_macroop_1 <= 1'b0;
                //this does not touch existing registers to allow us to do weird stuff
                p_pc_we = 1'b1;
                p_pc_wdata = 8'b00000000;
                flag_we = 1'b1;
                flag_pos = 4'b0011;
                flag_reg_bin_b2t_4t = 8'b00000000;
                cmd_buf_clr_ack <= 1'b1;
            end
            else begin
                if(register_address != 5'd0) begin
                    //valid read address
                    // if its 5 its the program counter
                    //anything else goes like normal
                    //triple read stages
                    if(register_address < 5'd15) begin
                        debug_reg_cache = reg_data_from;
                        reg_r_en = 1'b1;
                        reg_addr = 5'd5;
                    end else begin
                        //read native and microcode actions and stuff
                        //unfinished!
                    end
                    case(register_read_stage) 
                        2'b01: dbg_dout = debug_reg_cache[7:0];
                        2'b10: dbg_dout = debug_reg_cache[15:8];
                        2'b11: dbg_dout = debug_reg_cache[23:16];
                    endcase
                end
                else begin
                    //if theres a fence
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
            
    end
end



endmodule