
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
    input [7:0]  lad_t0, lad_t1, lad_t2;
    // PC & Secondary Registers
    input [23:0] pc_val;
    input [23:0] gp6, gp7, gp8;
    input [23:0] c0_0, c0_1, c0_2, c0_3, c0_4, c0_5;
    input [23:0] c1_0, c1_1, c1_2, c1_3, c1_4, c1_5;
    begin
        if (addr == 4'd0) begin
            read_any_reg_12t = 24'b0;
        end else if (alias_addr_12t == addr) begin
            // Active 12t alias intercept
            read_any_reg_12t = am_buff_12t;
        end else begin
            case (addr)
                4'd1: read_any_reg_12t = {sp0_t2, sp0_t1, sp0_t0};
                4'd2: read_any_reg_12t = {sp1_t2, sp1_t1, sp1_t0};
                4'd3: read_any_reg_12t = {spf_t2, spf_t1, spf_t0};
                4'd4: read_any_reg_12t = {lad_t2, lad_t1, lad_t0};
                default: read_any_reg_12t = read_secondary_reg(
                    1'b1, addr, sel_cmd1, pc_val,
                    gp6, gp7, gp8,
                    c0_0, c0_1, c0_2, c0_3, c0_4, c0_5,
                    c1_0, c1_1, c1_2, c1_3, c1_4, c1_5
                );
            endcase
        end
    end
endfunction

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
            read_secondary_reg = 24'b0;
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
                default: read_secondary_reg = 24'b0;
            endcase
        end
    end
endfunction

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

            default: read_debug_ext = 24'b0;
        endcase
    end
endfunction


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
        
        
        reg pc_w_enable;
        
        reg pc_r_enable;

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

        reg  [7:0] spf_trib2; // cannot be accessed normally. basically just to keep the data there to prevent losing it

        reg [7:0] ladder_trib0;
        reg [7:0] ladder_trib1;
        reg [7:0] ladder_trib2;
        reg [1:0] ladder_offset;

        reg [7:0] param_ctr;
        reg [7:0] action_ctr;

        reg [1:0] carry_reg;


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

task execute_alias_write;
    // Control & Alias State Inputs
    input        write_allow;
    input        active;
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

        if (active) begin
            
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


//fix later
assign pc_r_enable = (register_address == 5'd5) || (r_enable && t_addr_in == 4'd5);

assign collision = check_reg_collision(
    (write_allow && do_housekeeping_w && (alias_addr_12t >= 4'd6)),
    alias_addr_12t, sel_cmd1, 1'b1, ~using_macroop_1,
    command_write_enable, commandpipe_address
);

reg do_housekeeping = 1'b1;

always @(posedge clk or negedge rst_n) begin
    //this is the main loop of the system
    if (!rst_n) begin
        // Control & FSM state
        threadstate        <= THREAD_EMPTY;
        macroop_register   <= 16'b0;
        using_macroop_1    <= 1'b0;
        op_read_state      <= 2'b00;
        cmd_param_id       <= 3'd1;      // 1-indexed to match commandpipe_address (1..6)
        cmd_param_reg      <= 24'b0;
        ret_micro          <= 1'b0;
        execute_done       <= 1'b0;
        execute_stage      <= 2'b00;
        command_req_new    <= 1'b0;

        // Pulses & Handshakes
        cmd_buf_clr_ack    <= 1'b0;
        single_execute_ack <= 1'b0;
        pc_increment       <= 1'b0;
        pc_w_enable        <= 1'b0;
        pc_w_data          <= 24'b0;
        cmd_we             <= 1'b0;
        pipe_empty         <= 1'b1;
        pipe_full_rej      <= 1'b0;
        debug_reg_cache    <= 24'b0;
        dbg_dout           <= 8'b0;

        // Counters & Alias state
        action_ctr         <= 8'b0;
        param_ctr          <= 8'b0;
        alias_addr_12t     <= 4'b0;
        alias_addr_4t      <= 3'b0;
        alias_addr_1t      <= 3'b0;
        alias_active_4t    <= 1'b0;
        alias_active_1t    <= 1'b0;
        am_buff_12t        <= 24'b0;
        am_buff_4t         <= 8'b0;
        am_buff_1t         <= 2'b0;

        // Top-level registers
        {sp0_trib2, sp0_trib1, sp0_trib0} <= 24'b0;
        {sp1_trib2, sp1_trib1, sp1_trib0} <= 24'b0;
        {spf_flag7, spf_flag6, spf_flag5, spf_flag4,
         spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= 16'b0;
        {ladder_trib2, ladder_trib1, ladder_trib0} <= 24'b0;
        ladder_offset      <= 2'b0;
        gp_reg_6 <= 24'b0; gp_reg_7 <= 24'b0; gp_reg_8 <= 24'b0;
        cmd0_reg_0 <= 24'b0; cmd0_reg_1 <= 24'b0; cmd0_reg_2 <= 24'b0;
        cmd0_reg_3 <= 24'b0; cmd0_reg_4 <= 24'b0; cmd0_reg_5 <= 24'b0;
        cmd1_reg_0 <= 24'b0; cmd1_reg_1 <= 24'b0; cmd1_reg_2 <= 24'b0;
        cmd1_reg_3 <= 24'b0; cmd1_reg_4 <= 24'b0; cmd1_reg_5 <= 24'b0;
    end else begin
        
        can_execute_this_clock = 1'b0; // Blocking temp var
        pc_increment          <= 1'b0;
        pc_w_enable           <= 1'b0;
        cmd_we                <= 1'b0;
        pipe_empty            <= (threadstate == THREAD_EMPTY);
        command_req_new       <= 1'b0;
        do_housekeeping = 1'b0;
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
                {spf_flag3, spf_flag2, spf_flag1, spf_flag0} <= 8'b00000000;
                {spf_flag7, spf_flag6, spf_flag5, spf_flag4} <= 8'b00000000;
                cmd_buf_clr_ack <= 1'b1;
            end
            else begin
                cmd_buf_clr_ack <= 1'b0; // Ensure ack clears when cmd_buf_clr drops
                can_execute_this_clock = 1'b0;
                if(register_address != 5'd0) begin
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
                            sp0_trib0, sp0_trib1, sp0_trib2,
                            sp1_trib0, sp1_trib1, sp1_trib2,
                            spf_trib0, spf_trib1, spf_trib2,
                            ladder_trib0, ladder_trib1, ladder_trib2,
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
                        2'b01: dbg_dout <= debug_reg_cache[7:0];
                        2'b10: dbg_dout <= debug_reg_cache[15:8];
                        2'b11: dbg_dout <= debug_reg_cache[23:16];
                        default: dbg_dout <= 8'b0;
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
                    action_counter_ctr <= 8'b00000000;
                    //yes it is intentional that we can override it later


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
                    if(threadstate == THREAD_EMPTY) begin
                        //literally idle
                        pipe_empty = 1'b1;
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
                        if(execute_done) begin // execute done refers to the micro instruction
                            //request new from microcode
                            //if there was a jump previously, use a flush and make sure action counter and param counter are visible
                            // then command_req_new is inverted to show new request
                            command_req_new <= 1'b1;
                        end else if(command_data_sent && execute_stage == 2'b0) begin
                            //read command input
                            execute_done <= 1'b0;
                            //for some single clock stuff you can non block it back to 1 later
                            
                            if(command == 4'b0000) begin
                                // 0 parameters
                                case(param_0)
                                    4'b0001: begin
                                        //mac ladder flush
                                        if(write_allow) begin
                                            ladder_trib0 <= 8'd0;
                                            ladder_trib1 <= 8'd0;
                                            ladder_trib2 <= 8'd0;
                                        end
                                    end
                                    4'b0010: begin
                                        //clear carry
                                        if(write_allow) begin
                                            carry_reg <= 2'b00;
                                        end
                                    end
                                    4'b0011: begin
                                        //reset mask/alias
                                        if(write_allow) begin
                                            am_buff_12t <= 24'd0;
                                            am_buff_4t <= 8'd0;
                                            am_buff_1t <= 2'd0;
                                            alias_active_1t <= 1'b0;
                                            alias_active_4t <= 1'b0;
                                            alias_addr_12t <= 4'd0;
                                        end
                                    end
                                    4'b0100: begin
                                        //return
                                        ret_micro <= 1'b1;
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
                                            sp0_trib0 <= 8'd0;
                                            sp0_trib1 <= 8'd0;
                                            sp0_trib2 <= 8'd0;
                                            sp1_trib0 <= 8'd0;
                                            sp1_trib1 <= 8'd0;
                                            sp1_trib2 <= 8'd0;
                                        end
                                    end
                                    4'b1000: begin
                                        //clear general purpose registers
                                        if(write_allow) begin
                                            gp_reg_6 <= 24'd0;
                                            gp_reg_7 <= 24'd0;
                                            gp_reg_8 <= 24'd0;
                                        end
                                    end
                                    4'b1001: begin
                                        //stall until next macro command is in
                                        if(!direct_instr) begin
                                            //to prevent a permanent stall, do nothing if direct instr is up
                                            if(threadstate != THREAD_FULL) begin
                                                stall = WAIT_PIPEFULL;
                                            end

                                            //unfinished: need calls on the thread manager to ask for more instructions
                                        end
                                    end
                                    4'b1010: begin
                                        //alu priority step up, housekeep
                                        //this needs to be implemented on all sides
                                    end
                                    4'b1011: begin
                                        //program counter increment
                                        pc_inc_enable <= 1'b1;
                                        //blocking or non blocking here?
                                        //remember to always request a pc update when we change it inside here to prevent desync after each macro op and also for situations like these
                                    end
                                    4'b1100: begin
                                        //clear mac offset
                                        //remember this is not affected by we
                                        ladder_offset <= 2'b00;
                                    end
                                    4'b1101: begin
                                        //mac offset++
                                        //mac only goes from 0-2
                                        ladder_offset <= ladder_offset + 1'b1;
                                        if(ladder_offset == 2'b10) begin
                                            ladder_offset <= 2'b00;
                                        end
                                    end
                                endcase
                            end else if(command == 4'b0001) begin 
                                // 1 parameter
                                case(param_0)
                                    4'b0001: begin
                                        //mac ladder set overflow target (12 trits)
                                        // internal complex method of handling the overflow
                                        /*
                                            write section according to the mac offset
                                        */
                                        //param 1 here is the 4 bit 12 trit address
                                    end
                                    4'b0010: begin
                                        //mac ladder pump, 4 trit chunk source address 
                                        // Xxx0: fill normally, using xxx as the address to source 4 trits
                                        // Xxx1: fill with 0s
                                        //mac pumps do not shift the destination register trits, 
                                        //instead a counter keeps track of where each segment should go and overwrite that segment 
                                        //based off of the mac counter
                                        //the mac ladder pump copies trib 0 to overflow address, trib 1 to trib 0, trib 2 to trib 1
                                    end
                                    4'b0011: begin
                                        //clear 4 trits of scratch/mac
                                        //4 trit chunk address
                                        // Xxx0: clear scratchpads
                                        // Xxx1: clear mac ladder section
                                        if(write_allow) begin
                                            if(param_1[0]) begin
                                                case(param_1[3:1])
                                                3'b000: ladder_trib0 <= 8'd0;
                                                3'b001: ladder_trib1 <= 8'd0;
                                                3'b010: ladder_trib2 <= 8'd0;
                                                endcase
                                            end
                                            else begin
                                                case(param_1[3:1])
                                                3'b000: sp0_trib0 <= 8'd0;
                                                3'b001: sp0_trib1 <= 8'd0;
                                                3'b010: sp0_trib2 <= 8'd0;
                                                
                                                3'b011: sp1_trib0 <= 8'd0;
                                                3'b100: sp1_trib1 <= 8'd0;
                                                3'b101: sp1_trib2 <= 8'd0;

                                                3'b110: spf_trib0 <= 8'd0;
                                                3'b111: spf_trib1 <= 8'd0;
                                                endcase
                                            end
                                            
                                        end
                                    end
                                    4'b0100: begin
                                        //hold until register change
                                        //when a micro op with that register being used is reached, it must make sure the register is updated BEFORE it can proceed
                                        //12t register address
                                    end
                                    4'b0101: begin
                                        if(write_allow) begin
                                            if(alias_addr_12t == 4'd0) begin
                                                am_buff_12t <= read_any_reg_12t(
                                                    param_1, using_macroop_1,
                                                    sp0_trib0, sp0_trib1, sp0_trib2,
                                                    sp1_trib0, sp1_trib1, sp1_trib2,
                                                    spf_trib0, spf_trib1, spf_trib2,
                                                    ladder_trib0, ladder_trib1, ladder_trib2,
                                                    pc_r_data, gp_reg_6, gp_reg_7, gp_reg_8,
                                                    cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
                                                    cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
                                                );
                                            end
                                            else begin
                                                //housekeep to clear aliases first
                                            end
                                            mask_12t_singleuse <= 1'b1;
                                            //remember for masks if aliasing is active remember they share the same registers so clear the aliases first
                                            //set single use mask for 12 trits
                                            //the value at that 12t address will be copied into the mask buffer
                                            //param 1 is the addresss of the 12 trits that will be copied into the mask
                                        end
                                    end
                                    4'b0110: begin
                                        //*ignores write enable like a boss* (nevermind)
                                        if(write_allow) begin
                                            if(alias_addr_12t != 4'd0) begin
                                                am_buff_12t <= read_any_reg_12t(
                                                    param_1, using_macroop_1,
                                                    sp0_trib0, sp0_trib1, sp0_trib2,
                                                    sp1_trib0, sp1_trib1, sp1_trib2,
                                                    spf_trib0, spf_trib1, spf_trib2,
                                                    ladder_trib0, ladder_trib1, ladder_trib2,
                                                    pc_r_data, gp_reg_6, gp_reg_7, gp_reg_8,
                                                    cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
                                                    cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
                                                );
                                            end
                                            else begin
                                                //housekeep to clear aliases first
                                                //if write enable is off, it should still run through
                                            end
                                            mask_12t_singleuse <= 1'b0;
                                            //mask will not clear automatically for 12 trits
                                            //the value at that 12t address will be copied into the mask buffer
                                            //param 1 is the addresss of the 12 trits that will be copied into the mask
                                        end
                                    end
                                    4'b0111: begin
                                        //4t mask
                                        if(write_allow) begin
                                            if(!alias_active_4t) begin
                                                am_4t <= read_any_reg_4t(
                                                    param_1[3:1],
                                                    alias_active_4t,
                                                    alias_addr_4t,
                                                    am_buff_4t,
                                                    sp0_trib0, sp0_trib1, sp0_trib2,
                                                    sp1_trib0, sp1_trib1, sp1_trib2,
                                                    spf_trib0, spf_trib1
                                                );
                                                if(param_1[0] == 1'b0) begin
                                                    //xxx0 : single use
                                                    mask_4t_singleuse <= 1'b1;
                                                end else begin
                                                    //xxx1 : will not clear
                                                    mask_4t_singleuse <= 1'b0;
                                                end
                                            end else begin
                                                //housekeep
                                            end
                                        end
                                    end
                                    4'b1000: begin
                                        //1t mask
                                        if(write_allow) begin
                                            if(!alias_active_1t) begin
                                                read_any_reg_1t(
                                                    param_1[3:1],
                                                    alias_active_1t,
                                                    alias_addr_1t,
                                                    am_buff_1t,
                                                    spf_flag0, spf_flag1, spf_flag2, spf_flag3, spf_flag4, spf_flag5, spf_flag6, spf_flag7
                                                );
                                                if(param_1[0] == 1'b0) begin
                                                    //xxx0 : single use
                                                    mask_1t_singleuse <= 1'b1;
                                                end else begin
                                                    //xxx1 : will not clear
                                                    mask_1t_singleuse <= 1'b0;
                                                end
                                            end
                                            else begin
                                                
                                            end
                                        end
                                    end
                                    4'b1001: begin
                                        //hop micro op 4 trit address
                                        //xxx0 : action
                                        //xxx1 : param
                                        //xxx is the address of the 4 trit chunk
                                        if(write_allow) begin
                                            if(param_1[0] == 1'b0) begin
                                                //xxx0 : action
                                                case(param_1[3:1])
                                                    3'b000: action_ctr <= sp0_trib0;
                                                    3'b001: action_ctr <= sp0_trib1;
                                                    3'b010: action_ctr <= sp0_trib2;
                                                    
                                                    3'b011: action_ctr <= sp1_trib0;
                                                    3'b100: action_ctr <= sp1_trib1;
                                                    3'b101: action_ctr <= sp1_trib2;

                                                    3'b110: action_ctr <= spf_trib0;
                                                    3'b111: action_ctr <= spf_trib1;
                                                endcase
                                            end else begin
                                                //xxx1 : param
                                                case(param_1[3:1])
                                                    3'b000: param_ctr <= sp0_trib0;
                                                    3'b001: param_ctr <= sp0_trib1;
                                                    3'b010: param_ctr <= sp0_trib2;
                                                    
                                                    3'b011: param_ctr <= sp1_trib0;
                                                    3'b100: param_ctr <= sp1_trib1;
                                                    3'b101: param_ctr <= sp1_trib2;

                                                    3'b110: param_ctr <= spf_trib0;
                                                    3'b111: param_ctr <= spf_trib1;
                                                endcase
                                            end
                                        end
                                    end
                                    4'b1010: begin
                                        //read 12 trits of memory to that address (the memory address is overwritten from there)
                                        //this works to provide memory read writes
                                        //this needs to work in conjunction with the memory bus
                                        //need 2 finish
                                    end
                                    4'b1011: begin
                                        //alu hogging
                                        //for big operations that need alu max priority on all processes
                                        //param 1 is a binary number up to 16
                                    end
                                    4'b1100: begin
                                        //save to the extended memory address (so you can get up to 263GiB of addressable ram)
                                        //this gets sent to the thread manager and saved there
                                        //each thread has their own little memory address
                                    end
                                    4'b1101: begin
                                        //show register data on bus
                                        //param 1 is reg addr
                                    end
                                    4'b1110: begin
                                        //use the singular spi to do stuff i guess
                                        //param 1 is reg addr for spi, read as 24 bits
                                        //remember to add ownership stuff. send data to the thread control
                                    end
                                endcase
                            end 
                            else if(command == 4'b0010) begin 
                                // 2 parameters
                                case(param_0)
                                    4'b0000: begin
                                        //register A copy to register B (12 trits) without mask
                                        //param1 regA addr
                                        //param2 regB addr
                                        //we are strapped for addresses so this will have to do
                                    end
                                    4'b0001: begin
                                        //1 trit reg write
                                        //param1 xx00, xx is the immediate value, 01 means normal register space, 10 means that write into carry reg
                                        //param2 xxx0, xxx is the address, and 1 here means the mask is used
                                    end
                                    4'b0010: begin
                                        //register A copy to register B (12 trits) with mask
                                        //param1 regA addr
                                        //param2 regB addr
                                    end
                                    4'b0011: begin
                                        //1 trit reg move
                                        //param 1 (addr from) xxx0 : use mask
                                        //param 1 (addr from) xxx1 : skip mask
                                        //param 2 (addr to)   xxx0
                                    end
                                    4'b0100: begin
                                        //4 trit reg move
                                        //param 1 (addr from) xxx0 : use mask
                                        //param 1 (addr from) xxx1 : skip mask
                                        //param 2 (addr to)   xxx0
                                    end
                                    4'b0101: begin
                                        //due to the shoddy construction of things, you cant swap and mask
                                        //reg swap 12t
                                        //param 1 and 2 : addresses of A and B
                                    end
                                    4'b0110: begin
                                        //reg swap 4t
                                        //param 1 and 2 : addresses of A and B
                                    end
                                    4'b0111: begin
                                        //reg swap 1t
                                        //param 1 and 2 : addresses of A and B
                                    end
                                    4'b1000: begin
                                        //read extended register (the extended register is basically just ram stored on another fpga)
                                        //param 1 Local reg addr 12T
                                        //param 2 Extended reg addr 12t
                                    end
                                    4'b1001: begin
                                        //write extended register
                                        //param 1 Local reg addr 12T
                                        //param 2 Extended reg addr 12t
                                    end
                                    4'b1010: begin
                                        //hop parameter counter using immediate
                                        if(write_allow) begin
                                            param_ctr <= {param_1,param_2};
                                        end
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
                                    end
                                    4'b1011: begin
                                        //hop action counter using immediate
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
                                        if(write_allow) begin
                                            action_ctr <= {param_1,param_2};
                                        end
                                    end
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
                                    4'b1111: begin
                                        //read mac ladder,
                                        //param1 : the 4 trit address to write to
                                        //xxx0 : no mask , xxx1: use mask
                                        //param2
                                        //xxx0 : this is a 1t address, use it to determine reading sections
                                        //xx01 : this is an immediate, where 4 means that you read data from the designated overflow 4trits
                                    end
                                endcase
                            end
                            else begin
                                case(command)
                                    4'b0011: begin
                                        // trit add/subtract using the alu
                                        /*
                                        Operand A address 4 trit
                                        xxx0: add
                                        xxx1: sub
                                        Operand B address 4 trit
                                        0xxx:regular store
                                        1xxx:invert store

                                        Result store address 4 trit
                                        0xxx: skip mask
                                        1xxx: masked
                                        */
                                    end
                                    4'b0100: begin
                                        //multiply,accumulate, pump A * B + C
                                        /*
                                        param 1 Operand A address 4 trit aaab
                                        param 2 Operand B address 4 trit bbcc
                                        param 3 operand C address 4 trit cddd
                                        the 3 bits are the address 4 trits of a 4t overflow. set it to 000
                                         (000 is still a valid 4t address but idc)
                                        to *not* pump and store the overflow 
                                        if you really want to store it in 000, too bad!
                                        */
                                    end
                                    4'b0101: begin
                                        //add 3 A + B + C = D
                                        /*
                                        param 1 Operand A address 4 trit aaab
                                        param 2 Operand B address 4 trit bbcc
                                        param 3 operand C address 4 trit cddd
                                        result stored at address D
                                        */
                                    end
                                    4'b0110: begin
                                        //trit shift
                                        /*
                                        remember to modify alu to support this
                                        address A (original) : xxx , address B (shift amt): yyy (yyy is the 1 trit register address (here the balanced ternary instead is more like 1,2,3))
                                        00xxxyyy : local, no mask
                                        01xxxyyy : overflow/underflow , depending on shift dir , no mask
                                        10xxxyyy : use mask, local
                                        11xxxyyy : over/underflow, use mask

                                        Result address (4 trits) zzz0 : shift towards infinity, zzz1 : shift towards 0
                                        */
                                    end
                                    4'b0111: begin
                                        /*
                                        single instruction conditional
                                        param 0:
                                        xxx is 1 trit logical evaluation address
                                        1xxx: execute if equal
                                        0xxx: execute if unequal
                                        param 1&2 action rom addresses are aaa and param rom is ppp
                                        aaap ppt0
                                        t0 here is an immediate trit, where it checks if the evaluation address equals to this and writes the action and params when equal or unequal
                                        use override encoding 00 : -1 , 01 :  1 , 10|11 : 0

                                        */
                                    end
                                    4'b1000: begin
                                        //tritwise 1
                                        //params 012 , a against b , store value at c
                                        /*
                                            aaab bbb ccc0 : skip mask
                                            aaab bbb ccc1 : use mask
                                        */
                                    end
                                    4'b1001: begin
                                        //tritwise 2
                                        //params 012 , a against b , store value at c
                                        /*
                                            aaab bbb ccc0 : skip mask
                                            aaab bbb ccc1 : use mask
                                        */
                                    end
                                    4'b1010: begin
                                        //tritwise 3
                                        //params 012 , a against b , store value at c
                                        /*
                                            aaab bbb ccc0 : skip mask
                                            aaab bbb ccc1 : use mask
                                        */
                                    end
                                    4'b1011: begin
                                        /*
                                        4 trit imm conversion
                                            param 012
                                            t3t2 t1t0 xxx0 : no mask
                                            t3t2 t1t0 xxx0 : use mask
                                            xxx is store address
                                        */
                                    end
                                    4'b1100: begin
                                        /*
                                        12t to 4t transfer or 4t to 1t
                                            param 0: 12t addr
                                            param 1: 4/1t addr (if the lsb is 0, its a 12-4 trit; if the lsb is 1, its a 4-1)
                                            param 2:Which trit to copy (2 bits) + mask result bit + direction bit
                                        */
                                    end
                                    //we have 3 left over. maybe one day it can do something
                                endcase
                            end

                            //important! remember to increment based on command
                            if(command == 4'b0000) begin
                                // 0 parameters
                                p_pc_inc = 1'b1;
                                p_pc_inc_amt = 2'b01;
                            end else if(command == 4'b0001) begin 
                                // 1 parameter
                                p_pc_inc = 1'b1;
                                p_pc_inc_amt = 2'b10;
                            end 
                            else begin
                                p_pc_inc = 1'b1;
                                p_pc_inc_amt = 2'b11;
                            end


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
                            

                        end else if(command_data_sent) begin
                            //this is for multi stage execution
                            //at most you have 4 stages because reasons (its harder to mess up in 4 cycles)
                            //during this time you can still request microcode
                        end else begin
                            //Housekeeping
                            do_housekeeping = 1'b1;
                        end
                    end
                end

            // macro op acquisition section

            //command in section; check if direct execution mode has been locked
            if(ret_micro) begin
                // open for business
                    if(pipe_full_rej) begin
                        //normal, shows that it is ready for data!                
                        if(!data_valid) begin //if there is no data in
                            pipe_full_rej = 1'b0;
                            opstate = op_wait_cmd;
                            //the state handles itself here
                            if(using_macroop_1) begin
                                threadstate = THREAD_EMPTY1;
                            end
                            else begin
                                threadstate = THREAD_EMPTY0;
                            end
                        end
                        else begin 
                            if(using_macroop_1) begin
                                //thus macro op 0 shoulde be used at this moment
                                //writes into macro_op 1
                                macroop_register[15:8] <= bus_data_in;
                                threadstate = THREAD_EMPTY1;
                            end
                            else begin
                                macroop_register[7:0] <= bus_data_in;
                                threadstate = THREAD_EMPTY0;
                            end

                            op_read_state = 2'b01;
                            //if there is incoming data, then it is treated as the first incoming data
                            using_macroop_1 <= ~using_macroop_1; //non blocking to execute at the end of the cycle
                        end
                    end
                    else begin
                        //the pipe is fully empty
                        //warn
                        if(!data_valid) begin //if there is no data in
                            pipe_empty = 1'b1;
                            //pull pipe empty
                            threadstate = THREAD_EMPTY;
                            //the state handles itself here
                        end
                        else begin //if there is incoming data, then it is treated as the first incoming data
                            macroop_register[7:0] <= bus_data_in; //if fully empty then you should of course fill from command 0
                            using_macroop_1 <= 1'b0;
                            op_read_state = 2'b01;
                        end
                    end
                    //ret micro is now 0
            end

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
                end 
                else begin
                    //set to full    
                    threadstate <= THREAD_FULL;
                    pipe_full_rej <= 1'b1;
                end

                //when a new command is loaded in, the commander should start presubmitting microcode requests
                //even when it's still receiving the rest of the params
                //as well as check if its 11xxxxxx, indicating that the streamed data should be used as a micro op
                //
                //for direct execution, its loaded into a separate buffer, splitting the data into 11aaaa-- bbbbcccc dddd----

            end

        end
        else begin
            do_housekeeping = 1'b0;
        end

        execute_alias_write(
            write_allow, do_housekeeping && can_execute_this_clock, using_macroop_1, sel_cmd1,
            alias_addr_12t, am_buff_12t,
            alias_active_4t, alias_addr_4t, am_buff_4t,
            alias_active_1t, alias_addr_1t, am_buff_1t,
            command_write_enable, commandpipe_address, binary_trit_in,
            // Next-state alias flags
            alias_addr_12t, alias_active_4t, alias_active_1t,
            // PC write port
            pc_w_enable, pc_w_data,
            // Scratchpad 0/1, Flag, Ladder ports
            sp0_we, sp0_full12t, sp0_pos, sp0_reg_bin_b2t,
            sp1_we, sp1_full12t, sp1_pos, sp1_reg_bin_b2t,
            flag_we, flag_full12t, flag_pos, flag_reg_bin_b2t, flag_reg_bin_b2t_1t,
            ladder_we, ladder_full12t, ladd_reg_bin_b2t,
            // Secondary registers
            gp_reg_6, gp_reg_7, gp_reg_8,
            cmd0_reg_0, cmd0_reg_1, cmd0_reg_2, cmd0_reg_3, cmd0_reg_4, cmd0_reg_5,
            cmd1_reg_0, cmd1_reg_1, cmd1_reg_2, cmd1_reg_3, cmd1_reg_4, cmd1_reg_5
        );
        //housekeep here?
    end
end



endmodule