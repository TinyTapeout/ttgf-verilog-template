//the primary commander for the thread responsible for decoding operations 
/*
todo: add error and overflow warns outwards
finish this section for me because i am running out of time
when in doubt prioritise safety over efficiency
mark all changes and annotate them, explaining each section of the code and interpretation of previously laid out code
ask for clarification whenever things are poorly defined
warn me of ambiguity, state and explain how the code was interpreted and also modify any and all accompanying modules
warn me of micro ops that may be operating over multiple clock cycles
*/
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

    // Secondary register internal signals
        wire [23:0] reg_data_from;
        reg  [23:0] reg_data_to;
        reg         reg_r_en;
        reg         reg_w_en;
        reg  [3:0]  reg_addr;
        reg         override_cmd_sel_en;
        reg         override_cmd_sel;
        wire        reg_collision;

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


// Microcode PC control signals
reg        p_pc_inc, p_pc_we, p_pc_re;
reg  [7:0] p_pc_wdata;
wire [7:0] p_pc_rdata;
assign param_counter  = p_pc_rdata;
reg [1:0] p_pc_inc_amt;

// Native Scratchpad & Flag control signals
reg        sp0_re, sp0_we, sp0_full12t;
reg  [1:0] sp0_pos;
reg        sp1_re, sp1_we, sp1_full12t;
reg  [1:0] sp1_pos;
reg        flag_re, flag_we, flag_full12t;
reg  [3:0] flag_pos;
reg        ladder_re, ladder_we, ladder_full12t;
reg  [1:0] ladder_pos;

// Native Tri-State / Analog Bus interconnects
wire [11:0] sp0_din_12T, sp0_dout_12T;
wire [3:0]  sp0_din_4T,  sp0_dout_4T;
wire [11:0] sp1_din_12T, sp1_dout_12T;
wire [3:0]  sp1_din_4T,  sp1_dout_4T;
wire [11:0] flag_din_12T, flag_dout_12T;
wire [3:0]  flag_din_4T,  flag_dout_4T;
wire        flag_din_1T,  flag_dout_1T;


wire [11:0] laddre_din_12T, ladder_dout_12T;
wire [7:0]  ladder_din_8T;
wire [3:0]  ladder_din_4T,  ladder_dout_4T;


//carry bit stuff
wire c_trit_we, c_trit_re,c_trit_din,c_trit_dout;




reg execute_done;
reg [1:0] execute_stage;


reg can_execute_this_clock = 1'b1;



//todo: remove all conversion stuff because its binary at the alu, replace native scratchpads with normal registers


reg [23:0] debug_reg_cache;



//operation parameters and states for the main loop
reg write_allow = 1'b1; //need to finish later

reg single_m_12t = 1'b0;
reg single_m_4t = 1'b0;
reg single_m_1t = 1'b0;
reg [1:0] stallReason = 2'b0;
localparam  NOTSTALL        = 2'b00,
            WAIT_PIPEFULL   = 2'b01,
            WAIT_REGUPDAT   = 2'b10;





always @(posedge clk or negedge rst_n) begin
    //this is the main loop of the system
    if(!rst_n) begin
        // Clear Alias-Mask buffers
        am_buff_12t         <= 24'd0;
        am_buff_4t          <= 8'd0;
        am_buff_1t          <= 2'd0;
        alias_addr_12t      <= 4'b0000;
        alias_addr_4t       <= 3'b000;
        alias_addr_1t       <= 3'b000;
        alias_active_1t     <= 1'b0;
        alias_active_4t     <= 1'b0;
        carry_flag_reg      <= 1'b0;

        // Clear pipeline & stream states
        macroop_register    <= 16'd0;
        using_macroop_1     <= 1'b0;
        threadstate         <= THREAD_EMPTY;
        opstate             <= OP_WAIT_CMD;
        ret_micro           <= 1'b0;
        cmd_param_reg       <= 24'd0;
        cmd_param_id        <= 3'd0;
        cmd_we              <= 1'b0;
        op_read_state       <= 2'b00;
        pipe_empty          <= 1'b1;
        pipe_full_rej       <= 1'b0;
        request_set_pc      <= 1'b0;

        // Clear register & microcode control lines
        reg_data_to         <= 24'd0;
        reg_r_en            <= 1'b0;
        reg_w_en            <= 1'b0;
        reg_addr            <= 4'd0;
        pc_increment        <= 1'b0;
        override_cmd_sel_en <= 1'b0;
        override_cmd_sel    <= 1'b0;
        flush               <= 1'b0;
        command_req_new     <= 1'b0;

        // Clear ALU & sub-module controls
        alu_command                <= 3'd0;
        tritwise_command           <= 3'd0;
        tritshift_tritwise_command <= 3'd0;
        cmd_ready                  <= 1'b0;
        p_pc_inc <= 1'b0; p_pc_we <= 1'b0; p_pc_re <= 1'b1; p_pc_wdata <= 8'd0;
        sp0_re <= 1'b0; sp0_we <= 1'b0; sp0_full12t <= 1'b0; sp0_pos <= 2'b00;
        sp1_re <= 1'b0; sp1_we <= 1'b0; sp1_full12t <= 1'b0; sp1_pos <= 2'b00;
        flag_re <= 1'b0; flag_we <= 1'b0; flag_pos <= 4'b0000;
        offending_id <= 4'd0;
    end 
    else begin
        //reset single clock stuff
            reg_r_en            <= 1'b0;
            reg_w_en            <= 1'b0;
            override_cmd_sel_en <= 1'b0;
            pc_increment        <= 1'b0;
            cmd_we              <= 1'b0;
            flush               <= 1'b0;
            command_req_new     <= 1'b0;
            cmd_ready           <= 1'b0;
            p_pc_inc            <= 1'b0;
            p_pc_we             <= 1'b0;
            a_pc_inc            <= 1'b0;
            a_pc_we             <= 1'b0;
            sp0_re              <= 1'b0;
            sp0_we              <= 1'b0;
            sp1_re              <= 1'b0;
            sp1_we              <= 1'b0;
            flag_re             <= 1'b0;
            flag_we             <= 1'b0;
            can_execute_this_clock <= 1'b0;
            ladder_full12t       <= 1'b0;


            /*
            load new commands, while the rest of the thread either does housekeeping or something else
            when it is valid 
            */

        //the main structure
        /*
        1. check if theres anything like register read stage / fences in place <- these pause normal execution
        2. then check if we are in direct instruction mode
        
        for the main loop, check if the operation had been finished last clock (yes i know this is inefficient) and pull request

        multiple parallel stuff
        A: command stream in
        B: alu interface
        C: microcode reading 

        //as long as the alu/reading microcode isnt active you should do housekeeping
        //the key part is that you don't need to be using the alu to be busy. 
        //some logical evaluations happen without the need of an alu so housekeeping shouldn't happen then too
        //the idle command is for housekeeping
        //theres a full reset command that forces all scratchpads, alias-mask and carry to clear
        //when reading from / writing to the same registers currently being written to from overlord, it prioritise overlord always
        // there are

        */

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
                    //
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


        if(can_execute_this_clock) begin //for the core is allowed to do anything at all this needs to be 1
            if(direct_instr) begin
                //this bypasses the usual things, locking it to only operate at command 1
                //this also shuts down the ability for microcode to request more commands, potentially breaking some things
                //thus it should be used with specialised microcode for testing only
            end
            

            //logical execution section
                if(ret_micro) begin 
                    ret_micro    <= 1'b0;
                    pc_increment <= 1'b1; // Finishing a command increments the thread PC
                    //resets parameter program counter
                    p_pc_we = 1'b1;
                    p_pc_wdata = 8'b00000000;
                    flag_we = 1'b1;
                    flag_pos = 4'b0011;
                    flag_reg_bin_b2t_4t = 8'b00000000;
                    //yes it is intentional that we can override it later


                    //remind me to move this to a seperate func
                    //housekeeping section ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                    //best clock for housekeeping btw
                    //make sure that the aliased buffers that write into the command parameter addresses are written into the new command parameter space
                    //excellent for doing carry over stuff, which is intended behaviou
                    if(write_allow) begin
                        if(alias_addr_12t != 4'b0000) begin

                            if(alias_addr_12t >= 4'd5) begin
                            reg_data_to = am_buff_12t;
                            reg_w_en = 1'b1; // pull this to high for a single clock
                            override_cmd_sel_en = 1'b1; //1 clock
                            override_cmd_sel <= ~using_macroop_1;
                            end 
                            else begin
                                case(alias_addr_12t)
                                    4'd1: begin
                                        //scratchpad 0
                                        //fill scratchpad 
                                        sp0_full12t = 1'b1;
                                        sp0_we = 1'b1;
                                        sp0_reg_bin_b2t = am_buff_12t;
                                    end
                                    4'd2: begin
                                        //scratchpad 1
                                        sp1_full12t = 1'b1;
                                        sp1_we = 1'b1;
                                        sp1_reg_bin_b2t = am_buff_12t;
                                    end
                                    4'd3: begin
                                        //flags 
                                        flag_full12t = 1'b1;
                                        flag_we = 1'b1;
                                        flag_reg_bin_b2t = am_buff_12t;
                                    end
                                    4'd4: begin
                                        //ladder register
                                        ladder_full12t = 1'b1;
                                        ladder_we = 1'b1;
                                        ladd_reg_bin_b2t = am_buff_12t;
                                    end
                                endcase
                            end
                            alias_addr_12t <= 4'b0000;
                            //also check if this falls into the scratchpad registers /flag registers cause they are different
                        end
                        if(alias_active_4t) begin
                            alias_active_4t <= 1'b0;
                            //4t write
                            if(alias_addr_4t <= 3'd2) begin
                                // 0-2
                                //scratchpad 0
                                sp0_reg_bin_b2t = am_buff_4t;
                                sp0_we = 1'b1;
                            end else if (alias_addr_4t <= 3'd5) begin
                                //3-5
                                //scratchpad 1
                                sp1_reg_bin_b2t = am_buff_4t;
                                sp1_we = 1'b1;
                            end
                            else begin
                                //flag register sections
                                flag_we = 1'b1;
                                flag_reg_bin_b2t = am_buff_4t;
                            end
                            case(alias_addr_4t)
                                3'd0: sp0_pos  = 2'b01;
                                3'd1: sp0_pos  = 2'b10;
                                3'd2: sp0_pos  = 2'b11;
                                3'd3: sp1_pos  = 2'b01;
                                3'd4: sp1_pos  = 2'b10;
                                3'd5: sp1_pos  = 2'b11;
                                3'd6: flag_pos = 4'b0001;
                                3'd7: flag_pos = 4'b0010;
                            endcase
                        end
                        if(alias_active_1t) begin
                            alias_active_1t <= 1'b0;
                            //1t write
                            flag_reg_bin_b2t_1t = am_buff_1t;
                            flag_pos[3] = 1'b1;
                            flag_pos[2:0] = alias_active_1t;
                            flag_we = 1'b1;
                        end
                    end
                    //end of housekeeping
                end
                else begin
                    //either idling or working
                    if(threadstate == THREAD_EMPTY) begin
                        //literally idle
                        pipe_empty = 1'b1;
                        //do housekeeping here
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
                            command_req_new = ~command_req_new;
                        end else if(command_data_sent && execute_stage == 2'b0) begin
                            //read command input 
                            execute_done = 1'b0;
                            
                            
                            if(command == 4'b0000) begin
                                // 0 parameters
                                case(param_0)
                                    4'b0001: begin
                                        //mac ladder flush
                                        ladder_full12t = 1'b1;
                                        ladder_we = 1'b1;
                                        ladd_reg_bin_b2t = 24'd0;
                                    end
                                    4'b0010: begin
                                        //clear carry
                                        c_trit_we = 1'b1;
                                        carry_bin_b2t_1t = 2'b00;
                                    end
                                    4'b0011: begin
                                        //reset mask/alias
                                        am_buff_12t = 24'd0;
                                        am_buff_4t = 7'd0;
                                        am_buff_1t = 2'd0;
                                        alias_active_1t = 1'b0;
                                        alias_active_4t = 1'b0;
                                        alias_addr_12t = 4'd0;
                                    end
                                    4'b0100: begin
                                        //return
                                        ret_micro <= 1'b1;
                                        //unfinished
                                        //purge all cached microcode on the handler side

                                    end
                                    4'b0101: begin
                                        //disable write
                                        write_allow = 1'b0;
                                    end
                                    4'b0110: begin
                                        //enable write
                                        write_allow = 1'b1; //do not clear on a clockly basis 
                                    end
                                    4'b0111: begin
                                        //clear scratchpads
                                        if(write_allow) begin
                                            sp0_reg_bin_b2t = 24'd0;
                                            sp0_we = 1'b1;
                                            sp0_full12t = 1'b1; 
                                        end
                                    end
                                    4'b1000: begin
                                        //clear general purpose registers
                                    end
                                    4'b1001: begin
                                        //stall until next macro command is in
                                        if(!direct_instr) begin
                                            //to prevent a permanent stall, do nothing if direct instr is up
                                            if(threadstate != THREAD_FULL) begin
                                                stall = WAIT_PIPEFULL;
                                            end

                                            //unfinished: need calls on the other side to ask for more instructions
                                        end
                                    end
                                    4'b1010: begin
                                        //alu priority step up, housekeep
                                        //this needs to be implemented on all sides
                                    end
                                    4'b1011: begin
                                        //program counter increment
                                    end
                                    4'b1100: begin
                                        //clear mac offset
                                    end
                                    4'b1101: begin
                                        //mac offset++
                                    end
                                endcase
                            end else if(command == 4'b0001) begin 
                                // 1 parameter
                                case(param_0)
                                    4'b0001: begin
                                        //mac ladder set overflow target (12 trits)
                                        // internal complex method of handling the overflow
                                        /*
                                        if the 12 trits are not scratchpad, they are copied out first, then that section is replaced with the mac overflow and rewritten
                                        */
                                        //param 1 here is the 4 bit trit address
                                    end
                                    4'b0010: begin
                                        //mac ladder pump, 4 trit chunk source address 
                                        // Xxx0: fill normally, using xxx as the address to source 4 trits
                                        // Xxx1: fill with 0s
                                        //mac pumps do not shift the destination register trits, 
                                        //instead a counter keeps track of where each segment should go and overwrite that segment 
                                        //based off of the mac counter
                                    end
                                    4'b0011: begin
                                        //clear 4 trits of scratch/mac
                                        //4 trit chunk address
                                        // Xxx0: clear scratchpads
                                        // Xxx1: clear mac ladder section
                                    end
                                    4'b0100: begin
                                        //hold until register change
                                        //when a micro op with that register being used is reached, it must make sure the register is updated BEFORE it can proceed
                                        //12t register address
                                    end
                                    4'b0101: begin
                                        //set single use mask for 12 trits
                                        //the value at that 12t address will be copied into the mask buffer
                                        //param 1 is the addresss of the 12 trits that will be copied into the mask
                                    end
                                    4'b0110: begin
                                        //mask will not clear automatically for 12 trits
                                        //the value at that 12t address will be copied into the mask buffer
                                        //param 1 is the addresss of the 12 trits that will be copied into the mask
                                    end
                                    4'b0111: begin
                                        //4t mask
                                        if(param_0[0] == 1'b0) begin
                                            //xxx0 : single use
                                        end else begin
                                            //xxx1 : will not clear
                                        end
                                    end
                                    4'b1000: begin
                                        //1t mask
                                        if(param_0[0] == 1'b0) begin
                                            //xxx0 : single use
                                        end else begin
                                            //xxx1 : will not clear
                                        end
                                    end
                                    4'b1001: begin
                                        //hop micro op 4 trit address
                                        //xxx0 : action
                                        //xxx1 : param
                                        //xxx is the address of the 4 trit chunk
                                        if(param_0[0] == 1'b0) begin
                                            //xxx0 : action
                                        end else begin
                                            //xxx1 : param
                                        end
                                    end
                                    4'b1010: begin
                                        //read 12 trits of memory to that address (the memory address is overwritten from there)
                                        //this works in conjunction to provide memory read writes
                                        //this needs to work in conjunction with the memory bus
                                    end
                                    4'b1011: begin
                                        //alu hogging
                                        //for big operations that need alu max priority on all processes
                                        //param 1 is a binary number up to 16
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
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
                                    end
                                    4'b1011: begin
                                        //hop action counter using immediate
                                        //param 1 t3 t2
                                        //param 2 t1 t0
                                        //trits are encoded into 2 bits, using override encoding
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

                                //will fill this in later
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
                                        00xxxyyy : local
                                        01xxxyyy : overflow 
                                        10xxxyyy : underflow
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
            if(data_valid) begin
                if(op_read_stat > 2'b00) begin
                    //
                    case (op_read_state)
                        2'b01: cmd_param_reg [7:0] <= bus_data_in;  
                        2'b10: cmd_param_reg [15:8] <= bus_data_in;
                        2'b11: begin
                            op_read_state <= 2'b01;
                            cmd_param_id <= cmd_param_id + 1'b1;
                            //1 full 24 bit param is ready, push into the registers, increment cmd_param_id
                            //make sure that this happens all within a clock cycle because the bus data waits for no handshakes
                            cmd_we = 1'b1;
                        end
                    endcase
                    if(op_read_state != 2'b11) begin
                        op_read_state <= op_read_state + 1'b1;
                        cmd_we = 1'b0;
                    end
                end
            end
            if(data_done) begin
                // on the overlord side, make sure that when data done is sent, the bus is still transmitting properly,
                // cmd_param_id is not changed from last clock, data valid is low
                
                //if the pipe was empty, you can start executing command 0
                //if it was not, pull pipe full rej
                //either way something interesting is gonna happen
                //pull cmd write enable to false
                if(threadstate == THREAD_EMPTY) begin
                    //something here i forgot
                end 
                else begin
                    //set to full    
                    threadstate <= THREAD_FULL;
                end


                //when a new command is loaded in, thte commander should start presubmitting microcode requests
                //even when it's still receiving microcode requests
                //as well as check if its 11xxxxxx, indicating that the streamed data should be used as a micro op
                //
                //for direct execution, its loaded into a separate buffer, splitting the data into 11aaaa-- bbbbcccc dddd----

                //theres also an internal bus connecting the core to my friend, where only thread 0 is in use,
                // and we are locked to micro op direct execution as well as no memory operations
            end

        end
    end
        
end


//this is native. will switch out for binary soon
    native_12t_ladder ladder_reg(
        .VDD_1V65(VDD_1V65),
        .r_enable(),
        .w_enable(),
        .full12t(),
        .ladder_pump(),
        .position(),
        // 00 do nothing,
        // 01 write at position 0
        // 10 write at position 1
        // 11 write at position 2
        .data_in_12T(ladder_din_12T),
        inout [7:0]  data_in_8T,
        inout [3:0]  data_in_4T,
        inout [11:0] data_out_12T,
        inout [3:0]  data_out_4T
    );

//this is binary dont worry
    secondary_register secondary_register_inst (
        .clk                  (clk),
        .rst_n                (rst_n),
        .trit_out             (reg_data_from),
        .trit_in              (reg_data_to),
        .r_enable             (reg_r_en),
        .w_enable             (reg_w_en),
        .pc_increment         (pc_increment),
        .t_addr_in            (reg_addr),
        .sel_cmd1             (using_macroop_1),
        .override_cmd_sel_en  (override_cmd_sel_en),
        .override_cmd_sel     (override_cmd_sel),
        .binary_trit_in       (cmd_param_reg),
        .commandpipe_address  (cmd_param_id),
        .command_write_enable (cmd_we),
        .pc_overflow          (pc_overflow),
        .collision            (reg_collision)
    );

//this is also binary
    ternary_pc_4t parameter_microcode_pc (

        .clk        (clk),
        .rst_n      (rst_n),
        .inc_enable (p_pc_inc),
        .inc_step   (p_pc_inc_amt),
        .w_enable   (p_pc_we),
        .r_enable   (p_pc_re),
        .w_data     (p_pc_wdata),
        .r_data     (p_pc_rdata),
        .overflow   (p_ctr_overflow)
    );


    //remember to remove these later as everything is using override based 2 bit trit encoding
    native_4t4t4t scratchpad0 (
        .VDD_1V65     (VDD_1V65),
        .r_enable     (sp0_re),
        .w_enable     (sp0_we),
        .full12t      (sp0_full12t),
        .position     (sp0_pos),
        .data_in_12T  (sp0_din_12T),
        .data_in_4T   (sp0_din_4T),
        .data_out_12T (sp0_dout_12T),
        .data_out_4T  (sp0_dout_4T)
    );

    native_4t4t4t scratchpad1 (
        .VDD_1V65     (VDD_1V65),
        .r_enable     (sp1_re),
        .w_enable     (sp1_we),
        .full12t      (sp1_full12t),
        .position     (sp1_pos),
        .data_in_12T  (sp1_din_12T),
        .data_in_4T   (sp1_din_4T),
        .data_out_12T (sp1_dout_12T),
        .data_out_4T  (sp1_dout_4T)
    );

    native_8t_flags_4t_pc flag_trits (
        .VDD_1V65     (VDD_1V65),
        .r_enable     (flag_re),
        .w_enable     (flag_we),
        .position     (flag_pos),
        .full12t      (flag_full12t),
        .data_in_12T  (flag_din_12T),
        .data_in_4T   (flag_din_4T),
        .data_in_1T   (flag_din_1T),
        .data_out_12T (flag_dout_12T),
        .data_out_4T  (flag_dout_4T),
        .data_out_1T  (flag_dout_1T)
    );
    native_1t_storage carry_bit(
    .VDD_1V65(VDD_1V65),
    .r_enable(c_trit_re),
    .w_enable(c_trit_we),
    .data_in(c_trit_din),
    .data_out(c_trit_dout)
    );

endmodule