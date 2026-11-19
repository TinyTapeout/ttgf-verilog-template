//the logic container is in charge of housing the 3 threads and 1 core plus controller
//the overlord is a separate spectator
/*
for the logic container it is in charge of handling command streaming into the things, 
can also do memory read and writes through the io for all 3 threads, as well as handling the changes between microcode, thread command intake and stuff 
*/



//also logic testing etc is done somewhere else, preferably somewhere that i can't see
module logic_container(
    
    inout VDD_1V65,
    input clk,
    input rst_n,

    input  wire [7:0] ui_in,    // Dedicated inputs (Pins 0-7)
    output wire [7:0] uo_out,   // Dedicated outputs (Pins 16-23)
    input  wire [7:0] uio_in,   // IOs: Input path (Pins 8-15)
    output wire [7:0] uio_out,  // IOs: Output path (Pins 8-15)
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 1=drive, 0=high-Z)

    //add inputs from core overcommand
);




//for thread 0
    wire [2:0] cmd_t0_wire;
    wire [2:0] tritwise_cmd_t0_wire;
    wire [2:0] shift_wise_cmd_t0_wire;
    wire cmd_ready_t0_wire;  
    wire res_ready_t0_wire;  
    wire [3:0] t4_a_t0_wire;
    wire [3:0] t4_b_t0_wire;
    wire [3:0] t4_c_t0_wire;
    wire t1_a_t0_wire;
    wire t1_b_t0_wire;
    wire t1_c_t0_wire;
    wire [7:0] t8_t0_wire;
//for thread 1
    wire [2:0] cmd_t1_wire;
    wire [2:0] tritwise_cmd_t1_wire;
    wire [2:0] shift_wise_cmd_t1_wire;
    wire cmd_ready_t1_wire;  
    wire res_ready_t1_wire;  
    wire [3:0] t4_a_t1_wire;
    wire [3:0] t4_b_t1_wire;
    wire [3:0] t4_c_t1_wire;
    wire t1_a_t1_wire;
    wire t1_b_t1_wire;
    wire t1_c_t1_wire;
    wire [7:0] t8_t1_wire;
//for thread 2
    wire [2:0] cmd_t2_wire;
    wire [2:0] tritwise_cmd_t2_wire;
    wire [2:0] shift_wise_cmd_t2_wire;
    wire cmd_ready_t2_wire;  
    wire res_ready_t2_wire;  
    wire [3:0] t4_a_t2_wire;
    wire [3:0] t4_b_t2_wire;
    wire [3:0] t4_c_t2_wire;
    wire t1_a_t2_wire;
    wire t1_b_t2_wire;
    wire t1_c_t2_wire;
    wire [7:0] t8_t2_wire;

alu core_0 (
    /*
    the alu has a floating priority system and decides to operate upon the data
    it has been slimmed down to stateless
    */
     
    .VDD_1V65(VDD_1V65),

    .clk(clk),
    .rst_n(rst_n),
    //command lines
    .cmd_t0(cmd_t0_wire),
    .tritwise_command_t0(tritwise_cmd_t0_wire),
    .tritshift_tritwise_t0(shift_wise_cmd_t0_wire), //trit shifts up to 3 front and back

    .cmd_ready_t0(cmd_ready_t0_wire),
    .res_ready_t0(res_ready_t0_wire),
    //4 trit
    .t4_a_i_t0  (t4_a_t0_wire),
    .t4_b_i_t0  (t4_b_t0_wire),
    .t4_o_t0    (t4_c_t0_wire),
    //1 trit
    .t1_a_i_t0  (t1_a_t0_wire),
    .t1_b_i_t0  (t1_b_t0_wire),
    .t1_o_t0    (t1_c_t0_wire),
    .multiply_o_t0(t8_t0_wire),

    //thread 1
    .cmd_t1(cmd_t1_wire),
    .tritwise_command_t1(tritwise_cmd_t1_wire),
    .tritshift_tritwise_t1(shift_wise_cmd_t1_wire),
    .cmd_ready_t1(cmd_ready_t1_wire),
    .res_ready_t1(res_ready_t1_wire),
    //4 trit
    .t4_a_i_t1  (t4_a_t1_wire),
    .t4_b_i_t1  (t4_b_t1_wire),
    .t4_o_t1    (t4_c_t1_wire),
    //1 trit
    .t1_a_i_t1  (t1_a_t1_wire),
    .t1_b_i_t1  (t1_b_t1_wire),
    .t1_o_t1    (t1_c_t1_wire),
    .multiply_o_t1(t8_t1_wire),


    //thread 2
    .cmd_t2(cmd_t2_wire),
    .tritwise_command_t2(tritwise_cmd_t2_wire),
    .tritshift_tritwise_t2(shift_wise_cmd_t2_wire),
    .cmd_ready_t2(cmd_ready_t2_wire),
    .res_ready_t2(res_ready_t2_wire),
    //4 trit
    .t4_a_i_t2  (t4_a_t2_wire),
    .t4_b_i_t2  (t4_b_t2_wire),
    .t4_o_t2    (t4_c_t2_wire),
    //1 trit
    .t1_a_i_t2  (t1_a_t2_wire),
    .t1_b_i_t2  (t1_b_t2_wire),
    .t1_o_t2    (t1_c_t2_wire),
    .multiply_o_t2(t8_t2_wire)
);

microcode microcode_parser
(
    .clk(clk),
    .rst_n(rst_n),
    
    //thread 0 handler
        .macro_op_t0(macro_op_wire_t0),
        //from 1  to 31 , each micro op corresponds to 2 byte chunked reads
        //from 32 to 112, each micro op corresponds to 4 byte chunked reads
        //from 113 to 191, each micro op corresponds to 8 byte chunked reads
        // uses an 1024 offset for parameters
        // 32 x 2 + 80 x 4 + 80 x 8 = 1024
        //anything beyond 191 is ILLEGAL and should never be seen
        .microcode_action_offset_t0(converted_action_t0), //up to 128 but since its 4 trit to convert to this it's more like 81
        .microcode_param_offset_t0(converted_param_t0),
        .flush(micro_flush_wire_t0),//this forces the microcode handler to pause, flush cache and read from action position & param position, as well as macro op position
        //chunked reads mean that you start from indexes 0 , 4 , 8 , 12, etc, but the offsets can allow it to read any position
        //the params are chunked separately, on the same chip but with an offset
        .next_microop(micro_req_wire_t0),//this one turns on -> off , send new ; off -> on , send new 
        .data_ready(micro_sent_wire_t0),
        //resulting stuff
        .action_t0(micro_cmd_wire_t0),
        .param0_t0(micro_param0_wire_t0),
        .param1_t0(micro_param1_wire_t0),
        .param2_t0(micro_param2_wire_t0),

        //this shows the resulting positions of the main in case the thread manager needs it for anything 
        //we probably don't need this cause error checking is for chuds
        // output reg [6:0] microcode_action_offset_t0_res, 
        // output reg [6:0] microcode_param_offset_t0_res,

    // ================= THREAD 1 INTERFACE =================
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
        // output reg  [6:0]  microcode_action_offset_t1_res,
        // output reg  [6:0]  microcode_param_offset_t1_res,

    // ================= THREAD 2 INTERFACE =================
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
        // output reg  [6:0]  microcode_action_offset_t2_res,
        // output reg  [6:0]  microcode_param_offset_t2_res,

    // ================= MAIN SRAM BUS =================
    // main bus
        .bus(dbus),
        .address(microcode_address),
        output  cycle_threads,
        output  microcode_idle,
);


wire [11:0] microcode_address;
wire [7:0] dbus;

    always @(*) begin
        case (ui_in[2:0])
            3'b010: begin
                //thread -1 addressing
            end
            3'b011: begin
                //thread 0 addressing 
            end
            3'b100: begin
                //thread 1 addressing
            end
            3'b101: begin
                //microcode addressing
                ui_in[7:3] = dbus[4:0];
                uio_in[2:0] = dbus[7:5];
                uio_out[7:3] = microcode_address[4:0]; //
                uio_oe[2:0] = 3'b000;
                uio_oe[7:3] = 5'b11111;
                uo_out[6:0] = microcode_address[11:5]; 
            end
        endcase
    end

//thread 0 interconnects 
    wire [23:0] register_from_reg_t0;
    wire [23:0] register_to_reg_t0;
    wire w_en_t0;
    wire r_en_t0;
    wire pc_increment_t0;
    wire [3:0] trit_address_reg_t0;
    wire select_cmd_t0;

    wire [23:0] command_pipe_in_t0;
    wire [3:0] commandpipe_address_t0;
    wire write_enable;

    wire [7:0] macro_op_wire_t0;
    wire [7:0] action_ctr_wire_t0;
    wire [7:0] param_ctr_wire_t0;
    wire [3:0] micro_cmd_wire_t0;
    wire [3:0] micro_param0_wire_t0;
    wire [3:0] micro_param1_wire_t0;
    wire [3:0] micro_param2_wire_t0;
    wire micro_req_wire_t0;
    wire micro_sent_wire_t0;
    wire micro_flush_wire_t0;

thread_command thread0_command(
    
    .clk(clk),
    .rst_n(rst_n),

    //from the microcode parsers
        .macro_op(macro_op_wire_t0),
        .action_counter(action_ctr_wire_t0),
        .param_counter(param_ctr_wire_t0),
        .flush(micro_flush_wire_t0),
        .command(micro_cmd_wire_t0),
        .param_0(micro_param0_wire_t0),
        .param_1(micro_param1_wire_t0),
        .param_2(micro_param2_wire_t0),
        .command_req_new(micro_req_wire_t0),
        .command_data_sent(micro_sent_wire_t0),
        // output wire ,
    
    //secondary register
        .trit_in_from_2nd_reg(register_from_reg_t0),
        .trit_out_to_reg(register_to_reg_t0),
        .r_enable(r_en_t0),
        .w_enable(w_en_t0),
        .pc_increment(pc_increment_t0),
        .t_addr(trit_address_reg_t0), //thread address
        .sel_cmd1(select_cmd_t0),

    //alu interface
        .alu_command(cmd_t0_wire),
        .tritwise_command(tritwise_cmd_t0_wire),
        tritshift_tritwise_command(shift_wise_cmd_t0_wire), //trit shifts up to 3 front and back
        .cmd_ready(cmd_ready_t0_wire),
        .res_ready_in(res_ready_t0_wire),
        //4 trit
        .t4_a_out(t4_a_t0_wire),
        .t4_b_out(t4_b_t0_wire),
        .t1_a_out(t1_a_t0_wire),
        .t1_b_out(t1_b_t0_wire),
        //results here
        .t4_in(t4_c_t0_wire),
        .t1_in(t1_c_t0_wire),
        .multiply_in(t8_t0_wire),

    //overlord interface
        input  wire stall,
        input  wire [7:0] bus_data_in,
        output wire request_set_pc,
        output wire pipe_empty,
);
wire [6:0] converted_action_t0;
wire [6:0] converted_param_t0;
//short for "ternary to unsigned binary Action counter thread0"
ternary_to_unsigned_binary_logic_convert_4t t2ubActr_t0(
    .in_trits(action_ctr_wire_t0),
    .out_bin(converted_action_t0),
);
ternary_to_unsigned_binary_logic_convert_4t t2ubPctr_t0(
    .in_trits(param_ctr_wire_t0),
    .out_bin(converted_param_t0),
);



endmodule