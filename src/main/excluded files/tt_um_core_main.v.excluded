//this is the actual core, beneath the wrapper in all of its glory 
module tt_um_core_main (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       module_enable,


    input  wire [7:0] input_pins,
    
    output reg  [7:0] io_input,
    output reg  [7:0] io_output,
    output reg  set_output,    

    output reg  [7:0] output_pins,

    input wire [1:0] register_read_stage,
    input wire [2:0] register_read_id,
    
    //to commander
    output wire b2t_translation_err,
    output wire microcode_miss,
    output wire flag_error,

    //internal routing and stuff below
    input reg o_microcode_fence,  //this pauses cpu execution after each micro operation
    input reg o_direct_instruction, //bypasses command buffers
    
    input wire [1:0] register_read_stage,
    input wire [2:0] register_read_id,
    input wire is_read,
    input wire single_execute,
    output reg single_execute_ack,
    input reg soft_reset,//to the cpu
    input wire [1:0] thread_select

);



//the alu can work in breakpoint mode, and command buffers, as well as input can operate in normal clocks

//converts into ternary with a couple of black boxes
//this verilog file handles data forwarding as well as tracking internal state for the machine

endmodule