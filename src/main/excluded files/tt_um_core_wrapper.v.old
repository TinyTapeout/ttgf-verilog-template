//this is the wrapper for both the main ternary alu and the controller
module tt_um_core_wrapper (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);
    wire b2t_translation_err;
    wire microcode_miss;
    wire single_execute_ack;
    wire flag_error;
    reg [2:0] module_select = uu_in[2:0];

    wire use_commander = (module_select == 3'b010);
    // Commander to Core
    wire o_microcode_fence;
    wire o_direct_instruction;
    wire [1:0] register_read_stage;
    wire [2:0] register_read_id;
    wire is_read;
    wire single_execute;
    wire soft_reset;
    wire [1:0] thread_select;
    wire show_tracker;

    // Outputs from inner main core
    wire [7:0] core_io_input; // Note: declared as output reg in your core module
    wire [7:0] core_io_output;
    wire       core_set_output;

    wire [7:0] commander_ui_in = use_commander  ? ui_in : 8'h00;
    wire [7:0] core_ui_in      = !use_commander ? ui_in : 8'h00;
    // Outputs from commander
    wire [7:0] commander_uo_out;
    wire [7:0] commander_tracker;

    wire [7:0] core_uo_out;
    // --- Module Instantiations ---

    tt_um_core_main core_inst (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .module_enable          (ena),
        
        // Data input
        .input_pins             (core_ui_in),
        .output_pins             (core_uo_out),
        // Output routing (Bidirectional IO handling)
        .io_input               (core_io_input), 
        .io_output              (core_io_output),
        .set_output             (core_set_output),
        
        // Internal Controller Inputs
        .register_read_stage    (register_read_stage),
        .register_read_id       (register_read_id),
        .o_microcode_fence      (o_microcode_fence),
        .o_direct_instruction   (o_direct_instruction),
        .is_read                (is_read),
        .single_execute         (single_execute),
        .soft_reset             (soft_reset),
        .thread_select          (thread_select),
        
        // Outputs to Controller
        .b2t_translation_err    (b2t_translation_err),
        .microcode_miss         (microcode_miss),
        .flag_error             (flag_error),
        .single_execute_ack     (single_execute_ack)
    );

    tt_um_core_commander commander_inst (
        .clk                    (clk),
        .rst_n                  (rst_n),
        .ena                    (ena),
        .ui_in                  (commander_ui_in), 
        
        // Output routing
        .uo_out                 (commander_uo_out),
        .tracker                (commander_tracker),
        
        // Inputs from Core
        .b2t_translation_err    (b2t_translation_err),
        .microcode_miss         (microcode_miss),
        .single_execute_ack     (single_execute_ack),
        
        // Internal Controller Outputs
        .o_microcode_fence      (o_microcode_fence),
        .o_direct_instruction   (o_direct_instruction),
        .show_tracker           (show_tracker),
        .register_read_stage    (register_read_stage),
        .register_read_id       (register_read_id),
        .is_read                (is_read),
        .single_execute         (single_execute),
        .soft_reset             (soft_reset),
        .thread_select          (thread_select)
    );
//converts into ternary with a couple of black boxes
//this verilog file handles data forwarding as well as tracking internal state for the machine


//1 commander plus 1 core with internal routing
//if showing tracker make sure internal io is set to OUT and streaming from commander

//if 010, input and output bus to commander
//if show_tracker io bus to commander tracker
//if 011, input, output(uo_out) to main core
// --- Routing Control ---
// Based on your previous module_select (010 = commander, 011 = core)


assign uo_out = use_commander ? commander_uo_out : core_uo_out;
assign uio_out = show_tracker ? commander_tracker : core_io_output;
assign uio_oe  = show_tracker ? 8'hFF             : {8{core_set_output}};

endmodule