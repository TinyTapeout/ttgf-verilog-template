`default_nettype none
module logic_tester(
    inout VDD_1V65,
    input clk,
    input rst_n,

    input  wire [7:0] ui_in,    // Dedicated inputs (Pins 0-7)
    output reg  [7:0] uo_out,   // Dedicated outputs (Pins 16-23)
    input  wire [7:0] uio_in,   // IOs: Input path (Pins 8-15)
    output wire [7:0] uio_out,  // IOs: Output path (Pins 8-15)
    output wire [7:0] uio_oe    // IOs: Enable path (active high: 1=drive, 0=high-Z)
);

assign uio_oe  = 8'd0; // no output enable
assign uio_out = 8'd0;

// only use input [7:4] for control (thats still 16 addresses!)
// io is only reading in
// uo is results
always @(*) begin
    uo_out = 8'd0;
    if (ui_in[2:0] == 3'b000) begin
        case (ui_in[7:4])
            4'b0000: begin
                uo_out[1:0] = passthrough_binary_result;
            end
            4'b0001: begin
                uo_out[1:0] = nti_binary_result;
            end
            4'b0010: begin
                uo_out[1:0] = pti_binary_result;
            end
            4'b0011: begin
                uo_out[1:0] = sti_binary_result;
            end
            // 4'b0100: begin
            //     uo_out[1:0] = max_binary_result;
            // end
            // 4'b0101: begin
            //     uo_out[1:0] = min_binary_result;
            // end
            4'b0110: begin
                uo_out[1:0] = mul_binary_result;
            end
            4'b0111: begin
                uo_out[1:0] = add_binary_result;
                uo_out[3:2] = add_binary_carry;
            end
            // 4'b1000: begin
            //     uo_out[1:0] = ram_binary_result;
            // end
            4'b1001: begin
                uo_out[1:0] = mux_binary_result_a;
                uo_out[3:2] = mux_binary_result_b;
                uo_out[5:4] = mux_binary_result_c;
            end
            //no time for experimental stuff
            // 4'b1010: begin
            //     uo_out[1:0] = sti_binary_result_e;
            // end
            default: begin
                uo_out = 8'd0;
            end
        endcase
    end
end

// passthrough
    wire passthrough_ternary_wire;
    wire [1:0] passthrough_binary_result;
    native_b2t passthrough_b2t
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(passthrough_ternary_wire)
    );
    native_t2b passthrough_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(passthrough_binary_result),
        .ternary_input(passthrough_ternary_wire)
    );

// nti
    wire nti_ternary_wire;
    wire nti_res_ternary_wire;
    wire [1:0] nti_binary_result;
    native_b2t nti_b2t
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(nti_ternary_wire)
    );
    nti nti_module
    (
        .ternary_input(nti_ternary_wire),
        .ternary_result(nti_res_ternary_wire)
    );
    native_t2b nti_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(nti_binary_result),
        .ternary_input(nti_res_ternary_wire)
    );

// pti
    wire pti_ternary_wire;
    wire pti_res_ternary_wire;
    wire [1:0] pti_binary_result;
    native_b2t pti_b2t
    (
        .binary_input(uio_in[1:0]),
        .ternary_result(pti_ternary_wire)
    );
    pti pti_module
    (
        .VDD_1V65(VDD_1V65),
        .ternary_input(pti_ternary_wire),
        .ternary_result(pti_res_ternary_wire)
    );
    native_t2b pti_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(pti_binary_result),
        .ternary_input(pti_res_ternary_wire)
    );

// sti
    wire sti_ternary_wire;
    wire sti_res_ternary_wire;
    wire [1:0] sti_binary_result;
    native_b2t sti_b2t
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(sti_ternary_wire)
    );
    sti sti_module
    (
        .VDD_1V65(VDD_1V65),
        .ternary_input(sti_ternary_wire),
        .ternary_result(sti_res_ternary_wire)
    );
    native_t2b sti_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(sti_binary_result),
        .ternary_input(sti_res_ternary_wire)
    );

// // max gate
//     wire max_ternary_wire_a;
//     wire max_ternary_wire_b;
//     wire max_res_ternary_wire;
//     wire [1:0] max_binary_result;
//     native_b2t max_b2t_a
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_input(uio_in[1:0]),
//         .ternary_result(max_ternary_wire_a)
//     );
//     native_b2t max_b2t_b
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_input(uio_in[3:2]),
//         .ternary_result(max_ternary_wire_b)
//     );
//     max_gate max_module
//     (
//         .VDD_1V65(VDD_1V65),
//         .input_1(max_ternary_wire_a),
//         .input_2(max_ternary_wire_b),
//         .result(max_res_ternary_wire)
//     );
//     native_t2b max_t2b
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_result(max_binary_result),
//         .ternary_input(max_res_ternary_wire)
//     );

// // min gate
//     wire min_ternary_wire_a;
//     wire min_ternary_wire_b;
//     wire min_res_ternary_wire;
//     wire [1:0] min_binary_result;
//     native_b2t min_b2t_a
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_input(uio_in[1:0]),
//         .ternary_result(min_ternary_wire_a)
//     );
//     native_b2t min_b2t_b
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_input(uio_in[3:2]),
//         .ternary_result(min_ternary_wire_b)
//     );
//     min_gate min_module
//     (
//         .VDD_1V65(VDD_1V65),
//         .input_1(min_ternary_wire_a),
//         .input_2(min_ternary_wire_b),
//         .result(min_res_ternary_wire)
//     );
//     native_t2b min_t2b
//     (
//         .VDD_1V65(VDD_1V65),
//         .binary_result(min_binary_result),
//         .ternary_input(min_res_ternary_wire)
//     );

// consensus gate
    wire con_ternary_wire_a;
    wire con_ternary_wire_b;
    wire con_res_ternary_wire;
    wire [1:0] con_binary_result;
    native_b2t con_b2t_a
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(con_ternary_wire_a)
    );
    native_b2t con_b2t_b
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[3:2]),
        .ternary_result(con_ternary_wire_b)
    );
    consensus con_module
    (
        .VDD_1V65(VDD_1V65),
        .input_1(con_ternary_wire_a),
        .input_2(con_ternary_wire_b),
        .result(con_res_ternary_wire)
    );
    native_t2b con_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(con_binary_result),
        .ternary_input(con_res_ternary_wire)
    );

// full adder
    wire add_ternary_wire_a;
    wire add_ternary_wire_b;
    wire add_ternary_wire_c;
    wire add_ternary_wire_res;
    wire add_ternary_wire_carry;
    wire [1:0] add_binary_result;
    wire [1:0] add_binary_carry;
    native_b2t add_b2t_a
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(add_ternary_wire_a)
    );
    native_b2t add_b2t_b
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[3:2]),
        .ternary_result(add_ternary_wire_b)
    );
    full_adder add_module(
        .VDD_1V65(VDD_1V65),
        .input_1(add_ternary_wire_a),
        .input_2(add_ternary_wire_b),
        .carry_in(add_ternary_wire_c),
        .result(add_ternary_wire_res),
        .carry(add_ternary_wire_carry)
    );
    native_t2b add_t2b_r
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(add_binary_result),
        .ternary_input(add_ternary_wire_res)
    );
    native_t2b add_t2b_c
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(add_binary_carry),
        .ternary_input(add_ternary_wire_carry)
    );

// mux module
    wire mux_ternary_wire_a;
    wire mux_ternary_wire_b;
    wire mux_ternary_res_a;
    wire mux_ternary_res_b;
    wire mux_ternary_res_c;
    wire [1:0] mux_binary_result_a;
    wire [1:0] mux_binary_result_b;
    wire [1:0] mux_binary_result_c;
    native_b2t mux_b2t_a
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(mux_ternary_wire_a)
    );
    native_b2t mux_b2t_b
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[3:2]),
        .ternary_result(mux_ternary_wire_b)
    );
    triple_dmux mux_module(
        .VDD_1V65(VDD_1V65),
        .ternary_input(mux_ternary_wire_a),
        .ternary_selector(mux_ternary_wire_b),
        .out_1(mux_ternary_res_a),
        .out_2(mux_ternary_res_b),
        .out_3(mux_ternary_res_c)
    );
    native_t2b mux_t2b_a
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(mux_binary_result_a),
        .ternary_input(mux_ternary_res_a)
    );
    native_t2b mux_t2b_b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(mux_binary_result_b),
        .ternary_input(mux_ternary_res_b)
    );
    native_t2b mux_t2b_c
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(mux_binary_result_c),
        .ternary_input(mux_ternary_res_c)
    );


/*
// sram cell
    wire ram_ternary_wire;
    wire ram_ternary_res;
    wire [1:0] ram_binary_result;
    native_b2t ram_b2t
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(ram_ternary_wire)
    );
    sram_cell ram_module (
        .VDD_1V65(VDD_1V65),
        .wr_en(uio_in[2]),
        .r_en(uio_in[3]),
        .input_1(ram_ternary_wire),
        .result(ram_ternary_res)
    );
    native_t2b ram_t2b
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(ram_binary_result),
        .ternary_input(ram_ternary_res)
    );


// sti experimental

    wire sti_ternary_wire_e;
    wire sti_res_ternary_wire_e;
    wire [1:0] sti_binary_result_e;
    native_b2t sti_b2t_e
    (
        .VDD_1V65(VDD_1V65),
        .binary_input(uio_in[1:0]),
        .ternary_result(sti_ternary_wire_e)
    );
    sti_experimental sti_module_e
    (
        .VDD_1V65(VDD_1V65),
        .ternary_input(sti_ternary_wire_e),
        .ternary_result(sti_res_ternary_wire_e)
    );
    native_t2b sti_t2b_e
    (
        .VDD_1V65(VDD_1V65),
        .binary_result(sti_binary_result_e),
        .ternary_input(sti_res_ternary_wire_e)
    );

    */

endmodule