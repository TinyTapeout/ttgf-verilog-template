module alu(
    /*
    the alu has a floating priority system and decides to operate upon the data
    it has been slimmed down to stateless
    */
    inout VDD_1V65,

    input   wire clk,
    input   wire rst_n,
    //command lines
    input   wire [2:0] cmd_t0,
    input   wire [2:0] tritwise_command_t0,

    input   wire [2:0] tritshift_tritwise_t0, //trit shifts up to 3 front and back

    input   wire cmd_ready_t0,
    output  reg  res_ready_t0,
    //4 trit
    inout [3:0] t4_a_i_t0,
    inout [3:0] t4_b_i_t0,
    inout [3:0]   t4_o_t0,
    //1 trit
    inout       t1_a_i_t0,
    inout       t1_b_i_t0,
    inout         t1_o_t0,

    inout [7:0] multiply_o_t0,

    //command lines
    input   wire [2:0] cmd_t1,
    //
    input   wire [2:0] tritwise_command_t1,

    input   wire [2:0] tritshift_tritwise_t1, //trit shifts up to 3 front and back

    input   wire cmd_ready_t1,
    output  reg  res_ready_t1,
    //4 trit
    inout [3:0] t4_a_i_t1,
    inout [3:0] t4_b_i_t1,
    inout [3:0]   t4_o_t1,
    //1 trit
    inout       t1_a_i_t1,
    inout       t1_b_i_t1,
    inout         t1_o_t1,
    
    inout [7:0] multiply_o_t1,


    input   wire [2:0] cmd_t2,
    //
    input   wire [2:0] tritwise_command_t2,

    input   wire [2:0] tritshift_tritwise_t2, //trit shifts up to 3 front and back

    input   wire cmd_ready_t2,
    output  reg  res_ready_t2,
    //4 trit
    inout [3:0] t4_a_i_t2,
    inout [3:0] t4_b_i_t2,
    inout [3:0]   t4_o_t2,
    //1 trit
    inout       t1_a_i_t2,
    inout       t1_b_i_t2,
    inout         t1_o_t2,
    inout [7:0] multiply_o_t2

);

native_wrapper native_alu(
    
    .VDD_1V65(VDD_1V65),
    
    .adder_mux(adder_mux),
    .multi_mux(multi_mux),
    .shift_mux(shift_mux),
    .tritwise_1_mux(tritwise_1_mux),
    .tritwise_2_mux(tritwise_2_mux),
    .tritwise_3_mux(tritwise_3_mux),
    .tritwise_4_mux(tritwise_4_mux),

    .trit4_in_a_0(t4_a_i_t0),
    .trit4_in_b_0(t4_b_i_t0),
    .tritwise_control_0(tritwise_command_t0),
    .multiplier_out_0(multiply_o_t0),
    
    .trit4_in_cin_0(t1_a_i_t0),
    .trit1_in_b_0(t1_b_i_t0),

    .trit4_out_0(t4_o_t0),
    .trit4_out_cout_0(t1_o_t0),

    //thread 1
    .trit4_in_a_1(t4_a_i_t1),
    .trit4_in_b_1(t4_b_i_t1),
    .tritwise_control_1(tritwise_command_t1),
    .multiplier_out_1(multiply_o_t1),
    
    .trit4_in_cin_1(t1_a_i_t1),
    .trit1_in_b_1(t1_b_i_t1),

    .trit4_out_1(t4_o_t1),
    .trit4_out_cout_1(t1_o_t1),

    //thread 2
    .trit4_in_a_2(t4_a_i_t2),
    .trit4_in_b_2(t4_b_i_t2),
    .tritwise_control_2(tritwise_command_t2),
    .multiplier_out_2(multiply_o_t2),
    
    .trit4_in_cin_2(t1_a_i_t2),
    .trit1_in_b_2(t1_b_i_t2),

    .trit4_out_2(t4_o_t2),
    .trit4_out_cout_2(t1_o_t2)
);

reg [2:0] locked_t0, locked_t1, locked_t2;
reg [1:0] priority_t0, priority_t1, priority_t2;
reg [1:0] rr_turn; // Tie-breaker: 0=t0 favors, 1=t1 favors, 2=t2 favors

// 1. Valid Request Generation
wire req_t0 = cmd_ready_t0 && !res_ready_t0 && (cmd_t0 != 3'd0);
wire req_t1 = cmd_ready_t1 && !res_ready_t1 && (cmd_t1 != 3'd0);
wire req_t2 = cmd_ready_t2 && !res_ready_t2 && (cmd_t2 != 3'd0);

// 2. Conflict Detection (Equality checks against active locks)
wire t0_blocked = (cmd_t0 == locked_t1) || (cmd_t0 == locked_t2);
wire t1_blocked = (cmd_t1 == locked_t0) || (cmd_t1 == locked_t2);
wire t2_blocked = (cmd_t2 == locked_t0) || (cmd_t2 == locked_t1);

// 3. Parallel Arbitration (Flattened Critical Path & Tie Breaking)
wire t0_beats_t1 = (priority_t0 < priority_t1) || (priority_t0 == priority_t1 && (rr_turn == 2'd0 || rr_turn == 2'd2));
wire t0_beats_t2 = (priority_t0 < priority_t2) || (priority_t0 == priority_t2 && (rr_turn == 2'd0 || rr_turn == 2'd1));
wire t1_beats_t2 = (priority_t1 < priority_t2) || (priority_t1 == priority_t2 && (rr_turn == 2'd1 || rr_turn == 2'd0));

wire grant_t0 = req_t0 && !t0_blocked && (cmd_t0 != cmd_t1 || t0_beats_t1) && (cmd_t0 != cmd_t2 || t0_beats_t2);
wire grant_t1 = req_t1 && !t1_blocked && (cmd_t1 != cmd_t0 || !t0_beats_t1) && (cmd_t1 != cmd_t2 || t1_beats_t2);
wire grant_t2 = req_t2 && !t2_blocked && (cmd_t2 != cmd_t0 || !t0_beats_t2) && (cmd_t2 != cmd_t1 || !t1_beats_t2);

// 4. State & Priority Updates
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        locked_t0 <= 3'd0; locked_t1 <= 3'd0; locked_t2 <= 3'd0;
        priority_t0 <= 2'd0; priority_t1 <= 2'd1; priority_t2 <= 2'd2;
        res_ready_t0 <= 1'b0; res_ready_t1 <= 1'b0; res_ready_t2 <= 1'b0;
        rr_turn <= 2'd0;
    end else begin
        // Rotate tie-breaker
        rr_turn <= (rr_turn == 2'd2) ? 2'd0 : rr_turn + 1'b1;

        // Thread 0 Updates
        if (req_t0 && !grant_t0 && priority_t0 > 2'd0) priority_t0 <= priority_t0 - 1'b1;
        if (grant_t0) begin res_ready_t0 <= 1'b1; locked_t0 <= cmd_t0; end 
        else if (!cmd_ready_t0) begin res_ready_t0 <= 1'b0; locked_t0 <= 3'd0; end
        
        // [Apply identical logic for Thread 1 and Thread 2 updates here]
    end
end

// 5. Translating 3-bit Tags to Mux Controls and Enable Flags
// Assume 3'd1 is Adder, 3'd2 is Multiplier, etc.
wire [1:0] adder_mux      = (grant_t0 || (locked_t0 == 3'd1)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd1)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd1)) ? 2'd2 : 2'b11;
wire [1:0] multi_mux      = (grant_t0 || (locked_t0 == 3'd2)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd2)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd2)) ? 2'd2 : 2'b11;
wire [1:0] tritwise_1_mux = (grant_t0 || (locked_t0 == 3'd3)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd3)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd3)) ? 2'd2 : 2'b11;
wire [1:0] tritwise_2_mux = (grant_t0 || (locked_t0 == 3'd4)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd4)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd4)) ? 2'd2 : 2'b11;
wire [1:0] tritwise_3_mux = (grant_t0 || (locked_t0 == 3'd5)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd5)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd5)) ? 2'd2 : 2'b11;
wire [1:0] tritwise_4_mux = (grant_t0 || (locked_t0 == 3'd6)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd6)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd6)) ? 2'd2 : 2'b11;
wire [1:0] shift_mux      = (grant_t0 || (locked_t0 == 3'd7)) ? 2'd0 :
                            (grant_t1 || (locked_t1 == 3'd7)) ? 2'd1 :
                            (grant_t2 || (locked_t2 == 3'd7)) ? 2'd2 : 2'b11;
// wire [1:0] tritwise_1_mux = (grant_t0 || (locked_t0 == 3'd7)) ? 2'd0 :
//                             (grant_t1 || (locked_t1 == 3'd7)) ? 2'd1 :
//                             (grant_t2 || (locked_t2 == 3'd7)) ? 2'd2 : 2'b11;
endmodule