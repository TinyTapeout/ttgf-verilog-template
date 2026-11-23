module alu(
    // inout  wire        VDD_1V65,

    input  wire        clk,
    input  wire        rst_n,


    input  wire [2:0]  cmd_t0,
    input  wire [2:0]  tritwise_command_t0,
    input  wire [2:0]  tritshift_tritwise_t0,
    input  wire        cmd_ready_t0,
    input  wire        prio_inc_t0,           // Increment priority without issuing a command
    input  wire [3:0]  hog_time_t0,           // Hog window: 0=off, N>0 -> (N*2)+2 cycles (1 -> 4 cycles)
    output reg         res_ready_t0,

    input  wire [7:0]  t4_a_i_t0,
    input  wire [7:0]  t4_b_i_t0,
    input  wire [7:0]  t4_c_i_t0,             // 3rd 4-trit operand for add3
    output wire [7:0]  t4_o_t0,
    input  wire [1:0]  t1_a_i_t0,
    input  wire [1:0]  t1_b_i_t0,
    output wire [1:0]  t1_o_t0,
    output wire [15:0] multiply_o_t0,

    input  wire [2:0]  cmd_t1,
    input  wire [2:0]  tritwise_command_t1,
    input  wire [2:0]  tritshift_tritwise_t1,
    input  wire        cmd_ready_t1,
    input  wire        prio_inc_t1,
    input  wire [3:0]  hog_time_t1,
    output reg         res_ready_t1,

    input  wire [7:0]  t4_a_i_t1,
    input  wire [7:0]  t4_b_i_t1,
    input  wire [7:0]  t4_c_i_t1,
    output wire [7:0]  t4_o_t1,
    input  wire [1:0]  t1_a_i_t1,
    input  wire [1:0]  t1_b_i_t1,
    output wire [1:0]  t1_o_t1,
    output wire [15:0] multiply_o_t1,


    input  wire [2:0]  cmd_t2,
    input  wire [2:0]  tritwise_command_t2,
    input  wire [2:0]  tritshift_tritwise_t2,
    input  wire        cmd_ready_t2,
    input  wire        prio_inc_t2,
    input  wire [3:0]  hog_time_t2,
    output reg         res_ready_t2,

    input  wire [7:0]  t4_a_i_t2,
    input  wire [7:0]  t4_b_i_t2,
    input  wire [7:0]  t4_c_i_t2,
    output wire [7:0]  t4_o_t2,
    input  wire [1:0]  t1_a_i_t2,
    input  wire [1:0]  t1_b_i_t2,
    output wire [1:0]  t1_o_t2,
    output wire [15:0] multiply_o_t2
);

    reg [2:0] locked_t0, locked_t1, locked_t2;
    reg [1:0] priority_t0, priority_t1, priority_t2;
    reg [1:0] rr_turn;

    // thread hogging regs
    reg [1:0] hog_owner;   // 2'd0 = t0, 2'd1 = t1, 2'd2 = t2, 2'd3 = none
    // reg [4:0] hog_counter; // Counts down remaining hog cycles (max 15*2 + 2 = 32 -> needs 6 bits if 32, using [5:0])

    //hog counter with 6 bits to hold 32
    reg [5:0] hog_cycles_rem;
    wire      hog_active = (hog_cycles_rem != 6'd0);

    //who claims the throne~ (includes the cycle a new hog command is accepted)
    wire claim_hog_t0 = !hog_active && (hog_time_t0 != 4'd0);
    wire claim_hog_t1 = !hog_active && (hog_time_t0 == 4'd0) && (hog_time_t1 != 4'd0);
    wire claim_hog_t2 = !hog_active && (hog_time_t0 == 4'd0) && (hog_time_t1 == 4'd0) && (hog_time_t2 != 4'd0);

    wire is_hog_t0 = (hog_active && (hog_owner == 2'd0)) || claim_hog_t0;
    wire is_hog_t1 = (hog_active && (hog_owner == 2'd1)) || claim_hog_t1;
    wire is_hog_t2 = (hog_active && (hog_owner == 2'd2)) || claim_hog_t2;

    // request generation (filled up the tritwise 4 to turn it into add3)
    wire req_t0 = cmd_ready_t0 && !res_ready_t0 && (cmd_t0 != 3'd0);
    wire req_t1 = cmd_ready_t1 && !res_ready_t1 && (cmd_t1 != 3'd0);
    wire req_t2 = cmd_ready_t2 && !res_ready_t2 && (cmd_t2 != 3'd0);

    //detect collision
    wire t0_blocked = (cmd_t0 == locked_t1) || (cmd_t0 == locked_t2);
    wire t1_blocked = (cmd_t1 == locked_t0) || (cmd_t1 == locked_t2);
    wire t2_blocked = (cmd_t2 == locked_t0) || (cmd_t2 == locked_t1);

    //resolution, hogger has overbearing priority
    wire t0_beats_t1 = is_hog_t0 ? 1'b1 :
                       is_hog_t1 ? 1'b0 :
                       (priority_t0 < priority_t1) || (priority_t0 == priority_t1 && (rr_turn == 2'd0 || rr_turn == 2'd2));

    wire t0_beats_t2 = is_hog_t0 ? 1'b1 :
                       is_hog_t2 ? 1'b0 :
                       (priority_t0 < priority_t2) || (priority_t0 == priority_t2 && (rr_turn == 2'd0 || rr_turn == 2'd1));

    wire t1_beats_t2 = is_hog_t1 ? 1'b1 :
                       is_hog_t2 ? 1'b0 :
                       (priority_t1 < priority_t2) || (priority_t1 == priority_t2 && (rr_turn == 2'd1 || rr_turn == 2'd0));

    wire grant_t0 = req_t0 && !t0_blocked && (!req_t1 || cmd_t0 != cmd_t1 || t0_beats_t1) && (!req_t2 || cmd_t0 != cmd_t2 || t0_beats_t2);
    wire grant_t1 = req_t1 && !t1_blocked && (!req_t0 || cmd_t1 != cmd_t0 || !t0_beats_t1) && (!req_t2 || cmd_t1 != cmd_t2 || t1_beats_t2);
    wire grant_t2 = req_t2 && !t2_blocked && (!req_t0 || cmd_t2 != cmd_t0 || !t0_beats_t2) && (!req_t1 || cmd_t2 != cmd_t1 || !t1_beats_t2);

    //update the stuff
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            locked_t0      <= 3'd0; locked_t1      <= 3'd0; locked_t2      <= 3'd0;
            priority_t0    <= 2'd0; priority_t1    <= 2'd1; priority_t2    <= 2'd2;
            res_ready_t0   <= 1'b0; res_ready_t1   <= 1'b0; res_ready_t2   <= 1'b0;
            rr_turn        <= 2'd0;
            hog_owner      <= 2'd3;
            hog_cycles_rem <= 6'd0;
        end else begin
            // Rotate tie-breaker
            rr_turn <= (rr_turn == 2'd2) ? 2'd0 : rr_turn + 1'b1;

            //State Machine (0000 -> 0 cycles; N>0 -> N*2 + 2 cycles, so 0001 -> 4 cycles)
            if (hog_active) begin
                if (hog_cycles_rem == 6'd1) begin
                    hog_cycles_rem <= 6'd0;
                    hog_owner      <= 2'd3;
                end else begin
                    hog_cycles_rem <= hog_cycles_rem - 6'd1;
                end
            end else begin
                if (claim_hog_t0) begin
                    hog_owner      <= 2'd0;
                    hog_cycles_rem <= ({2'd0, hog_time_t0} << 1) + 6'd1; // Counts current cycle + (2*N + 1) more = 2*N + 2 total
                end else if (claim_hog_t1) begin
                    hog_owner      <= 2'd1;
                    hog_cycles_rem <= ({2'd0, hog_time_t1} << 1) + 6'd1;
                end else if (claim_hog_t2) begin
                    hog_owner      <= 2'd2;
                    hog_cycles_rem <= ({2'd0, hog_time_t2} << 1) + 6'd1;
                end
            end

            //update t0
            if (is_hog_t0) begin
                priority_t0 <= 2'd0;
            end else if (grant_t0) begin
                priority_t0 <= 2'd2;
            end else if (((req_t0 && !grant_t0) || prio_inc_t0) && (priority_t0 > 2'd0)) begin
                priority_t0 <= priority_t0 - 1'b1;
            end

            if (grant_t0) begin
                res_ready_t0 <= 1'b1;
                locked_t0    <= cmd_t0;
            end else if (!cmd_ready_t0) begin
                res_ready_t0 <= 1'b0;
                locked_t0    <= 3'd0;
            end

            //t1
            if (is_hog_t1) begin
                priority_t1 <= 2'd0;
            end else if (grant_t1) begin
                priority_t1 <= 2'd2;
            end else if (((req_t1 && !grant_t1) || prio_inc_t1) && (priority_t1 > 2'd0)) begin
                priority_t1 <= priority_t1 - 1'b1;
            end

            if (grant_t1) begin
                res_ready_t1 <= 1'b1;
                locked_t1    <= cmd_t1;
            end else if (!cmd_ready_t1) begin
                res_ready_t1 <= 1'b0;
                locked_t1    <= 3'd0;
            end

            //t2
            if (is_hog_t2) begin
                priority_t2 <= 2'd0;
            end else if (grant_t2) begin
                priority_t2 <= 2'd2;
            end else if (((req_t2 && !grant_t2) || prio_inc_t2) && (priority_t2 > 2'd0)) begin
                priority_t2 <= priority_t2 - 1'b1;
            end

            if (grant_t2) begin
                res_ready_t2 <= 1'b1;
                locked_t2    <= cmd_t2;
            end else if (!cmd_ready_t2) begin
                res_ready_t2 <= 1'b0;
                locked_t2    <= 3'd0;
            end
            //ctrl-f for the win!
        end
    end

    //command tracking and mux stuff
    wire [2:0] active_cmd_t0 = grant_t0 ? cmd_t0 : locked_t0;
    wire [2:0] active_cmd_t1 = grant_t1 ? cmd_t1 : locked_t1;
    wire [2:0] active_cmd_t2 = grant_t2 ? cmd_t2 : locked_t2;

    // 3'd1: Adder, 3'd2: Multiplier, 3'd3..5: Tritwise 1..3, 3'd6: Add3, 3'd7: Shifter
    wire [1:0] adder_mux      = (active_cmd_t0 == 3'd1) ? 2'd0 :
                                (active_cmd_t1 == 3'd1) ? 2'd1 :
                                (active_cmd_t2 == 3'd1) ? 2'd2 : 2'b11;

    wire [1:0] multi_mux      = (active_cmd_t0 == 3'd2) ? 2'd0 :
                                (active_cmd_t1 == 3'd2) ? 2'd1 :
                                (active_cmd_t2 == 3'd2) ? 2'd2 : 2'b11;

    wire [1:0] tritwise_1_mux = (active_cmd_t0 == 3'd3) ? 2'd0 :
                                (active_cmd_t1 == 3'd3) ? 2'd1 :
                                (active_cmd_t2 == 3'd3) ? 2'd2 : 2'b11;

    wire [1:0] tritwise_2_mux = (active_cmd_t0 == 3'd4) ? 2'd0 :
                                (active_cmd_t1 == 3'd4) ? 2'd1 :
                                (active_cmd_t2 == 3'd4) ? 2'd2 : 2'b11;

    wire [1:0] tritwise_3_mux = (active_cmd_t0 == 3'd5) ? 2'd0 :
                                (active_cmd_t1 == 3'd5) ? 2'd1 :
                                (active_cmd_t2 == 3'd5) ? 2'd2 : 2'b11;

    wire [1:0] add3_mux       = (active_cmd_t0 == 3'd6) ? 2'd0 :
                                (active_cmd_t1 == 3'd6) ? 2'd1 :
                                (active_cmd_t2 == 3'd6) ? 2'd2 : 2'b11;

    wire [1:0] shift_mux      = (active_cmd_t0 == 3'd7) ? 2'd0 :
                                (active_cmd_t1 == 3'd7) ? 2'd1 :
                                (active_cmd_t2 == 3'd7) ? 2'd2 : 2'b11;

    //native wrapper below
    native_wrapper_sim native_alu(
        // .VDD_1V65           (VDD_1V65),

        .active_cmd_t0      (active_cmd_t0),
        .active_cmd_t1      (active_cmd_t1),
        .active_cmd_t2      (active_cmd_t2),

        .adder_mux          (adder_mux),
        .add3_mux           (add3_mux),
        .multi_mux          (multi_mux),
        .shift_mux          (shift_mux),
        .tritwise_1_mux     (tritwise_1_mux),
        .tritwise_2_mux     (tritwise_2_mux),
        .tritwise_3_mux     (tritwise_3_mux),

        // Thread 0
        .trit4_in_a_0       (t4_a_i_t0),
        .trit4_in_b_0       (t4_b_i_t0),
        .trit4_in_c_0       (t4_c_i_t0),
        .tritwise_control_0 (tritwise_command_t0),
        .shift_control_0    (tritshift_tritwise_t0),
        .trit4_in_cin_0     (t1_a_i_t0),
        .trit1_in_b_0       (t1_b_i_t0),
        .trit4_out_0        (t4_o_t0),
        .trit4_out_cout_0   (t1_o_t0),
        .multiplier_out_0   (multiply_o_t0),

        // Thread 1
        .trit4_in_a_1       (t4_a_i_t1),
        .trit4_in_b_1       (t4_b_i_t1),
        .trit4_in_c_1       (t4_c_i_t1),
        .tritwise_control_1 (tritwise_command_t1),
        .shift_control_1    (tritshift_tritwise_t1),
        .trit4_in_cin_1     (t1_a_i_t1),
        .trit1_in_b_1       (t1_b_i_t1),
        .trit4_out_1        (t4_o_t1),
        .trit4_out_cout_1   (t1_o_t1),
        .multiplier_out_1   (multiply_o_t1),

        // Thread 2
        .trit4_in_a_2       (t4_a_i_t2),
        .trit4_in_b_2       (t4_b_i_t2),
        .trit4_in_c_2       (t4_c_i_t2),
        .tritwise_control_2 (tritwise_command_t2),
        .shift_control_2    (tritshift_tritwise_t2),
        .trit4_in_cin_2     (t1_a_i_t2),
        .trit1_in_b_2       (t1_b_i_t2),
        .trit4_out_2        (t4_o_t2),
        .trit4_out_cout_2   (t1_o_t2),
        .multiplier_out_2   (multiply_o_t2)
    );

endmodule