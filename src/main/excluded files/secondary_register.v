module secondary_register (

    input clk,
    input rst_n,
    output reg [23:0] trit_out,
    input  [23:0] trit_in,
    input r_enable,
    input w_enable,
    
    input       pc_increment,
    input [3:0] t_addr_in, //thread address
    input sel_cmd1,

    input override_cmd_sel_en,
    input override_cmd_sel,


    //2 spots, one from the alu and the thread manager to read and write, and another for the commandpipe feed
    // 6 per command, 
    //dock at 0 to signify its done, dock at any other invalid address to signify problem

    input  wire [23:0]  binary_trit_in,
    input  wire [2:0]   commandpipe_address,
    input  wire         command_write_enable,
    output wire         pc_overflow,
    output wire         collision
);

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
    reg fetch_state_active;

    // Drive the PC enable directly with the 1-cycle pulse
    assign pc_inc_enable = fetch_state_active;
    always @(*) begin
        if (r_enable) begin
            case (t_addr_in)
                // Command Registers
                4'd9:  trit_out = sel_cmd1 ? cmd1_reg_0 : cmd0_reg_0;
                4'd10: trit_out = sel_cmd1 ? cmd1_reg_1 : cmd0_reg_1;
                4'd11: trit_out = sel_cmd1 ? cmd1_reg_2 : cmd0_reg_2;
                4'd12: trit_out = sel_cmd1 ? cmd1_reg_3 : cmd0_reg_3;
                4'd13: trit_out = sel_cmd1 ? cmd1_reg_4 : cmd0_reg_4;
                4'd14: trit_out = sel_cmd1 ? cmd1_reg_5 : cmd0_reg_5;
                
                // General Purpose Registers
                4'd6: trit_out = gp_reg_6;
                4'd7: trit_out = gp_reg_7;
                4'd8: trit_out = gp_reg_8;
                
                // Program Counter Read
                4'd5: trit_out = pc_r_data;
                
                default: trit_out = 24'b0;
            endcase
        end else begin
            trit_out = 24'b0;
        end
    end
    wire cmd_sel = override_cmd_sel_en ? override_cmd_sel : sel_cmd1;

    // Determine target banks (0 for cmd0, 1 for cmd1)
    wire bin_target_bank = ~sel_cmd1;
    wire t_target_bank   = cmd_sel;

    // Check if both sides are targeting valid command register ranges
    wire t_is_cmd   = (t_addr_in >= 4'd9 && t_addr_in <= 4'd14);
    wire bin_is_cmd = (commandpipe_address >= 3'd1 && commandpipe_address <= 3'd6);

    // Address alignment check: commandpipe_address 1 corresponds to t_addr_in 9 (1 + 8 = 9)
    wire addresses_match = ( {1'b0, commandpipe_address} + 4'd8 ) == t_addr_in;

    // Collision goes high if both are writing to the same bank and the same aligned address
    assign collision = command_write_enable && w_enable && 
                       bin_is_cmd && t_is_cmd && 
                       (bin_target_bank == t_target_bank) && 
                       addresses_match;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            
            cmd0_reg_0 <= 24'b0; cmd0_reg_1 <= 24'b0; cmd0_reg_2 <= 24'b0;
            cmd0_reg_3 <= 24'b0; cmd0_reg_4 <= 24'b0; cmd0_reg_5 <= 24'b0;
            
            cmd1_reg_0 <= 24'b0; cmd1_reg_1 <= 24'b0; cmd1_reg_2 <= 24'b0;
            cmd1_reg_3 <= 24'b0; cmd1_reg_4 <= 24'b0; cmd1_reg_5 <= 24'b0;
            
            gp_reg_6 <= 24'b0; gp_reg_7 <= 24'b0; gp_reg_8 <= 24'b0;
            
            fetch_state_active <= 1'b0;
        end else begin 
            
            // PORT 1: Command Pipe Background Loading
            // "Docking" at 0 or invalid addresses is safe because command_write_enable will be 0.
            // although it is possible to stream 1 while reading the other, it is much safer to NOT do that
            if (w_enable) begin
                case (t_addr_in)
                    // Command Registers Write
                    4'd9:  if (t_target_bank) cmd1_reg_0 <= trit_in; else cmd0_reg_0 <= trit_in;
                    4'd10: if (t_target_bank) cmd1_reg_1 <= trit_in; else cmd0_reg_1 <= trit_in;
                    4'd11: if (t_target_bank) cmd1_reg_2 <= trit_in; else cmd0_reg_2 <= trit_in;
                    4'd12: if (t_target_bank) cmd1_reg_3 <= trit_in; else cmd0_reg_3 <= trit_in;
                    4'd13: if (t_target_bank) cmd1_reg_4 <= trit_in; else cmd0_reg_4 <= trit_in;
                    4'd14: if (t_target_bank) cmd1_reg_5 <= trit_in; else cmd0_reg_5 <= trit_in;
                    
                    // General Purpose Registers Write
                    4'd6: gp_reg_6 <= trit_in;
                    4'd7: gp_reg_7 <= trit_in;
                    4'd8: gp_reg_8 <= trit_in;
                endcase
            end

            // ---------------------------------------------------------
            // PORT 1: Binary Side Background Loading (Processed Second)
            // ---------------------------------------------------------
            // By placing this AFTER the w_enable block, Verilog's non-blocking (<=) 
            // assignment rules dictate that if both blocks target the exact same 
            // register, this block's assignment takes PRIORITY in the delta cycle.
            if (command_write_enable) begin
                if (bin_target_bank == 1'b1) begin
                    // Pipe targets cmd1
                    case (commandpipe_address)
                        3'b001: cmd1_reg_0 <= binary_trit_in;
                        3'b010: cmd1_reg_1 <= binary_trit_in;
                        3'b011: cmd1_reg_2 <= binary_trit_in;
                        3'b100: cmd1_reg_3 <= binary_trit_in;
                        3'b101: cmd1_reg_4 <= binary_trit_in;
                        3'b110: cmd1_reg_5 <= binary_trit_in;
                    endcase
                end else begin
                    // Pipe targets cmd0
                    case (commandpipe_address)
                        3'b001: cmd0_reg_0 <= binary_trit_in;
                        3'b010: cmd0_reg_1 <= binary_trit_in;
                        3'b011: cmd0_reg_2 <= binary_trit_in;
                        3'b100: cmd0_reg_3 <= binary_trit_in;
                        3'b101: cmd0_reg_4 <= binary_trit_in;
                        3'b110: cmd0_reg_5 <= binary_trit_in;
                    endcase
                end
            end
            fetch_state_active <= pc_increment && !fetch_state_active; 
        end
    end

endmodule