module dummy_worker (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       module_enable,
    input  wire [4:0] ctrl_payload,
    input  wire [7:0] data_payload,
    
    output reg  [7:0] out_dedicated,
    output reg  [7:0] out_bus_write,
    output reg  [7:0] out_bus_oe
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_dedicated <= 8'h00;
            out_bus_write <= 8'h00;
            out_bus_oe    <= 8'h00;
        end else if (module_enable) begin
            //execute module-specific logic here
            out_dedicated <= {3'b000, ctrl_payload}; //example logic
            out_bus_write <= ~data_payload;          //example logic
            out_bus_oe    <= 8'hFF;                  //claim IO bus
        end else begin
            //when disabled, drop the Output Enable flag to release the shared bus
            out_bus_oe <= 8'h00;
        end
    end

endmodule