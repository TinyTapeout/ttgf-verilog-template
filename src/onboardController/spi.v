// -------------------------------------------------------------------------
// this is the top level module
// -------------------------------------------------------------------------
module dual_spi_flash_ctrl (
    input wire clk,           //clk
    input wire reset,         // Active high reset

    input wire [23:0] addrA,
    input wire reqA,          // Pulse high to start reading from addrA
    output wire [7:0] dataA,  // Streamed data output
    output wire readyA,       // Pulses high for 1 clock cycle when dataA is valid

    input wire [23:0] addrB,
    input wire reqB,          // Pulse high to start reading from addrB
    output wire [7:0] dataB,  // Streamed data output
    output wire readyB,       // Pulses high for 1 clock cycle when dataB is valid

    // Physical SPI Pins for Chip A
    output wire cs_nA,        // Chip Select (Active Low)
    output wire sclkA,        // SPI Clock
    output wire mosiA,        // Master Out Slave In
    input  wire misoA,        // Master In Slave Out

    // Physical SPI Pins for Chip B
    output wire cs_nB,
    output wire sclkB,
    output wire mosiB,
    input  wire misoB
);

    //these flashes use 2 submoduels for stuff
    spi_flash_reader readerA (
        .clk(clk),
        .reset(reset),
        .req(reqA),
        .addr(addrA),
        .data_out(dataA),
        .data_ready(readyA),
        .cs_n(cs_nA),
        .sclk(sclkA),
        .mosi(mosiA),
        .miso(misoA)
    );

    spi_flash_reader readerB (
        .clk(clk),
        .reset(reset),
        .req(reqB),
        .addr(addrB),
        .data_out(dataB),
        .data_ready(readyB),
        .cs_n(cs_nB),
        .sclk(sclkB),
        .mosi(mosiB),
        .miso(misoB)
    );

endmodule


// -------------------------------------------------------------------------
// single spi flash + state machine 
// -------------------------------------------------------------------------
module asic_spi_flash_reader (
    input  wire        clk,        //20 to 80 MHz
    input  wire        reset,      //active-high reset (DFT friendly)
    
    // Core Interface
    input  wire        req,        // Pulse high to start/restart reading
    input  wire [23:0] addr,       // 24-bit Flash memory address
    output reg  [7:0]  data_out,   // 8-bit streamed data
    output reg         data_ready, // Pulses high for 1 system clk when data_out is valid
    
    // Physical SPI Pins (To be routed to GF180 GPIO pad macros at top level)
    output reg         cs_n,       // Chip Select (Active Low)
    output reg         sclk,       // SPI Clock (Generated synchronously)
    output reg         mosi,       // Master Out Slave In
    input  wire        miso        // Master In Slave Out (Raw input from pad)
);

    //---------------------------------------------------------------------
    //MISO 2-Stage Synchronizer (Metastability Prevention)
    //---------------------------------------------------------------------
    reg miso_sync_1;
    reg miso_sync_2;
    
    always @(posedge clk) begin
        if (reset) begin
            miso_sync_1 <= 1'b0;
            miso_sync_2 <= 1'b0;
        end else begin
            miso_sync_1 <= miso;       // First stage captures raw async data
            miso_sync_2 <= miso_sync_1;// Second stage resolves metastability
        end
    end

    //---------------------------------------------------------------------
    //State Machine & Oversampling Definitions
    //---------------------------------------------------------------------
    localparam ST_IDLE     = 3'd0;
    localparam ST_DESELECT = 3'd1;//enforces CS_N high time (tSHSL) between jumps
    localparam ST_CMD      = 3'd2;
    localparam ST_ADDR     = 3'd3;
    localparam ST_STREAM   = 3'd4;

    localparam READ_CMD    = 8'h03;

    reg [2:0]  state;
    reg [23:0] latched_addr;
    reg [4:0]  bit_cnt;    //counts bits for current phase
    reg [2:0]  tick_cnt;   //0 to 4 (5x oversampling counter)
    reg [7:0]  shift_reg;  //Assembles incoming MISO bits
    reg [2:0]  deselect_cnt;//counter for CS_N high duration

//state machien
    always @(posedge clk) begin
        if (reset) begin
            state        <= ST_IDLE;
            cs_n         <= 1'b1;
            sclk         <= 1'b0;
            mosi         <= 1'b0;
            data_out     <= 8'h00;
            data_ready   <= 1'b0;
            bit_cnt      <= 5'd0;
            tick_cnt     <= 3'd0;
            shift_reg    <= 8'h00;
            deselect_cnt <= 3'd0;
            latched_addr <= 24'd0;
        end else begin
            // Default signal states
            data_ready <= 1'b0; 

            case (state)
                ST_IDLE: begin
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    if (req) begin
                        latched_addr <= addr;
                        state        <= ST_CMD;
                        cs_n         <= 1'b0;
                        bit_cnt      <= 5'd7;
                        tick_cnt     <= 3'd4; //counter so next tick is 0 (sclk rises)
                        mosi         <= READ_CMD[7]; //setup first bit immediately
                    end
                end

                ST_DESELECT: begin
                    //flash chips require CS_N to remain high 
                    //for ~10-50ns when aborting a transaction before asserting low
                    cs_n <= 1'b1;
                    sclk <= 1'b0;
                    if (deselect_cnt == 0) begin
                        state    <= ST_CMD;
                        cs_n     <= 1'b0;
                        bit_cnt  <= 5'd7;
                        tick_cnt <= 3'd4;
                        mosi     <= READ_CMD[7];
                    end else begin
                        deselect_cnt <= deselect_cnt - 1;
                    end
                end

                ST_CMD, ST_ADDR, ST_STREAM: begin
                    //Handle Abort/Jump Request during active transaction
                    if (req) begin
                        state        <= ST_DESELECT;
                        cs_n         <= 1'b1;     //Immediately terminate flash read
                        sclk         <= 1'b0;
                        latched_addr <= addr;     //Latch the new jump address
                        deselect_cnt <= 3'd7;     //Wait 8 system clocks (~100ns @ 80MHz)
                    end else begin
                        
                        // 5x Oversampling Logic
                        if (tick_cnt == 3'd4) begin
                            tick_cnt <= 3'd0;
                        end else begin
                            tick_cnt <= tick_cnt + 1;
                        end

                        // SCLK Generation & Data Sampling
                        case (tick_cnt)
                            // Tick 0: Rising edge of SCLK
                            3'd0: sclk <= 1'b1; 
                            
                            // Tick 2: Dead center of SCLK HIGH. Sample synchronized MISO here.
                            3'd2: shift_reg <= {shift_reg[6:0], miso_sync_2}; 
                            
                            // Tick 3: Falling edge of SCLK. Drive next MOSI bit.
                            3'd3: begin
                                sclk <= 1'b0;
                                
                                if (bit_cnt == 0) begin
                                    // Phase transitions
                                    if (state == ST_CMD) begin
                                        state   <= ST_ADDR;
                                        bit_cnt <= 5'd23;
                                        mosi    <= latched_addr[23];
                                    end 
                                    else if (state == ST_ADDR) begin
                                        state   <= ST_STREAM;
                                        bit_cnt <= 5'd7;
                                        mosi    <= 1'b0; // MOSI ignored during read phase
                                    end 
                                    else if (state == ST_STREAM) begin
                                        data_out   <= shift_reg;
                                        data_ready <= 1'b1;
                                        bit_cnt    <= 5'd7; // Loop continuously
                                    end
                                end else begin
                                    // Shift next bit out
                                    bit_cnt <= bit_cnt - 1;
                                    if (state == ST_CMD)  mosi <= READ_CMD[bit_cnt - 1];
                                    if (state == ST_ADDR) mosi <= latched_addr[bit_cnt - 1];
                                end
                            end
                        endcase
                    end
                end
            endcase
        end
    end
endmodule