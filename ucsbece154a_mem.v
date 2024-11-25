// ucsbece154a_mem.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited

`define MIN(A,B) (((A)<(B))?(A):(B))

module ucsbece154a_mem #(
    parameter TEXT_SIZE = 64,
    parameter DATA_SIZE = 64
) (
    input               clk, we_i,
    input        [31:0] a_i,
    input        [31:0] wd_i,
    output wire  [31:0] rd_o
);

// instantiate/initialize BRAM
reg [31:0] TEXT [0:TEXT_SIZE-1];
initial $readmemh("text.dat",TEXT);
reg [31:0] DATA [0:DATA_SIZE-1];
// initial $readmemh("data.dat",DATA);

// calculate address bounds for memory
localparam TEXT_START = 32'h00010000;
localparam TEXT_END   = `MIN( TEXT_START + (TEXT_SIZE*4), 32'h10000000);
localparam DATA_START = 32'h10000000;
localparam DATA_END   = `MIN( DATA_START + (DATA_SIZE*4), 32'h80000000);

// calculate address widths
localparam TEXT_ADDRESS_WIDTH = $clog2(TEXT_SIZE);
localparam DATA_ADDRESS_WIDTH = $clog2(DATA_SIZE);

// create flags to specify whether in-range of each BRAM
wire text_enable = (TEXT_START <= a_i) && (a_i < TEXT_END);
wire data_enable = (DATA_START <= a_i) && (a_i < DATA_END);

// create addresses for each BRAM
wire [TEXT_ADDRESS_WIDTH-1:0] text_address = a_i[2 +: TEXT_ADDRESS_WIDTH]-(TEXT_START[2 +: TEXT_ADDRESS_WIDTH]);
wire [DATA_ADDRESS_WIDTH-1:0] data_address = a_i[2 +: DATA_ADDRESS_WIDTH]-(DATA_START[2 +: DATA_ADDRESS_WIDTH]);

// get read-data from each BRAM
wire [31:0] text_data = TEXT[ text_address ];
wire [31:0] data_data = DATA[ data_address ];

// set rd_o iff a_i is in range of one of the BRAMs
assign rd_o =
    text_enable ? text_data :
    data_enable ? data_data :
    {32{1'bz}}; // not driven by this memory

// write routine
always @ (posedge clk) begin
    if (we_i) begin
        if (we_i && data_enable)
            DATA[data_address] <= wd_i;
`ifdef SIM
        if (a_i[1:0]!=2'b0)
            $warning("Attempted to write to invalid address 0x%h. Address coerced to 0x%h.", a_i, (a_i&(~32'b11)));
        if (!data_enable)
            $warning("Attempted to write to out-of-range address 0x%h.", (a_i&(~32'b11)));
`endif
    end
end

endmodule

`undef MIN
