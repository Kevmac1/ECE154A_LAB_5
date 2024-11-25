// ucsbece154a_top.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited


module ucsbece154a_top (
    input clk, reset
);

wire we;
wire [31:0] a, wd, rd;

// processor and memories are instantiated here
ucsbece154a_riscv riscv (
    .clk(clk), .reset(reset),
    .MemWrite_o(we),
    .Adr_o(a),
    .WriteData_o(wd),
    .ReadData_i(rd)
);
ucsbece154a_mem mem (
    .clk(clk),
    .we_i(we),
    .a_i(a),
    .wd_i(wd),
    .rd_o(rd)
);

endmodule
