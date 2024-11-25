// ucsbece154a_riscv.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited


module ucsbece154a_riscv (
    input               clk, reset,
    output wire         MemWrite_o,
    output wire  [31:0] Adr_o,
    output wire  [31:0] WriteData_o,
    input        [31:0] ReadData_i
);

wire [6:0] op;
wire [2:0] funct3;
wire funct7;
wire zero;

wire        PCEn;
wire        IRWrite;
wire        RegWrite;
wire  [1:0] ALUSrcA;
wire        AdrSrc;
wire  [1:0] ResultSrc;
wire  [1:0] ALUSrcB;
wire  [2:0] ALUControl;
wire  [2:0] ImmSrc;

ucsbece154a_controller c (
    .clk(clk),
    .reset(reset),
    .op_i(op),
    .funct3_i(funct3),
    .funct7_i(funct7),
    .zero_i(zero),
    .PCWrite_o(PCEn),
    .MemWrite_o(MemWrite_o),
    .IRWrite_o(IRWrite),
    .RegWrite_o(RegWrite),
    .ALUSrcA_o(ALUSrcA),
    .AdrSrc_o(AdrSrc),
    .ResultSrc_o(ResultSrc),
    .ALUSrcB_o(ALUSrcB),
    .ALUControl_o(ALUControl),
    .ImmSrc_o(ImmSrc)
);

ucsbece154a_datapath dp (
    .clk(clk),
    .reset(reset),
    .PCEn_i(PCEn),
    .ALUSrcA_i(ALUSrcA),
    .ALUSrcB_i(ALUSrcB),
    .RegWrite_i(RegWrite),
    .AdrSrc_i(AdrSrc),
    .IRWrite_i(IRWrite),
    .ResultSrc_i(ResultSrc),
    .ALUControl_i(ALUControl),
    .ImmSrc_i(ImmSrc),
    .zero_o(zero),
    .Adr_o(Adr_o),
    .WriteData_o(WriteData_o),
    .ReadData_i(ReadData_i),
    .op_o(op),
    .funct3_o(funct3),
    .funct7_o(funct7)
);
endmodule
