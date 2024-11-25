// ucsbece154a_datapath.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// TO DO: Add mising code below  
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

module ucsbece154a_datapath (
    input               clk, reset,
    input               PCEn_i,
    input         [1:0] ALUSrcA_i,
    input         [1:0] ALUSrcB_i,
    input               RegWrite_i,
    input               AdrSrc_i,
    input               IRWrite_i,
    input         [1:0] ResultSrc_i,
    input         [2:0] ALUControl_i,
    input         [2:0] ImmSrc_i,
    output  wire        zero_o,
    output  wire [31:0] Adr_o,                       
    output  wire [31:0] WriteData_o,                    
    input        [31:0] ReadData_i,
    output  wire [6:0]  op_o,
    output  wire [2:0]  funct3_o,
    output  wire        funct7_o
);

`include "ucsbece154a_defines.vh"

// Internal registers

reg [31:0] PC, OldPC, Instr, Data, A, B, ALUout;

// Buses connected to internal registers
reg [31:0] Result;
wire [4:0] a1 = Instr[19:15];
wire [4:0] a2 = Instr[24:20];
wire [4:0] a3 = Instr[11:7];
wire [31:0] rd1, rd2;
wire [31:0] ALUResult;


// Update for all internal registers

always @(posedge clk) begin
    if (reset) begin
        PC <= pc_start;
        OldPC <= {32{1'bx}};
        Instr <= {32{1'bx}};
        Data <= {32{1'bx}};
        A <= {32{1'bx}};
        B <= {32{1'bx}};
        ALUout <= {32{1'bx}};
    end else begin
        if (PCEn_i) PC <= Result;
        if (IRWrite_i) OldPC <= PC;
        if (IRWrite_i) Instr <= ReadData_i;
        Data <= ReadData_i;
        A <= rd1;
        B <= rd2;
        ALUout <= ALUResult;
    end
end

// **PUT THE REST OF YOUR CODE HERE**
// Register File
ucsbece154a_rf rf (
    .clk(clk),
    .reset(reset),
    .we3(RegWrite_i),
    .a1(a1),
    .a2(a2),
    .a3(a3),
    .wd3(Result),
    .rd1(rd1),
    .rd2(rd2)
);

// ALU
ucsbece154a_alu alu (
    .A(ALUInputA),
    .B(ALUInputB),
    .ALUControl(ALUControl_i),
    .Result(ALUResult),
    .Zero(zero_o)
);

// Extend unit
ucsbece154a_extender ext (
    .Instr(Instr[31:7]),
    .ImmSrc(ImmSrc_i),
    .ImmExt(ImmExt)
);

// Mux for ALU input A
reg [31:0] ALUInputA;
always @(*) begin
    case (ALUSrcA_i)
        2'b00: ALUInputA = PC;
        2'b01: ALUInputA = OldPC;
        2'b10: ALUInputA = A;
        default: ALUInputA = 32'bx;
    endcase
end

// Mux for ALU input B
reg [31:0] ALUInputB;
always @(*) begin
    case (ALUSrcB_i)
        2'b00: ALUInputB = B;
        2'b01: ALUInputB = 32'd4;     // For PC increment
        2'b10: ALUInputB = ImmExt;    // Immediate value
        default: ALUInputB = 32'bx;
    endcase
end

// Mux for Address source
assign Adr_o = (AdrSrc_i) ? PC : ALUout;

// Mux for WriteData_o (Data to be written to memory)
assign WriteData_o = B;

// Mux for selecting the Result (write-back to register file)
always @(*) begin
    case (ResultSrc_i)
        2'b00: Result = ALUout;     // ALU result
        2'b01: Result = Data;       // Memory read data
        2'b10: Result = PC + 32'd4; // PC + 4 (for JAL)
        default: Result = 32'bx;
    endcase
end

// Instruction fields for control unit
assign op_o = Instr[6:0];
assign funct3_o = Instr[14:12];
assign funct7_o = Instr[30];

endmodule
