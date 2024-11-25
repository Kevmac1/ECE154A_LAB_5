// ucsbece154a_datapath.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// TO DO: Add missing code below  
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
reg [31:0] Result;

// Buses connected to internal registers
wire [4:0] a1 = Instr[19:15];
wire [4:0] a2 = Instr[24:20];
wire [4:0] a3 = Instr[11:7];
wire [31:0] rd1, rd2;
wire [31:0] ALUResult;

// Fetch Instruction
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

// Register File
ucsbece154a_rf rf (
    .clk(clk),
    .reset(reset),
    .RegWrite_i(RegWrite_i),
    .a1(a1),
    .a2(a2),
    .a3(a3),
    .wd(WriteData_o),
    .rd1(rd1),
    .rd2(rd2)
);

// ALU
ucsbece154a_alu alu (
    .A(A),
    .B(B),
    .ALUControl(ALUControl_i),
    .ALUResult(ALUResult),
    .zero(zero_o)
);

// Extend Unit
ucsbece154a_extender extender (
    .Instr(Instr),
    .ImmSrc(ImmSrc_i),
    .ImmResult(WriteData_o)
);

// MUXes
// Mux for ALU Src A
always @(*) begin
    case (ALUSrcA_i)
        2'b00: Result = A;            // A
        2'b01: Result = OldPC;       // OldPC
        2'b10: Result = 32'b0;       // Zero
        default: Result = 32'bx;     // Undefined
    endcase
end

// Mux for ALU Src B
always @(*) begin
    case (ALUSrcB_i)
        2'b00: WriteData_o = B;      // B
        2'b01: WriteData_o = ImmResult; // ImmExt
        2'b10: WriteData_o = 32'b0;  // Zero
        default: WriteData_o = 32'bx; // Undefined
    endcase
end

// Mux for ResultSrc
always @(*) begin
    case (ResultSrc_i)
        2'b00: WriteData_o = ALUout;       // ALU result
        2'b01: WriteData_o = ReadData_i;   // Memory read
        2'b10: WriteData_o = OldPC + 4;    // PC + 4 (for JAL)
        default: WriteData_o = 32'bx;       // Undefined
    endcase
end

// Address Source Mux
assign Adr_o = AdrSrc_i ? ALUout : OldPC + 4;

// Output for op and funct3, funct7 fields
assign op_o = Instr[6:0];
assign funct3_o = Instr[14:12];
assign funct7_o = Instr[31];

// End of module
endmodule
