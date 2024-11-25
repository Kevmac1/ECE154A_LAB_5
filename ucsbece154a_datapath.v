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

ucsbece154a_rf rf (
    /*FILL*/
);

ucsbece154a_alu alu (
    /*FILL*/
);

// Extend unit block
   /* FILL */

// Muxes
   /* FILL */
 

endmodule
