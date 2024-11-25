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

wire [4:0] a1 = Instr[19:15];  // rs1 address
wire [4:0] a2 = Instr[24:20];  // rs2 address
wire [4:0] a3 = Instr[11:7];   // rd address
wire [31:0] rd1, rd2;
wire [31:0] ALUResult;
wire [31:0] ImmExtended;  // Sign-extended immediate value

// Register File
ucsbece154a_rf rf (
    .clk(clk),
    .reset(reset),
    .RegWrite_i(RegWrite_i),
    .a1(a1),  // rs1 address
    .a2(a2),  // rs2 address
    .a3(a3),  // rd address
    .rd1(rd1),  // Output from rs1
    .rd2(rd2),  // Output from rs2
    .WriteData_i(WriteData_o)  // Data to be written to the register file
);

// ALU
ucsbece154a_alu alu (
    .A(A),
    .B(B),
    .ALUControl_i(ALUControl_i),
    .ALUResult(ALUResult),
    .zero_o(zero_o)  // Zero output for comparison
);

// Sign Extension
ucsbece154a_sign_ext sign_ext (
    .ImmSrc_i(ImmSrc_i),
    .Instr(Instr),
    .ImmExtended(ImmExtended)
);

// Muxes
mux2to1 #(.WIDTH(32)) mux_A (
    .sel(ALUSrcA_i), 
    .a(A),
    .b(PC),  // Program counter as an alternative input
    .y(ALUSrcA)
);

mux2to1 #(.WIDTH(32)) mux_B (
    .sel(ALUSrcB_i),
    .a(B),
    .b(ImmExtended),  // Use the extended immediate value as the second source
    .y(ALUSrcB)
);

mux3to1 #(.WIDTH(32)) mux_Result (
    .sel(ResultSrc_i),
    .a(ALUResult),  // ALU result
    .b(ReadData_i),  // Data read from memory
    .c(OldPC),  // Previous PC (for branch or jump operations)
    .y(WriteData_o)  // Data to be written to register
);

// PC update logic
always @(posedge clk) begin
    if (reset) begin
        PC <= 32'b0;  // Reset PC to zero
    end else if (PCEn_i) begin
        PC <= Result;  // Update PC with the selected result
    end
end

// Address output (example: using ALU result for address generation)
assign Adr_o = (AdrSrc_i) ? ALUResult : PC;

assign op_o = Instr[6:0];  // Opcode extraction
assign funct3_o = Instr[14:12];  // funct3 extraction
assign funct7_o = Instr[31];  // funct7 extraction

endmodule
