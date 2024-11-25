// ucsbece154a_controller.v
// All Rights Reserved
// Copyright (c) 2023 UCSB ECE
// Distribution Prohibited

module ucsbece154a_controller (
    input               clk, reset,
    input         [6:0] op_i, 
    input         [2:0] funct3_i,
    input               funct7_i,
    input 	        zero_i,
    output wire         PCWrite_o,
    output reg          MemWrite_o,    
    output reg          IRWrite_o,
    output reg          RegWrite_o,
    output reg    [1:0] ALUSrcA_o,
    output reg          AdrSrc_o,
    output reg    [1:0] ResultSrc_o,
    output reg    [1:0] ALUSrcB_o,
    output reg    [2:0] ALUControl_o,
    output reg    [2:0] ImmSrc_o
);

`include "ucsbece154a_defines.vh"

// **********   Extend unit    *********
always @ * begin
    case (op_i)
        instr_lw_op:        ImmSrc_o = 3'b000;       
        instr_sw_op:        ImmSrc_o = 3'b001; 
        instr_Rtype_op:     ImmSrc_o = 3'b010;  
        instr_beq_op:       ImmSrc_o = 3'b010;  
        instr_ItypeALU_op:  ImmSrc_o = 3'b000; 
        instr_jal_op:       ImmSrc_o = 3'b011; 
        instr_lui_op:       ImmSrc_o = 3'b100;  
        default: 	        ImmSrc_o = 3'bxxx; 
    endcase
end

// **********  ALU Control  *********
reg  [1:0] ALUOp;    // these are FFs updated each cycle 
wire RtypeSub = funct7_i & op_i[5];

always @ * begin
    case(ALUOp)
        ALUop_mem:                  ALUControl_o = ALUcontrol_add;
        ALUop_beq:                  ALUControl_o = ALUcontrol_sub;
        ALUop_other: 
            case(funct3_i) 
                instr_addsub_funct3: begin
                    if (RtypeSub)    ALUControl_o = ALUcontrol_sub;
                    else             ALUControl_o = ALUcontrol_add;  
                end
                instr_slt_funct3:     ALUControl_o = ALUcontrol_slt;  
                instr_or_funct3:      ALUControl_o = ALUcontrol_or;  
                instr_and_funct3:     ALUControl_o = ALUcontrol_and;  
                default:              ALUControl_o = 3'bxxx;
            endcase
        default:                      ALUControl_o = 3'bxxx;
    endcase
end

// **********  Generating PC Write  *********
reg Branch, PCUpdate;   // these are FFs updated each cycle 

assign PCWrite_o = Branch & zero_i | PCUpdate; 

// ******************************************
// *********  Main FSM  *********************
// ******************************************

// *********  FSM state transistion  ****** 
reg [3:0] state; //  FSM FFs encoding the state 
reg [3:0] state_next;

always @ * begin
    if (reset) begin
        state_next = state_Fetch;  
    end else begin             
        case (state) 
            state_Fetch:           state_next = state_Decode;  
            state_Decode: begin
                case (op_i) 
                    instr_lw_op:       state_next = state_MemAdr;  
                    instr_sw_op:       state_next = state_MemAdr;  
                    instr_Rtype_op:    state_next = state_ExecuteR;  
                    instr_beq_op:      state_next = state_BEQ;  
                    instr_ItypeALU_op: state_next = state_ExecuteI;  
                    instr_lui_op:      state_next = state_LUI;  
                    instr_jal_op:      state_next = state_JAL;  
                    default:           state_next = state_Fetch;
                endcase
            end
            state_MemAdr: begin 
                case (op_i)
                    instr_lw_op:       state_next = state_MemRead;  
                    instr_sw_op:       state_next = state_MemWrite;  
                    default:           state_next = state_Fetch;
                endcase
            end
            state_MemRead:           state_next = state_MemWB;  
            state_MemWB:             state_next = state_Fetch;  
            state_MemWrite:          state_next = state_Fetch;  
            state_ExecuteR:          state_next = state_ALUWB;  
            state_ALUWB:             state_next = state_Fetch;  
            state_ExecuteI:          state_next = state_ALUWB;  
            state_JAL:               state_next = state_Fetch;  
            state_BEQ:               state_next = state_Fetch;  
            state_LUI:               state_next = state_Fetch;     
            default:                 state_next = state_Fetch;
        endcase
    end
end

// *******  Control signal generation  ********

reg [13:0] controls_next;
wire       PCUpdate_next, Branch_next, MemWrite_next, IRWrite_next, RegWrite_next, AdrSrc_next;
wire [1:0] ALUSrcA_next, ALUSrcB_next, ResultSrc_next, ALUOp_next;

assign {
    PCUpdate_next, Branch_next, MemWrite_next, IRWrite_next, RegWrite_next,
    ALUSrcA_next, ALUSrcB_next, AdrSrc_next, ResultSrc_next, ALUOp_next
} = controls_next;

always @ * begin
   case (state_next)
        state_Fetch:     controls_next = 14'b1_0_1_1_0_00_00_0_00_00;  // Set control signals for Fetch state
        state_Decode:    controls_next = 14'b0_0_1_1_1_01_01_1_00_00;  // Set for Decode state (example)
        state_MemAdr:    controls_next = 14'b0_0_0_0_0_00_01_1_00_00;  // For MemAdr state (example)
        state_MemRead:   controls_next = 14'b0_1_0_1_1_00_01_1_00_00;  // Set for MemRead state
        state_MemWrite:  controls_next = 14'b0_1_1_1_0_00_01_0_00_00;  // For MemWrite state (example)
        state_ExecuteR:  controls_next = 14'b0_0_0_1_1_01_00_0_00_00;  // Set for ExecuteR (R-type)
        state_ALUWB:     controls_next = 14'b1_0_0_1_0_00_00_1_00_00;  // For ALUWB (example)
        state_ExecuteI:  controls_next = 14'b0_1_1_1_1_01_00_0_00_00;  // For ExecuteI (I-type)
        state_JAL:       controls_next = 14'b1_0_0_1_1_10_00_0_00_00;  // JAL operation
        state_BEQ:       controls_next = 14'b0_1_0_1_1_00_01_0_01_00;  // BEQ operation
        state_LUI:       controls_next = 14'b1_0_1_1_1_00_00_1_00_00;  // LUI operation
        default:         controls_next = 14'bx_x_x_x_x_xx_xx_x_xx_xx;
   endcase
end


// *******  Updating control and main FSM FFs  ********
always @(posedge clk) begin
    state <= state_next;
    PCUpdate <= PCUpdate_next;
    Branch <= Branch_next;
    MemWrite_o <= MemWrite_next;
    IRWrite_o <= IRWrite_next;
    RegWrite_o <= RegWrite_next;
    ALUSrcA_o <= ALUSrcA_next;
    ALUSrcB_o <= ALUSrcB_next;
    AdrSrc_o <= AdrSrc_next;
    ResultSrc_o <= ResultSrc_next;
    ALUOp <= ALUOp_next;
end

endmodule

