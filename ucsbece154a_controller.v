module ucsbece154a_controller (
    input               clk, reset,
    input               zero_i,
    input wire [5:0] opcode,
    output reg MemWrite_o,
    output reg MemRead_o,
    output reg RegWrite_o,
    output reg ALUSrcA_o,
    output reg ALUSrcB_o
    input         [6:0] opcode_i,
    input         [2:0] funct3_i,
    input         [6:0] funct7_i,
    output reg          PCEn_o,               // Enable PC update
    output reg          IRWrite_o,            // Control for Instruction Register write
    output reg [1:0]    ALUSrcA_o,            // ALU source A selection
    output reg [1:0]    ALUSrcB_o,            // ALU source B selection
    output reg          RegWrite_o,           // Register write enable
    output reg          AdrSrc_o,             // Address source selection
    output reg [1:0]    ResultSrc_o,          // Result source selection
    output reg [2:0]    ALUControl_o,         // ALU control signal
    output reg [2:0]    ImmSrc_o              // Immediate source selection
);
 always @(*) begin
        // Default values
        MemWrite_o = 0;
        MemRead_o = 0;
        RegWrite_o = 0;
        ALUSrcA_o = 0;
        ALUSrcB_o = 0;

        case(opcode)
            6'b101011: begin  // Store word (SW)
                MemWrite_o = 1;
            end
            6'b100011: begin  // Load word (LW)
                MemRead_o = 1;
                RegWrite_o = 1;
            end
            6'b000000: begin  // R-type (ADD, SUB, etc.)
                RegWrite_o = 1;
                ALUSrcA_o = 1;  // Example for R-type operations
            end
            default: begin
                // Handle other cases as necessary
            end
        endcase
    end
endmodule
`include "ucsbece154a_defines.vh"

// State Definitions (if needed)
localparam STATE_RESET  = 3'b000;
localparam STATE_FETCH  = 3'b001;
localparam STATE_DECODE = 3'b010;
localparam STATE_EXEC   = 3'b011;
localparam STATE_MEM    = 3'b100;
localparam STATE_WRITEBACK = 3'b101;

// State register (FSM)
reg [2:0] state, next_state;

// Always block for state transitions
always @(posedge clk or posedge reset) begin
    if (reset)
        state <= STATE_RESET;
    else
        state <= next_state;
end

// Next state and control logic
always @(*) begin
    // Default control signals
    PCEn_o = 0;
    IRWrite_o = 0;
    ALUSrcA_o = 2'b00;
    ALUSrcB_o = 2'b00;
    RegWrite_o = 0;
    AdrSrc_o = 0;
    ResultSrc_o = 2'b00;
    ALUControl_o = 3'b000;
    ImmSrc_o = 3'b000;
    
    next_state = state;
    
    case(state)
        STATE_RESET: begin
            next_state = STATE_FETCH;
        end
        
        STATE_FETCH: begin
            IRWrite_o = 1;  // Write instruction
            PCEn_o = 1;     // Enable PC update
            next_state = STATE_DECODE;
        end
        
        STATE_DECODE: begin
            case(opcode_i)
                7'b0110011: begin  // R-type (Add, Sub, etc.)
                    ALUSrcA_o = 2'b00;  // ALU source A (register)
                    ALUSrcB_o = 2'b00;  // ALU source B (register)
                    RegWrite_o = 1;     // Enable register write
                    ALUControl_o = 3'b010; // ALU control (e.g., ADD)
                    ResultSrc_o = 2'b00; // Use ALU result
                    next_state = STATE_EXEC;
                end
                
                7'b0000011: begin  // I-type (Load)
                    ALUSrcA_o = 2'b00;
                    ALUSrcB_o = 2'b01; // Immediate value
                    RegWrite_o = 1;
                    AdrSrc_o = 1;     // Address source (ALU)
                    ALUControl_o = 3'b010; // ALU control for add
                    ResultSrc_o = 2'b01; // Load from memory
                    next_state = STATE_MEM;
                end
                
                7'b1100011: begin  // B-type (Branch)
                    ALUSrcA_o = 2'b00;
                    ALUSrcB_o = 2'b01; // Immediate value for branch offset
                    ALUControl_o = 3'b110; // ALU control for subtraction
                    next_state = STATE_EXEC;
                end
                
                // More cases for other opcodes (e.g., Store, Jump, etc.)
                
                default: begin
                    next_state = STATE_RESET; // Default reset state
                end
            endcase
        end
        
        STATE_EXEC: begin
            // Execute ALU operation
            next_state = STATE_WRITEBACK;
        end
        
        STATE_MEM: begin
            // Memory read/write operations if needed
            next_state = STATE_WRITEBACK;
        end
        
        STATE_WRITEBACK: begin
            // Writeback to register
            next_state = STATE_FETCH; // Back to fetch
        end
        
        default: next_state = STATE_RESET;
    endcase
end

endmodule
