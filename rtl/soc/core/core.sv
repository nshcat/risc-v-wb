module core(
    input logic clk_in,
    input logic reset_in
);

// ==== Constants ====
typedef enum logic[6:0]
{
    OPCODE_ARITH_R      = 7'b0110011,   // rd <- rs1 OP rs2
    OPCODE_ARITH_I      = 7'b0010011,   // rd <- rs1 OP Iimm
    OPCODE_BRANCH       = 7'b1100011,   // if(rs1 OP rs2) PC<-PC+Bimm
    OPCODE_JAL          = 7'b1101111,   // rd <- PC+4; PC<-PC+Jimm
    OPCODE_JALR         = 7'b1100111,   // rd <- PC+4; PC<-rs1+Iimm
    OPCODE_LOAD         = 7'b0000011,   // rd <- mem[rs1+Iimm]
    OPCODE_STORE        = 7'b0100011,   // mem[rs1+Simm] <- rs2
    OPCODE_LUI          = 7'b0110111,   // rd <- Uimm
    OPCODE_AUIPC        = 7'b0010111,   // rd <- PC + Uimm
    OPCODE_SYSTEM       = 7'b1110011    // rd <- CSR <- rs1/uimm5
} opcode_t;

// Arithmetic interpretation of FUNCT3 instruction component
typedef enum logic [2:0]
{
    FUNCT3_ARITH_ADDSUB     = 3'b000,
    FUNCT3_ARITH_SHIFTL     = 3'b001,
    FUNCT3_ARITH_SHIFTR     = 3'b101,
    FUNCT3_ARITH_SLT        = 3'b010,
    FUNCT3_ARITH_SLTU       = 3'b011,
    FUNCT3_ARITH_XOR        = 3'b100,
    FUNCT3_ARITH_OR         = 3'b110,
    FUNCT3_ARITH_AND        = 3'b111
} funct3_arith_t;

// Funct7 field constants
typedef enum logic [6:0]
{
    FUNCT7_DEFAULT          = 7'b0,
    FUNCT7_ALT              = 7'b0100000        // Alternative operation, SUB instead of ADD, arithmetic shift
} funct7_t;

// ==== Instruction decoding ====

// The currently latched instruction
logic [31:0] instruction;

opcode_t instr_opcode = instruction[6:0];
logic [4:0] instr_rs1 = instruction[19:15];
logic [4:0] instr_rs2 = instruction[24:20];
logic [4:0] instr_rd = instruction[11:7];
logic [4:0] instr_shamt = instruction[24:20];
logic [2:0] instr_func3 = instruction[14:12];
logic [6:0] instr_func7 = instruction[31:25];
logic [4:0] instr_shamt = instruction[24:20];

logic is_arith_reg = (instr_opcode == OPCODE_ARITH_R);
logic is_arith_imm = (instr_opcode == OPCODE_ARITH_I);
logic is_arith = (is_arith_reg | is_arith_imm);
logic is_branch = (instr_opcode == OPCODE_BRANCH);
logic is_jal = (instr_opcode == OPCODE_JAL);
logic is_jalr = (instr_opcode == OPCODE_JALR);
logic is_system = (instr_opcode == OPCODE_SYSTEM);
logic is_lui = (instr_opcode == OPCODE_LUI);
logic is_auipc = (instr_opcode == OPCODE_AUIPC);
logic is_load = (instr_opcode == OPCODE_LOAD);
logic is_store = (instr_opcode == OPCODE_STORE);

// IMM extraction
logic imm_sign = instruction[31];
logic [31:0] imm_I = { (imm_sign ? 21'h1FFFFF : 21'h0), instruction[30:20] };
logic [31:0] imm_S = { (imm_sign ? 21'h1FFFFF : 21'h0), instruction[30:25], instruction[11:8], instruction[7] };
logic [31:0] imm_B = { (imm_sign ? 20'hFFFFF : 20'h0), instruction[7], instruction[30:25], instruction[11:8], 1'b0 };
logic [31:0] imm_U = { instruction[31:12], 12'h0 };
logic [31:0] imm_J = { (imm_sign ? 12'hFFF : 12'h0), instruction[19:12], instruction[20], instruction[30:25], instruction[24:21], 1'b0 };


// ==== Register file ====

// XXX We might need two registers for the two read register values RS1 and RS2 here, if we do the
// statemachine thing.

logic [31:0] registers [31:0];

initial begin
    integer i;
    for (i = 0; i < 32; i = i + 1)
        registers[i] = 32'h0;
end

// Register writeback and reset
always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        integer i;
        for (i = 0; i < 32; i = i + 1)
            registers[i] <= 32'h0;
    end
    else begin
        if (do_writeback) begin
            if (instr_rd != 5'h0) begin
                registers[instr_rd] <= writeback_value;
            end
        end
    end
end

// ==== ALU operations ====

// XXX
// XXX
// /!\ Rework this - dont do any of this reusing (decision!), but implement it more in your own way
// => Just do another subtraction in the branch section, and addition in the load section, instead of weirdly reusing
// the code here
// XXX
// XXX

// ArithR and ArithI both use the full ALU result: rd <- rs1 OP (rs2 | imm_I)
// JALR just uses the add result: PC <- rs1 + Iimm
// Branches use the subtraction to evaluate the branch condition
// Loads use imm_I as well, so we can reuse the adder: rd <- mem[rs1+imm_I]

// First input is always reg[rs1]
logic [31:0] arith_in1 = registers[instr_rs1];

// For branches and R type arithmetic instructions, the second ALU input is reg[rs2], for ALUimm, Load or JALR its I type IMM value
// We use this for the branch predicate evaluation as well, and JALR.
logic [31:0] arith_in2 = (isArithR | isBranch) ? registers[instr_rs2] : imm_I;

// Adder used for arithmetic operations, memory
logic [31:0] arith_add = arith_in1 + arith_in2;

// Subtraction used for both branch condition checking, and as ALU result for sub/subi
logic arith_C;
logic [31:0] arith_sub;
assign {arith_C, arith_sub} = arith_in1 - arith_in2;
logic arith_Z = (arith_sub == 32'h0) ? 1'b1 : 1'b0;
logic arith_V = (arith_in1[31] & !arith_in2[31] & !arith_sub[31]) | (!arith_in1[31] & arith_in2[31] & arith_sub[31]);
logic arith_N = arith_sub[31];
logic arith_S = arith_N ^ arith_V;

// Branch conditions, retrieved from subtraction result
logic cond_beq = arith_Z;
logic cond_bne = ~arith_Z;
logic cond_blt = arith_S;
logic cond_bge = ~arith_S;
logic cond_bltu = arith_C;
logic cond_bgeu = ~arith_C;

// ALU output. This is only used if isArith is true.
logic [31:0] arith_result;
logic [31:0] arith_right_shift = (arith_in1 >> (is_arith_imm ? { 27'h0, instr_shamt } : arith_in2));

// ALU output calculation
always_comb begin
    arith_result = 32'h0;
    
    // FUNCT3 field of instruction describes which operation to perform
    case (instr_func3)
        FUNCT3_ARITH_ADDSUB: begin
            // For register arithmetic instruction, funct7 decides whether its addition or subtraction
            // For immediate arithmetic instructions, no subtraction instruction is provided
            if (is_arith_reg && (instr_func7 == FUNCT7_ALT)) begin
                arith_result = arith_sub;
            end
            else begin
                arith_result = arith_add;
            end
        end
        FUNCT3_ARITH_AND: begin
            arith_result = arith_in1 & arith_in2;
        end
        FUNCT3_ARITH_OR: begin
            arith_result = arith_in1 | arith_in2;
        end
        FUNCT3_ARITH_XOR: begin
            arith_result = arith_in1 ^ arith_in2;
        end
        FUNCT3_ARITH_SLT: begin
            arith_result = ($signed(arith_in1) < $signed(arith_in2)) ? 32'h1 : 32'h0;
        end
        FUNCT3_ARITH_SLTU: begin
            arith_result = ($unsigned(arith_in1) < $unsigned(arith_in2)) ? 32'h1 : 32'h0;
        end
        FUNCT3_ARITH_SHIFTL: begin
            // For imm arithmetic left shifts, we need to use the shamt field in the instruction
            arith_result = arith_in1 << (is_arith_imm ? { 27'h0, instr_shamt } : arith_in2);
        end
        FUNCT3_ARITH_SHIFTR: begin
            // FUNCT7_ALT implies arithmetic right shift
            if (instr_func7 == FUNCT7_ALT) begin
                // As for left shifts, imm arithmetic right shift uses shamt field of instruction
                arith_result = { (arith_in1[31] ? 1'b1 : 1'b0), arith_right_shift[30:0] };
            end
            else begin
                arith_result = arith_right_shift;
            end
        end
    endcase
end

// ==== Writeback value ====
logic [31:0] writeback_value;

always_comb begin
    writeback_value = 32'h0;

    case (1'b1)
        (is_arith_imm | is_arith_reg): begin
            writeback_value = arith_result;
        end
        (is_lui): begin
            writeback_value = imm_U;
        end
        (is_auipc): begin
            writeback_value = pc + imm_U;
        end
        (is_jal | is_jalr): begin
            writeback_value = pc + 32'h4;
        end
    endcase
end

// ==== Program counter handling ====
logic [31:0] pc;

initial begin
    pc = 32'h0;
end

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        pc <= 32'h0;
    end
    else begin
        // XXX PC logic
    end
end



endmodule