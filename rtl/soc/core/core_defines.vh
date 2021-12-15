`ifndef _CORE_DEFINES_H
`define _CORE_DEFINES_H

// ======== Base data types ======== 
`define WORD_SIZE 32
typedef logic [`WORD_SIZE-1:0] word_t;


// ======== ALU types ========

// ALU operation mode enum
typedef enum logic[3:0]
{
    ALU_OP_ADD          = 4'b0000,
    ALU_OP_SUB          = 4'b0001,
    ALU_OP_SLL          = 4'b0010,
    ALU_OP_SLT          = 4'b0011,
    ALU_OP_SLTU         = 4'b0100,
    ALU_OP_XOR          = 4'b0101,
    ALU_OP_SRL          = 4'b0110,
    ALU_OP_SRA          = 4'b0111,
    ALU_OP_OR           = 4'b1000,
    ALU_OP_AND          = 4'b1001
} alu_op_e;

// ALU result flags
typedef struct packed
{
    logic Z;
    logic N;
    logic S;
    logic C;
    logic V;
} alu_flags_t;

`endif