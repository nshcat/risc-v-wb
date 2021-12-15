`include "core_defines.vh"

module alu(
    input word_t a_in,
    input word_t b_in,
    input alu_op_e op_in,

    output word_t result_out,
    output alu_flags_t flags_out
);

logic [31:0] shr = a_in >> b_in;

always_comb begin
    flags_out.C = 1'b0;
    flags_out.V = 1'b0;

    case (op_in)
        ALU_OP_ADD: begin
            {flags_out.C, result_out} = a_in + b_in;
            flags_out.V = (a_in[31] & b_in[31] & !result_out[31]) | (!a_in[31] & !b_in[31] & result_out[31]);
        end

        ALU_OP_SUB: begin
            {flags_out.C, result_out} = a_in - b_in;
            flags_out.V = (a_in[31] & !b_in[31] & !result_out[31]) | (!a_in[31] & b_in[31] & result_out[31]);
        end

        ALU_OP_SLL: begin
            result_out = a_in << b_in;
        end

        ALU_OP_SLT: begin
            result_out = ($signed(a_in) <  $signed(b_in)) ? 32'b1 : 32'b0;
        end

        ALU_OP_SLTU: begin
            result_out = ($unsigned(a_in) <  $unsigned(b_in)) ? 32'b1 : 32'b0;
        end

        ALU_OP_XOR: begin
            result_out = a_in ^ b_in;
        end

        ALU_OP_SRL: begin
            result_out = a_in >> b_in;
        end

        ALU_OP_SRA: begin
            result_out = { (a_in[31] ? 1'b1: 1'b0), shr[30:0] };
        end

        ALU_OP_OR: begin
            result_out = a_in | b_in;
        end

        default: begin
            result_out = a_in & b_in;
        end
    endcase
end

assign flags_out.Z = (result_out == 32'b0) ? 1'b1 : 1'b0;
assign flags_out.N = result_out[31];
assign flags_out.S = flags_out.N ^ flags_out.V;

endmodule