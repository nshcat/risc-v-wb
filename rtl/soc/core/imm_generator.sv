`include "core_defines.vh"
`include "control_signals.vh"

module imm_generator(
    input word_t instruction_i,
    input control_imm_source_e ctrl_imm_src_i,
    output word_t imm_o
);

logic sign;
assign sign = instruction_i[31];

always_comb begin
    case(ctrl_imm_src_i)
        CONTROL_IMM_SRC_I_TYPE: begin
            imm_o = { (sign ? 21'h1FFFFF : 21'h0), instruction_i[30:20] };
        end

        CONTROL_IMM_SRC_S_TYPE: begin
            imm_o = { (sign ? 21'h1FFFFF : 21'h0), instruction_i[30:25], instruction_i[11:8], instruction_i[7] };
        end

        CONTROL_IMM_SRC_B_TYPE: begin
            imm_o = { (sign ? 20'hFFFFF : 20'h0), instruction_i[7], instruction_i[30:25], instruction_i[11:8], 1'b0 };
        end

        CONTROL_IMM_SRC_U_TYPE: begin
            imm_o = { instruction_i[31:12], 12'h0 };
        end
        
        CONTROL_IMM_SRC_J_TYPE: begin
            imm_o = { (sign ? 12'hFFF : 12'h0), instruction_i[19:12], instruction_i[20], instruction_i[30:25], instruction_i[24:21], 1'b0 };
        end
    endcase
end

endmodule