`include "core_defines.vh"

module top_verilator(
    input logic clk_i,
    input logic reset_i
);

logic [31:0] a;
logic [31:0] b;
logic [31:0] result;

/* verilator lint_off UNOPTFLAT */
alu_flags_t flags;
alu_op_e operation;

alu uut(
    .a_in(a),
    .b_in(b),
    .result_out(result),
    .op_in(operation),
    .flags_out(flags)
);

always_ff @(posedge clk_i) begin
    if(!reset_i) begin
        a <= 32'd1200;
        b <= 32'd0;
    end
    else begin
        b <= b + 32'd1;

        if ((b % 2) == 0) begin
            operation <= ALU_OP_ADD;
        end
        else begin
            operation <= ALU_OP_SUB;
        end
    end
end

endmodule;