`include "core_defines.vh"

module register_file(
    input logic clk_i,
    input logic reset_i,

    input logic [4:0] read_register_1_i,
    input logic [4:0] read_register_2_i,
    input logic [4:0] write_register_i,
    input word_t write_back_data_i,
    input logic ctrl_write_back_i,

    output word_t register_data_1_o,
    output word_t register_data_2_o
);

`ifndef FEATURE_RV32E
    // 32 registers. Note that $0 is hard-wired to always be 0.
    word_t registers [31:0];
    `define NUM_REGISTERS 32
    `define REG_ADDR_BITS 5
`else
    // 16 registers. $0 is hard-wired to always be 0.
    word_t registers [15:0];
    `define NUM_REGISTERS 16
    `define REG_ADDR_BITS 4
`endif

initial begin
    integer i;
    for (i = 0; i < `NUM_REGISTERS; i = i + 1)
        registers[i] = 32'h0;
end

// Register write back
always_ff @(posedge clk_i) begin
    if (!reset_i) begin
        integer i;
        for (i = 0; i < `NUM_REGISTERS; i = i + 1)
            registers[i] <= 32'h0;
    end
    else begin
        if(ctrl_write_back_i) begin
            registers[write_register_i[`REG_ADDR_BITS-1:0]] <= write_back_data_i;
        end
    end
end

// Register read
logic needs_forward_read_1;
logic needs_forward_read_2;

assign needs_forward_read_1 = (write_register_i == read_register_1_i) & ctrl_write_back_i;
assign needs_forward_read_2 = (write_register_i == read_register_2_i) & ctrl_write_back_i;

always_comb begin
    register_data_1_o = needs_forward_read_1 ? write_back_data_i : registers[read_register_1_i[`REG_ADDR_BITS-1:0]];
    register_data_2_o = needs_forward_read_2 ? write_back_data_i : registers[read_register_2_i[`REG_ADDR_BITS-1:0]];
end

endmodule