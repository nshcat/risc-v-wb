module top_verilator(
    input logic clk_i,
    input logic reset_i
);

core cpu(
    .clk_in(clk_i),
    .reset_in(reset_i)
);

endmodule;