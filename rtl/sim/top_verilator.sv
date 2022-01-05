module top_verilator(
    input logic clk_i,
    input logic reset_i
);

logic [3:0] leds;

soc soc(
    .clk_in(clk_i),
    .reset_in(reset_i),
    .leds_out(leds)
);

endmodule