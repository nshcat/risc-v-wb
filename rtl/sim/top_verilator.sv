module top_verilator(
    input logic clk_i,
    input logic reset_i
);

soc soc(
    .clk_in(clk_i),
    .reset_in(reset_i)
);

/*core cpu(
    .clk_in(clk_i),
    .reset_in(reset_i)
);

wb_bus bus_in;
wb_bus bus_out[2];

assign bus_in.addr = 32'hAA;

wb_interconnect #(
    .N(2),
    .AddrRanges({32'h0, 32'hFF, 32'h100, 32'h1FF})
) inter (
    .bus_in(bus_in),
    .bus_out(bus_out)
);*/

endmodule