module testbench;
timeunit 1ns;
timeprecision 100ps;

logic clk, reset;
logic [3:0] leds;

parameter PERIOD = 10;

initial begin
    reset = 1'b1;
    clk = 1'b0;
    #5 clk = 1'b1;
    #5 reset = 1'b0;
end

always #PERIOD clk=~clk;

top dut(
    .sys_clk_in(clk),
    .reset_btn_in(reset),
    .leds_out(leds)
);

endmodule