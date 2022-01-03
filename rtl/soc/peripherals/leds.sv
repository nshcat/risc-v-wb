module leds #(
    parameter [31:0] BaseAddr = 32'h4000
)(
    input logic clk_in,
    input logic reset_in,

    wb_bus.slave bus_slave,

    output logic [3:0] leds_out
);

logic [3:0] led_state;

wire addr_valid = (bus_slave.addr - BaseAddr) == 32'h0;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        led_state <= 4'h0;
    end
    else if (addr_valid & bus_slave.we & bus_slave.sel[0]) begin
        led_state <= bus_slave.wdata[3:0];
    end
end

assign bus_slave.rdata = { 28'h0, led_state };
assign bus_slave.ack = bus_slave.cyc & bus_slave.stb;
assign bus_slave.err = ~addr_valid & bus_slave.cyc & bus_slave.stb;

assign leds_out = led_state;

endmodule