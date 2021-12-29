module program_memory #(
    parameter [31:0] BaseAddr = 32'h0
)(
    input logic clk_in,
    input logic reset_in,

    wb_bus.slave bus_slave
);

// Error logic. We dont support writes or misaligned reads
wire err = (bus_slave.we || bus_slave.addr[1:0] != 2'h0);

logic [31:0] memory [1023:0];

initial begin
    for (int i = 0; i < 1024; i++)
        memory[i] = 32'h93821200;
end


// Reading
logic [31:0] rdata;
initial begin
    rdata = 32'h0;
end

wire [31:0] local_addr = bus_slave.addr - BaseAddr;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        rdata <= 32'h0;
    end
    else if (bus_slave.stb) begin
        rdata <= memory[local_addr];
    end
end

// Ack
assign bus_slave.ack = bus_slave.stb & ~err;

assign bus_slave.rdata = rdata;
assign bus_slave.err = err;

endmodule