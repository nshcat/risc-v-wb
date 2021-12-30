module program_memory #(
    parameter [31:0] BaseAddr = 32'h0
)(
    input logic clk_in,
    input logic reset_in,

    wb_bus.slave bus_slave
);

// Error logic. We dont support writes or misaligned reads
wire err = (bus_slave.we || bus_slave.addr[1:0] != 2'h0);

// 12 kb of program flash
logic [31:0] memory [0:3071];

`ifdef VERILATOR
initial $readmemh("flash.txt", memory);
`else
initial $readmemh("./../../memory/flash.txt", memory);
`endif

// Reading
logic [31:0] rdata;
initial begin
    rdata = 32'h0;
end

wire [31:0] absolute_addr = (bus_slave.addr - BaseAddr);
wire [11:0] word_addr = absolute_addr[13:2];    // Word-based address

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        rdata <= 32'h0;
    end
    else if (bus_slave.stb) begin
        rdata <= memory[word_addr];
    end
end

// Ack
logic ack;
initial begin
    ack = 1'h0;
end

always_ff @(posedge(clk_in)) begin
    if (~reset_in) begin
        ack <= 1'h0;
    end
    else begin
        ack <= bus_slave.stb & ~err;
    end
end

assign bus_slave.ack = ack & bus_slave.stb & ~err;
assign bus_slave.rdata = rdata;
assign bus_slave.err = err;

endmodule