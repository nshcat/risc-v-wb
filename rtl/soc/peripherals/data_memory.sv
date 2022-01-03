module data_memory #(
    parameter [31:0] BaseAddr = 32'h3000
)(
    input logic clk_in,
    input logic reset_in,

    wb_bus.slave bus_slave
);

typedef enum logic [1:0]
{ 
    STATE_WAIT = 2'b00,
    STATE_BUSY = 2'b01
} state_t;

// Error logic. We dont support misaligned reads or writes
wire err = (bus_slave.addr[1:0] != 2'h0);

// 4kb of RAM
logic [31:0] memory [0:1023];
initial begin
    for (int i = 0; i < 1024; i++)
        memory[i] = 32'h0;
end

wire [31:0] absolute_addr = (bus_slave.addr - BaseAddr);
wire [9:0] word_addr = absolute_addr[11:2];    // Word-based address

wire [31:0] word_wmask = {
    (bus_slave.sel[3] ? 8'hFF : 8'h0),
    (bus_slave.sel[2] ? 8'hFF : 8'h0),
    (bus_slave.sel[1] ? 8'hFF : 8'h0),
    (bus_slave.sel[0] ? 8'hFF : 8'h0)
};

logic [31:0] rdata;
logic ack;
state_t state;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        rdata <= 32'h0;
        ack <= 1'h0;
        state <= STATE_WAIT;
    end
    else begin
        case (state)
            STATE_WAIT: begin
                if (bus_slave.stb & bus_slave.cyc & ~err) begin
                    ack <= 1'h1;
                    state <= STATE_BUSY;

                    // We need to perform a read for both loads and stores
                    rdata <= memory[word_addr];
                end
            end
            /*STATE_BUSY*/
            default: begin
                // Transaction complete, deassert ack
                ack <= 1'h0;
                state <= STATE_WAIT;

                // If this was a store operation, commit it
                if (bus_slave.we) begin
                    memory[word_addr] <= (rdata & ~word_wmask) | (bus_slave.wdata & word_wmask);
                end
            end
        endcase
    end
end

// Bus connections
assign bus_slave.ack = ack;
assign bus_slave.err = err;
assign bus_slave.rdata = rdata;

endmodule