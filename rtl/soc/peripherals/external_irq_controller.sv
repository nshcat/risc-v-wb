module external_irq_controller#(
    parameter [31:0] BaseAddr = 32'h4010
)(
    input logic clk_in,
    input logic reset_in,

    input logic [15:0] irq_lines_in,
    output logic ext_irq_out,
    output logic ext_irq_clr_out,

    wb_bus.slave bus_slave
);

localparam [31:0] REG_MASK_ADDR = 32'(BaseAddr) + 32'h0;
localparam [31:0] REG_CIP_ADDR = 32'(BaseAddr) + 32'h4;
localparam [31:0] REG_IRQNUM_ADDR = 32'(BaseAddr) + 32'h8;
localparam [31:0] REG_IRQBIT_ADDR = 32'(BaseAddr) + 32'hC;

// Pending requests, sampled each clock cycle from input irq lines
// Reset by writing bits to pseudo register EIC_CLEAR_IP
logic [15:0] pending_requests;

// IRQ mask, pending requests can only fire interrupt if they are enabled here
logic [15:0] mask;

// Whether we are currently firing an interupt. Is reset if IP flag corresponding
// to current interrupt is reset.
logic interrupt_active;

// Currently active interrupt, both as number and bit
logic [3:0] active_interrupt;
logic [15:0] active_interrupt_bit;

wire is_write = (bus_slave.cyc & bus_slave.stb & bus_slave.we);
wire is_read = (bus_slave.cyc & bus_slave.stb & ~bus_slave.we);
// Only lower 16 bits are accesible
wire is_valid_write = (bus_slave.sel[1:0] != 2'h0)
    && (bus_slave.addr != REG_IRQBIT_ADDR)
    && (bus_slave.addr != REG_IRQNUM_ADDR);

wire err = bus_slave.cyc & bus_slave.stb & ~(is_read | is_valid_write);

wire [15:0] word_wmask = {
    (bus_slave.sel[1] ? 8'hFF : 8'h0),
    (bus_slave.sel[0] ? 8'hFF : 8'h0)
};

// Bus read result data
logic [31:0] rdata;
always_comb begin
    rdata = 32'h0;

    if (is_read) begin
        case (bus_slave.addr)
            REG_MASK_ADDR: rdata = { 16'h0, mask };
            REG_IRQBIT_ADDR: rdata = { 16'h0, active_interrupt_bit };
            REG_IRQNUM_ADDR: rdata = { 28'h0, active_interrupt };
            default: rdata = 32'h0;
        endcase
    end
end

wire [15:0] next_pending = (is_write && (bus_slave.addr == REG_CIP_ADDR))
    ? (pending_requests & ~(bus_slave.wdata[15:0] & word_wmask)) | irq_lines_in
    : (pending_requests | irq_lines_in);

logic [15:0] next_interrupt_bit;
logic [3:0] next_interrupt;

always_comb begin
    casez (next_pending & mask)
        16'b???????????????1: next_interrupt_bit = 16'b1;
        16'b??????????????10: next_interrupt_bit = 16'b10;
        16'b?????????????100: next_interrupt_bit = 16'b100;
        16'b????????????1000: next_interrupt_bit = 16'b1000;
        16'b???????????10000: next_interrupt_bit = 16'b10000;
        16'b??????????100000: next_interrupt_bit = 16'b100000;
        16'b?????????1000000: next_interrupt_bit = 16'b1000000;
        16'b????????10000000: next_interrupt_bit = 16'b10000000;
        16'b???????100000000: next_interrupt_bit = 16'b100000000;
        16'b??????1000000000: next_interrupt_bit = 16'b1000000000;
        16'b?????10000000000: next_interrupt_bit = 16'b10000000000;
        16'b????100000000000: next_interrupt_bit = 16'b100000000000;
        16'b???1000000000000: next_interrupt_bit = 16'b1000000000000;
        16'b??10000000000000: next_interrupt_bit = 16'b10000000000000;
        16'b?100000000000000: next_interrupt_bit = 16'b100000000000000;
        16'b1000000000000000: next_interrupt_bit = 16'b1000000000000000;
        default: next_interrupt_bit = 16'h0;
    endcase

    next_interrupt = 4'($clog2(next_interrupt_bit));
end

// Check if interrupt was just cleared
wire interrupt_cleared = (interrupt_active 
    && (is_write && (bus_slave.addr == REG_CIP_ADDR))
    && ((active_interrupt_bit & (bus_slave.wdata[15:0] & word_wmask)) != 16'h0));

// Pending interrupt bus access handling and IRQ line sampling
always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        pending_requests <= 16'h0;
        interrupt_active <= 1'b0;
        active_interrupt <= 4'h0;
        active_interrupt_bit <= 16'h0;
    end
    else begin
        pending_requests <= next_pending;

        // Check if theres any more interrupts to be fired
        if ((~interrupt_active || interrupt_cleared) && next_interrupt_bit != 16'h0) begin
            interrupt_active <= 1'b1;
            active_interrupt <= next_interrupt;
            active_interrupt_bit <= next_interrupt_bit;
        end
        else if (interrupt_cleared) begin
            interrupt_active <= 1'b0;
            active_interrupt <= 4'h0;
            active_interrupt_bit <= 16'h0;
        end
    end
end

// Mask bus access
always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        mask <= 16'h0;
    end
    else begin
        if (is_write && (bus_slave.addr == REG_MASK_ADDR)) begin
            mask <= (mask & ~word_wmask) | (bus_slave.wdata[15:0] & word_wmask);
        end
    end
end

assign bus_slave.err = err;
assign bus_slave.rdata = rdata;
assign bus_slave.ack = bus_slave.cyc & bus_slave.stb & ~err;

assign ext_irq_out = (active_interrupt_bit != 16'h0);
assign ext_irq_clr_out = (active_interrupt_bit == 16'h0);

endmodule