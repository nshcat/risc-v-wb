// XXX Different counting modes to implement center aligned pwm etc

module timer#(
    parameter [31:0] BaseAddr = 32'h4020
)(
    input logic clk_in,
    input logic reset_in,

    output logic timer_irq_out,
    output logic [2:0] compare_out,

    wb_bus.slave bus_slave
);

// Register addresses defined by this peripheral
localparam [31:0] REG_CONTROL_ADDR = 32'(BaseAddr) + 32'h0;
localparam [31:0] REG_PRETH_ADDR = 32'(BaseAddr) + 32'h4;
localparam [31:0] REG_CNTTH_ADDR = 32'(BaseAddr) + 32'h8;
localparam [31:0] REG_CMPV1_ADDR = 32'(BaseAddr) + 32'hC;
localparam [31:0] REG_CMPV2_ADDR = 32'(BaseAddr) + 32'h10;
localparam [31:0] REG_CMPV3_ADDR = 32'(BaseAddr) + 32'h14;

localparam CONTROL_TIMEN_BIT = 0;
localparam CONTROL_IRQEN_BIT = 1;
localparam CONTROL_CMPO1EN_BIT = 2;
localparam CONTROL_CMPO2EN_BIT = 3;
localparam CONTROL_CMPO3EN_BIT = 4;

wire enable_cmp1 = control[CONTROL_CMPO1EN_BIT];
wire enable_cmp2 = control[CONTROL_CMPO2EN_BIT];
wire enable_cmp3 = control[CONTROL_CMPO3EN_BIT];
wire enable_timer = control[CONTROL_TIMEN_BIT];
wire enable_irq = control[CONTROL_IRQEN_BIT];

// Bus write handling
wire [31:0] word_wmask = {
    (bus_slave.sel[3] ? 8'hFF : 8'h0),
    (bus_slave.sel[2] ? 8'hFF : 8'h0),
    (bus_slave.sel[1] ? 8'hFF : 8'h0),
    (bus_slave.sel[0] ? 8'hFF : 8'h0)
};

wire [31:0] masked_wdata = bus_slave.wdata & word_wmask;

// Timer control register
logic [4:0] control;
logic [31:0] prescale_threshold;
logic [31:0] counter_threshold;
logic [31:0] cmp_value_1;
logic [31:0] cmp_value_2;
logic [31:0] cmp_value_3;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        control <= 5'h0;
        prescale_threshold <= 32'h0;
        counter_threshold <= 32'h0;
        cmp_value_1 <= 32'h0;
        cmp_value_2 <= 32'h0;
        cmp_value_3 <= 32'h0;
    end
    else begin
        if (bus_slave.stb & bus_slave.cyc & bus_slave.we) begin
            case (bus_slave.addr)
                REG_CONTROL_ADDR: control <= (control & ~word_wmask[4:0]) | (bus_slave.wdata[4:0] & word_wmask[4:0]);
                REG_PRETH_ADDR: prescale_threshold <= (prescale_threshold & ~word_wmask) | masked_wdata;
                REG_CNTTH_ADDR: counter_threshold <= (counter_threshold & ~word_wmask) | masked_wdata;
                REG_CMPV1_ADDR: cmp_value_1 <= (cmp_value_1 & ~word_wmask) | masked_wdata;
                REG_CMPV2_ADDR: cmp_value_2 <= (cmp_value_2 & ~word_wmask) | masked_wdata;
                default: cmp_value_3 <= (cmp_value_3 & ~word_wmask) | masked_wdata;
            endcase
        end
    end
end

wire config_changed = (bus_slave.stb & bus_slave.cyc & bus_slave.we) &&
    ((bus_slave.addr == REG_PRETH_ADDR) || (bus_slave.addr == REG_CNTTH_ADDR));

// Bus read data handling
logic [31:0] rdata;
always_comb begin
    rdata = 32'h0;

    if (bus_slave.stb & bus_slave.cyc & ~bus_slave.we) begin
        case (bus_slave.addr)
            REG_CONTROL_ADDR: rdata = { 27'h0, control };
            default: rdata = 32'h0;
        endcase
    end
end

// Timer logic
logic [31:0] prescaler;
logic [31:0] counter;
logic interrupt;

always_ff @(posedge clk_in) begin
    // Reset counting operation if configuration changed
    if (~reset_in | config_changed) begin
        prescaler <= 32'h0;
        counter <= 32'h0;
        interrupt <= 1'b0;
    end
    else if(enable_timer) begin
        if (prescaler == prescale_threshold) begin
            prescaler <= 32'h0;
            if (counter == counter_threshold) begin
                counter <= 32'h0;
                interrupt <= 1'b1;
            end
            else begin
                counter <= counter + 32'h1;
                interrupt <= 1'b0;
            end
        end
        else begin
            prescaler <= prescaler + 32'h1;
            interrupt <= 1'b0;
        end
    end
    else begin
        // Make sure interrupt line is definitely turned off even if timer is disabled
        interrupt <= 1'b0;
    end
end

// Comparator output
assign compare_out[0] = (counter < cmp_value_1) & enable_cmp1;
assign compare_out[1] = (counter < cmp_value_2) & enable_cmp2;
assign compare_out[2] = (counter < cmp_value_3) & enable_cmp3;

// Interrupt generation
assign timer_irq_out = enable_irq & interrupt;

// Bus connection
assign bus_slave.rdata = rdata;
assign bus_slave.ack = (bus_slave.cyc & bus_slave.stb);
assign bus_slave.err = 1'b0;

endmodule