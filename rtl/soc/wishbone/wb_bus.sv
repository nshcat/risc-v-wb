interface wb_bus();

logic [31:0] addr;
logic [31:0] rdata;
logic [31:0] wdata;
logic [3:0] sel;
logic we;
logic stb;
logic ack;
logic cyc;
logic err;

modport master(
    input rdata, ack, err,
    output addr, wdata, sel, we, stb, cyc
);

modport slave(
    input addr, wdata, sel, we, stb, cyc,
    output rdata, ack, err
);

endinterface