// Combinational 1:N wishbone bus interconnect
module wb_interconnect #(
    parameter N = 2,
    parameter [(N*2*32)-1:0] AddrRanges
)(
    wb_bus.slave bus_in,
    wb_bus.master bus_out [N-1:0]
);


logic [31:0] addr_begin [N-1:0];
logic [31:0] addr_end [N-1:0];
genvar i;
for (i = 0; i < N; i++) begin
    assign addr_begin[i] = AddrRanges[((32*(i*2+1)+32)-1):((32*(i*2)+32))];
    assign addr_end[i] = AddrRanges[(32*(i*2+1)-1):(32*(i*2))];
end

logic select [N-1:0];
genvar j;
for (j = 0; j < N; j++) begin
    assign select[j] = (bus_in.addr >= addr_begin[j] && bus_in.addr <= addr_end[j]);
end

// Address is valid if it belongs to one of the slaves of this interconnect.
logic addr_valid;
always_comb begin
    addr_valid = 1'b0;
    for(int i = 0; i < N; i++)
        addr_valid |= select[i];
end

// This is a workaround for verilator, since it cant directly work with interfaces inside for loops
logic [31:0] read_data_i [N-1:0];
logic ack_i [N-1:0];
logic err_i [N-1:0];
for (genvar i = 0; i < N; i++) begin
    always_comb begin
        read_data_i[i] = bus_out[i].rdata;
        ack_i[i] = bus_out[i].ack;
        err_i[i] = bus_out[i].err;
    end
end



// Bus errors are either generated if the requested address doesnt correspond to any
// of the slaves connected to the interconnect, or if any of the slaves signalled an error.
logic bus_error;
always_comb begin
    bus_error = ~addr_valid;

    for(int i = 0; i < N; i++)
        bus_error |= (select[i] & err_i[i]);
end


// Read data coming from the slaves, and feedback control signals
logic [31:0] read_data;
logic ack;
always_comb begin
    read_data = 32'h0;
    ack = 1'h0;

    for(int i = 0; i < N; i++) begin
        if (select[i]) begin
            read_data = read_data_i[i];
            ack = ack_i[i];
            break;
        end
    end
end

// Connection of slave->master signals
assign bus_in.err = bus_error;
assign bus_in.ack = ack;
assign bus_in.rdata = read_data;

// Distribution of master->slave signals
genvar k;
for (k = 0; k < N; k++) begin
    assign bus_out[k].we = select[k] ? bus_in.we : 1'h0;
    assign bus_out[k].stb = select[k] ? bus_in.stb : 1'h0;
    assign bus_out[k].cyc = select[k] ? bus_in.cyc : 1'h0;
    assign bus_out[k].wdata = select[k] ? bus_in.wdata : 32'h0;
    assign bus_out[k].sel = select[k] ? bus_in.sel : 4'h0;
    assign bus_out[k].addr = select[k] ? bus_in.addr : 32'h0;
end

endmodule