module wb_endian_adapter(
    wb_bus.master bus_master,   // Bus from component to adapt
    wb_bus.slave bus_slave      // To master using this component
);

function [31:0] endian_swap_word(input [31:0] word);
    endian_swap_word = {
        word[ 7: 0],
        word[15: 8],
        word[23:16],
        word[31:24]
    };
endfunction

// Component -> Bus
assign bus_slave.ack = bus_master.ack;
assign bus_slave.err = bus_master.err;
assign bus_slave.rdata = endian_swap_word(bus_master.rdata);

// Bus -> Component
assign bus_master.addr = bus_slave.addr;
assign bus_master.wdata = bus_slave.wdata;
assign bus_master.we = bus_slave.we;
assign bus_master.stb = bus_slave.stb;
assign bus_master.cyc = bus_slave.cyc;
assign bus_master.sel = { bus_slave.sel[0], bus_slave.sel[1], bus_slave.sel[2], bus_slave.sel[3] };

endmodule