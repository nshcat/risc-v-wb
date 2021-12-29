module soc(
    input logic clk_in,
    input logic reset_in
);

// CPU Core
core cpu(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_master(core_bus)
);

// Main bus and interconnect
wb_bus core_bus();
wb_interconnect #(
    .N(1),
    .AddrRanges({
        32'h0, 32'h400      // Program memory
    })
) inter(
    .bus_in(core_bus),
    .bus_out(peripheral_busses)
);

// Peripherals

// All Interconnect<->Peripheral busses
wb_bus peripheral_busses [0:0];

program_memory pmem(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[0])
);

endmodule
