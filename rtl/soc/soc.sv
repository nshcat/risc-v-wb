module soc(
    input logic clk_in,
    input logic reset_in
);

// Main bus
wb_bus core_bus();

// All Interconnect<->Peripheral busses
wb_bus peripheral_busses [1]();

// CPU Core and main interconnect
core cpu(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_master(core_bus)
);

wb_interconnect #(
    .N(1),
    .AddrRanges({
        32'h0, 32'h2FFF      // Program memory
    })
) inter(
    .bus_in(core_bus),
    .bus_out(peripheral_busses)
);

// Peripherals
program_memory pmem(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[0])
);

endmodule
