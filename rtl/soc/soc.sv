module soc(
    input logic clk_in,
    input logic reset_in,

    input logic test_irq_in,

    output logic [3:0] leds_out
);

// Connecting wires
logic ext_irq;
logic ext_irq_clr;

// Main bus
wb_bus core_bus();

// All Interconnect<->Peripheral busses
wb_bus peripheral_busses [4]();

// CPU Core and main interconnect
core cpu(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .ext_irq_in(ext_irq),
    .ext_irq_clr_in(ext_irq_clr),
    .bus_master(core_bus)
);

wb_interconnect #(
    .N(4),
    .AddrRanges({
        32'h0,      32'h2FFC,           // Program memory
        32'h3000,   32'h3FFC,           // Data memory
        32'h4000,   32'h4000,           // LED controller
        32'h4010,   32'h401C            // External IRQ controller
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

data_memory dmem(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[1])
);

leds led(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[2]),
    .leds_out(leds_out)
);

external_irq_controller eic(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[3]),
    .ext_irq_out(ext_irq),
    .ext_irq_clr_out(ext_irq_clr),
    .irq_lines_in({15'h0, test_irq_in})
);

endmodule
