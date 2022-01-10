module soc(
    input logic clk_in,
    input logic reset_in,

    output logic [3:0] leds_out,

    output logic [2:0] rgb1_out,
    output logic [2:0] rgb2_out
);

// Connecting wires
logic ext_irq;
logic ext_irq_clr;
logic tim1_irq;
logic tim2_irq;

// Main bus
wb_bus core_bus();

// All Interconnect<->Peripheral busses
wb_bus peripheral_busses [6]();

// CPU Core and main interconnect
core cpu(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .ext_irq_in(ext_irq),
    .ext_irq_clr_in(ext_irq_clr),
    .bus_master(core_bus)
);

wb_interconnect #(
    .N(6),
    .AddrRanges({
        32'h0,      32'h2FFC,           // Program memory
        32'h3000,   32'h3FFC,           // Data memory
        32'h4000,   32'h4000,           // LED controller
        32'h4010,   32'h401C,           // External IRQ controller
        32'h4020,   32'h4034,           // Timer 1
        32'h4038,   32'h404C            // Timer 2
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
    .irq_lines_in({14'h0, tim2_irq, tim1_irq})
);

timer #(
    .BaseAddr(32'h4020)
) timer1(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[4]),
    .timer_irq_out(tim1_irq),
    .compare_out(rgb1_out)
);

timer #(
    .BaseAddr(32'h4038)
) timer2(
    .clk_in(clk_in),
    .reset_in(reset_in),
    .bus_slave(peripheral_busses[5]),
    .timer_irq_out(tim2_irq),
    .compare_out(rgb2_out)
);

endmodule
