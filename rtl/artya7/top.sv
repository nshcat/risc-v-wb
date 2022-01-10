// Topmost module for Arty A7 board
module top(
    input logic sys_clk_in,

    input logic reset_btn_in,   // Gets asserted when pushed

    output logic [3:0] leds_out,

    output logic [2:0] rgb1_out,
    output logic [2:0] rgb2_out
);

// Clock generation
logic slow_clk;
logic locked;

pll clockgen(
    .fast_clk_in(sys_clk_in),
    .slow_clk_out(slow_clk),
    .locked_out(locked)
);

// Reset signal generation
logic reset_sync;
synchronizer reset_synchronizer(
    .clk_in(slow_clk),
    .signal_in(reset_btn_in),
    .signal_out(reset_sync)
);

logic reset_debounced;
logic reset_falling;
logic reset_rising; // This is asserted for one clock cycle when button is pressed
debouncer reset_debouncer(
    .clk_in(slow_clk),
    .signal_in(reset_sync),
    .signal_out(reset_debounced),
    .is_falling_out(reset_falling),
    .is_rising_out(reset_rising)
);

// The main reset signal
logic reset = ~reset_rising & locked;

// SoC instance
soc soc(
    .clk_in(slow_clk),
    .reset_in(reset),
    .leds_out(leds_out),
    .rgb1_out(rgb1_out),
    .rgb2_out(rgb2_out)
);

endmodule