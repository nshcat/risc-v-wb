// Topmost module for Arty A7 board
module top(
    input logic sys_clk_in,

    input logic reset_btn_in,   // Gets asserted when pushed

    output logic led0_r_out,
    output logic led0_g_out,
    output logic led0_b_out,

    output logic [3:0] leds_out
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
    .leds_out(leds_out)
);

// Debug stuff
logic [2:0] led_state;
initial led_state = 3'h00;
assign { led0_r_out, led0_g_out, led0_b_out } = led_state;

logic [23:0] counter;
initial begin
    counter = 24'h0;
end

always_ff @(posedge slow_clk) begin
    if(~reset) begin
        counter <= 24'h0;
        led_state <= 3'h0;
    end
    else begin
        if(counter == 24'd10000000) begin
            counter <= 24'h0;
            led_state <= led_state + 3'h1;
        end
        else begin
            counter <= counter + 24'h1;
        end
    end
end

endmodule