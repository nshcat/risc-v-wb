// Debouncer, used mainly for the reset button
// The input signal is required to be already synchronized to the input clock
module debouncer #(
    parameter MAX_COUNT = 16
)
(
    input logic clk_in,
    input logic signal_in,
    output logic signal_out,
    output logic is_falling_out,        // Is asserted for one clock cycle if falling edge was detected
    output logic is_rising_out          // Is asserted for one clock cycle if rising edge was detected
);

localparam COUNTER_BITS = $clog2(MAX_COUNT);

logic [COUNTER_BITS-1:0] counter;
logic internal_falling;
logic internal_rising;

initial begin
    counter = {(COUNTER_BITS){1'b0}};
    signal_out = 1'b0;
end

always_ff @(posedge clk_in) begin
    counter <= {(COUNTER_BITS){1'b0}};
    is_falling_out <= 1'b0;
    is_rising_out <= 1'b0;

    if (counter == MAX_COUNT - 1) begin
        signal_out <= signal_in;
        is_rising_out <= internal_rising;
        is_falling_out <= internal_falling;
    end
    else if (signal_in != signal_out) begin
        counter <= counter + COUNTER_BITS'('b1);
    end
end

// Edge detection
always_comb begin
    internal_falling = ~signal_in & signal_out;
    internal_rising = signal_in & ~signal_out;
end

endmodule