module pll // 25 MHz clock
(
    input logic fast_clk_in,
    output logic slow_clk_out,
    output logic locked_out
);

logic clk_unbuffered;
logic clk_feedback;
logic clk_feedback_buffered;

logic clk1_unused, clk2_unused, clk3_unused, clk4_unused, clk5_unused;

PLLE2_BASE #(
    .BANDWIDTH("OPTIMIZED"), // OPTIMIZED, HIGH, LOW
    .CLKFBOUT_MULT(8),  // Multiply value for all CLKOUT (2-64)
    .CLKFBOUT_PHASE(0.0),  // Phase offset in degrees of CLKFB, (-360-360)
    .CLKIN1_PERIOD(10.0),  // Input clock period in ns to ps resolution

    .CLKOUT0_DIVIDE(32),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0.0),

    .CLKOUT1_DIVIDE(32),
    .CLKOUT1_DUTY_CYCLE(0.5),
    .CLKOUT1_PHASE(0.0),

    .CLKOUT2_DIVIDE(32),
    .CLKOUT2_DUTY_CYCLE(0.5),
    .CLKOUT2_PHASE(0.0),

    .CLKOUT3_DIVIDE(32),
    .CLKOUT3_DUTY_CYCLE(0.5),
    .CLKOUT3_PHASE(0.0),

    .CLKOUT4_DIVIDE(32),
    .CLKOUT4_DUTY_CYCLE(0.5),
    .CLKOUT4_PHASE(0.0),

    .CLKOUT5_DIVIDE(32),
    .CLKOUT5_DUTY_CYCLE(0.5),
    .CLKOUT5_PHASE(0.0),

    .DIVCLK_DIVIDE(1),      // Master division value , (1-56)
    .REF_JITTER1(0.0),    // Reference input jitter in UI (0.000-0.999)
    .STARTUP_WAIT("FALSE")  // Delayu DONE until PLL Locks, ("TRUE"/"FALSE")
 ) genclock(
     .CLKOUT0(clk_unbuffered),
     .CLKOUT1(clk1_unused),
     .CLKOUT2(clk2_unused),
     .CLKOUT3(clk3_unused),
     .CLKOUT4(clk4_unused),
     .CLKOUT5(clk5_unused),
     .CLKFBOUT(clk_feedback), // 1-bit output, feedback clock
     .CLKIN1(fast_clk_in),
     .PWRDWN(1'b0),
     .RST(1'b0),
     .CLKFBIN(clk_feedback_buffered),    // 1-bit input, feedback clock
     .LOCKED(locked_out)
 );

BUFG bufg(
    .I(clk_unbuffered),
    .O(slow_clk_out)
);

BUFH bufh(
    .I(clk_feedback),
    .O(clk_feedback_buffered)
);

endmodule