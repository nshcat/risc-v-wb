module top_verilator(
    input logic clk_i,
    input logic reset_i
);

logic [3:0] leds;
logic [2:0] rgb1;
logic [2:0] rgb2;
logic test_irq;

soc soc(
    .clk_in(clk_i),
    .reset_in(reset_i),
    .leds_out(leds),
    .rgb1_out(rgb1),
    .rgb2_out(rgb2)
);

logic [31:0] counter;
always_ff @(posedge clk_i) begin
    if (~reset_i) begin
        counter <= 32'h0;
        test_irq <= 1'b0;
    end
    if (test_irq == 1'b1) begin
        test_irq <= 1'b0;
        counter <= 32'h0;
    end
    else begin
        if (counter == 32'h60) begin
            test_irq <= 1'b1;
        end
        else begin
            counter <= counter + 32'h1;
        end
    end
end

endmodule