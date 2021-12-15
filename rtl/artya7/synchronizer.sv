// Synchronized async input to given clock signal
module synchronizer #(
    parameter DEPTH = 3 // How many FF stages to use for synchronization
)
(
    input logic clk_in,
    input logic signal_in,
    output logic signal_out
);

logic [DEPTH-1:0] buffer;
initial begin
    buffer = {(DEPTH){1'b0}};
end

always_ff @(posedge clk_in) begin
    buffer[DEPTH-1:0] <= {buffer[DEPTH-2:0], signal_in};
end

assign signal_out = buffer[DEPTH-1];

endmodule