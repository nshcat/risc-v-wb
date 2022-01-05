// UART receiver module
// Uses 8 bits, one stop bit, one start bit, no parity bit
module uart_rx#(
    parameter CLK_FREQ = 25000000,      // Main clock frequency, in Hz
    parameter BAUD_RATE = 115200
)(
    input logic clk_in,
    input logic reset_in,

    input logic uart_rx_in,             // UART RX line, synchronized to clk_in

    output logic [7:0] rx_data_out,
    output logic rx_data_valid_out
);

// How many clock cycles a single bit transfer takes
localparam CYCLES_PER_BIT = CLK_FREQ / BAUD_RATE;
localparam CYCLES_PER_BIT_HALF = CYCLES_PER_BIT / 2;
localparam COUNTER_BITS = $clog2(CYCLES_PER_BIT + 1);

typedef enum logic [2:0]
{  
    STATE_IDLE          = 3'b000,
    STATE_RECV_START    = 3'b001,
    STATE_RECV_DATA     = 3'b010,
    STATE_RECV_STOP     = 3'b011,
    STATE_END           = 3'b100
} state_t;

state_t state;
logic [2:0] bit_idx;
logic [7:0] rx_data;
logic rx_valid;
logic [COUNTER_BITS-1:0] counter;

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        state <= STATE_IDLE;
        bit_idx <= 3'h0;
        rx_data <= 8'h0;
        rx_valid <= 1'b0;
        counter <= 0;
    end
    else begin
        case (state)  
            STATE_RECV_START: begin
                // We wait half the cycles of a single bit to check the start bit again
                // in the middle. This allows us to wait the full number of cycles
                // to keep sampling the bits in the center.
                if (counter < COUNTER_BITS'(CYCLES_PER_BIT_HALF - 1)) begin
                    counter <= counter + 1;
                end
                else begin
                    // Sample start bit again at center
                    if (uart_rx_in == 1'b0) begin
                       // We can start sampling the bits
                       state <= STATE_RECV_DATA;
                       counter <= 0; 
                    end
                    else begin
                        // Start bit corrupted, abort
                        state <= STATE_IDLE;
                    end
                end
            end
            STATE_RECV_DATA: begin
                // Wait a full bit cycle to read the center of the next bit
                if (counter < COUNTER_BITS'(CYCLES_PER_BIT - 1)) begin
                    counter <= counter + 1;
                end
                else begin
                    counter <= 0;

                    // Read the current bit
                    rx_data[bit_idx] <= uart_rx_in;

                    // Are there any bits left?
                    if (bit_idx < 3'h7) begin
                        bit_idx <= bit_idx + 3'h1;
                    end
                    else begin // We are done, wait for STOP bit
                        state <= STATE_RECV_STOP;
                    end
                end
            end
            STATE_RECV_STOP: begin
                // Wait until middle of stop bit
                if (counter < COUNTER_BITS'(CYCLES_PER_BIT - 1)) begin
                    counter <= counter + 1;
                end
                else begin
                    state <= STATE_END;
                    rx_valid <= 1'b1;
                end
            end
            STATE_END: begin
                // We keep the rx_valid signal asserted for one clock cycle
                rx_valid <= 1'b0;
                state <= STATE_IDLE;
            end
            /* STATE_IDLE */
            default: begin
                // We continuously sample the input line, looking for a 1->0 transition, indicating
                // a start bit.
                if (uart_rx_in == 1'b0) begin
                    state <= STATE_RECV_START;
                    counter <= 0;
                    rx_data <= 8'h0;
                    bit_idx <= 3'h0;
                end
            end
        endcase
    end
end

assign rx_data_out = rx_data;
assign rx_data_valid_out = rx_valid;

endmodule