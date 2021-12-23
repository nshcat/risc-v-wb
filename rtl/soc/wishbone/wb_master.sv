`include "wb_master.vh"

// The address and write value/mask (if needed) are latched in, so only have to be valid on the rising edge
// on which the command is issued
module wb_master(
    input logic clk_in,
    input logic reset_in,

    // Interface to core
    input wb_command_t cmd_in,
    output logic busy_out,

    input logic [31:0] addr_in,
    output logic [31:0] rdata_out,
    input logic [31:0] wdata_in,
    input logic [3:0] wmask_in

    // Wishbone interface
    // XXX
);

typedef enum logic [1:0]
{  
    STATE_WAIT_FOR_CMD = 2'b00,
    STATE_DO_READ      = 2'b01,
    STATE_DO_WRITE     = 2'b10
} state_t;

logic [31:0] read_data;
logic busy;
state_t state;

initial begin
    read_data = 32'h0;
    busy = 1'b0;
    state = STATE_WAIT_FOR_CMD;
end

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        read_data <= 32'h0;
        busy <= 1'b0;
        state <= STATE_WAIT_FOR_CMD;
    end
    else begin
        case (state)  
            STATE_DO_READ: begin
                read_data <= 32'h00128293;   // addi t0, t0, 0x1
                busy <= 1'b0;
                state <= STATE_WAIT_FOR_CMD;
            end
            STATE_DO_WRITE: begin
                busy <= 1'b0;
                state <= STATE_WAIT_FOR_CMD;
            end
            // STATE_WAIT_FOR_CMD
            default: begin
                // Check if core provided command
                if (cmd_in != WISHBONE_CMD_NONE) begin
                    // We are busy now
                    busy <= 1'b1;
                    state <= (cmd_in == WISHBONE_CMD_LOAD) ? STATE_DO_READ : STATE_DO_WRITE;
                end
            end
        endcase
    end
end

assign rdata_out = read_data;
assign busy_out = busy;

// XXX WIP implementation

endmodule