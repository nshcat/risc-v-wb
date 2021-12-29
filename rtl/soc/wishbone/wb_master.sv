`include "wb_master.vh"

// The address and write value/mask (if needed) are latched in, so only have to be valid on the rising edge
// on which the command is issued
module wb_master(
    input logic clk_in,
    input logic reset_in,

    // Interface to core
    input wb_command_t cmd_in,
    output logic busy_out,
    output logic err_out,

    input logic [31:0] addr_in,
    output logic [31:0] rdata_out,
    input logic [31:0] wdata_in,
    input logic [3:0] wmask_in,

    // Wishbone interface
    wb_bus.master bus_master
);

typedef enum logic [1:0]
{  
    STATE_WAIT_FOR_CMD = 2'b00,
    STATE_WAIT_ACK     = 2'b01
} state_t;

logic [31:0] write_data;
logic [3:0] write_mask;
logic [31:0] read_data;
logic [31:0] address;
logic busy, error;
state_t state;

// TODO maybe have these be determined combinationally?
// Or can you directly <= to interface outputs?
// Or all WB signals combinationally derived from current state?
logic we, stb, cyc;

initial begin
    read_data = 32'h0;
    busy = 1'b0;
    error = 1'b0;
    state = STATE_WAIT_FOR_CMD;
    address = 32'h0;
    write_mask = 4'h0;
    write_data = 32'h0;
    we = 1'h0;
    stb = 1'h0;
    cyc = 1'h0;
end

always_ff @(posedge clk_in) begin
    if (~reset_in) begin
        read_data <= 32'h0;
        busy <= 1'b0;
        state <= STATE_WAIT_FOR_CMD;
        address <= 32'h0;
        write_mask <= 4'h0;
        write_data <= 32'h0;
        we <= 1'h0;
        stb <= 1'h0;
        cyc <= 1'h0;
        error <= 1'h0;
    end
    else begin
        case (state)  
            STATE_WAIT_ACK: begin
                // Check for errors
                if (bus_master.err) begin
                    // Set error indicator. Will be cleared upon next command.
                    error <= 1'b1;

                    // Transaction cancelled
                    stb <= 1'b0;
                    cyc <= 1'b0;
                    we <= 1'b0;
                    state <= STATE_WAIT_FOR_CMD;
                    busy <= 1'b0;
                end
                // Check for slaves ack
                else if (bus_master.ack) begin
                    if (~we) begin
                        // This was a load, retrieve read data from bus
                        read_data <= bus_master.rdata;
                    end

                    // Transaction is complete, deassert control lines
                    stb <= 1'b0;
                    cyc <= 1'b0;
                    we <= 1'b0;
                    state <= STATE_WAIT_FOR_CMD;
                    busy <= 1'b0;
                end
            end
            // STATE_WAIT_FOR_CMD
            default: begin
                // Check if core provided command
                if (cmd_in != WISHBONE_CMD_NONE) begin
                    // We are busy now
                    busy <= 1'b1;

                    // Latch task data
                    address <= addr_in;
                    write_data <= wdata_in;
                    write_mask <= wmask_in;

                    // Clear error indicator
                    error <= 1'h0;

                    // Start transaction
                    we <= (cmd_in == WISHBONE_CMD_LOAD) ? 1'b0 : 1'b1;
                    stb <= 1'b1;
                    cyc <= 1'b1;

                    // Wait for slave to ack transaction
                    state <= STATE_WAIT_ACK;
                end
            end
        endcase
    end
end

assign rdata_out = read_data;
assign busy_out = busy;
assign err_out = error;

// Bus assignments
assign bus_master.addr = address;
assign bus_master.wdata = write_data;
assign bus_master.sel = we ? write_mask : 4'b1111; // Always read full words
assign bus_master.we = we;
assign bus_master.stb = stb;
assign bus_master.cyc = cyc;


endmodule