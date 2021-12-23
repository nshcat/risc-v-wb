`ifndef _WB_MASTER_H
`define _WB_MASTER_H

typedef enum logic [1:0]
{
    WISHBONE_CMD_NONE    = 2'b00,
    WISHBONE_CMD_LOAD    = 2'b01,
    WISHBONE_CMD_STORE   = 2'b10
} wb_command_t;

`endif