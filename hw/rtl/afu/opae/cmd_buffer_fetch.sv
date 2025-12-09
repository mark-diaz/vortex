// Copyright © 2019-2023
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

`ifndef NOPAE
`include "afu_json_info.vh"
`else
`include "vortex_afu.vh"
`endif
`include "VX_define.vh"

module cmd_buffer_fetch import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(
    parameter CCI_ADDR_WIDTH      = $bits(t_ccip_clAddr),
    parameter HALF_CMD_ARG_WIDTH  = 32,
    parameter CMD_BUFFER_REQ_DATA_WIDTH = CCI_ADDR_WIDTH + HALF_CMD_ARG_WIDTH

) (
    // global signals
    input wire clk,
    input wire reset,

    // flush queue fields
    input logic cmd_buffer_fetch_fire, // flush pop

    input logic [HALF_CMD_ARG_WIDTH-1:0] num_blocks,
    input logic [HALF_CMD_ARG_WIDTH-1:0] num_commands,
    input logic [CCI_ADDR_WIDTH-1:0]     base_addr,

    // arbiter
    input logic read_req_arb_ready,
    output logic cmd_buffer_fetch_ready,

    output logic [CMD_BUFFER_REQ_DATA_WIDTH-1:0] read_cmd_buffer_req_data,
    output logic                                 read_cmd_buffer_valid
);

    reg cmd_buffer_fetch_active;

    // Flush registers
    reg [HALF_CMD_ARG_WIDTH-1:0] num_blocks_ctr;
    reg [HALF_CMD_ARG_WIDTH-1:0] num_commands_r;
    reg [CCI_ADDR_WIDTH-1:0]     base_addr_r;

    // Handshake signals
    assign cmd_buffer_fetch_ready = !cmd_buffer_fetch_active;
    assign read_cmd_buffer_valid = cmd_buffer_fetch_active;
    assign read_cmd_buffer_req_data = {base_addr_r, num_commands_r};

    // Send read requests to the arbiter
    always_ff @(posedge clk) begin

        if (reset) begin 
            cmd_buffer_fetch_active <= 0;
        end
        else begin

            // Start registering signals 
            if (cmd_buffer_fetch_fire && !cmd_buffer_fetch_active) begin
                cmd_buffer_fetch_active <= 1;
                num_blocks_ctr <= num_blocks;
                num_commands_r <= num_commands;
                base_addr_r <= base_addr;

            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: FETCH_START: blocks=%0d cmds=%0d base=0x%0h\n", $time, num_blocks, num_commands, base_addr))
            `endif

            end

            if (cmd_buffer_fetch_active && read_req_arb_ready) begin

            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: FETCH_SEND: addr=0x%0h blocks_left=%0d\n", $time, base_addr_r, num_blocks_ctr))
            `endif

                num_blocks_ctr <= num_blocks_ctr - HALF_CMD_ARG_WIDTH'(1);
                base_addr_r <= base_addr_r + CCI_ADDR_WIDTH'(64);

                if (num_blocks_ctr == 1) begin
                    cmd_buffer_fetch_active <= 0;

                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: FETCH_DONE\n", $time))
                `endif
                end
            end
        end

    end

endmodule
