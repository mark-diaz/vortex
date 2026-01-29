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

module command_fetch import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(
    parameter CCI_ADDR_WIDTH        = $bits(t_ccip_clAddr),
    parameter HALF_CMD_ARG_WIDTH    = 32,
    parameter CCI_RD_WINDOW_SIZE = 8,
    parameter CCI_RD_QUEUE_SIZE  = 2 * CCI_RD_WINDOW_SIZE,
    parameter CCI_RD_QUEUE_TAGW     = `CLOG2(CCI_RD_WINDOW_SIZE),
    parameter RD_REQ_ARB_DATA_WIDTH = CCI_ADDR_WIDTH + HALF_CMD_ARG_WIDTH + CCI_RD_QUEUE_TAGW + 1

) (
    // global signals
    input wire clk,
    input wire reset,

    // flush queue fields
    input logic                          flush_valid, // Valid from flush queue: not empty
    output logic                         flush_ready, // FSM is ready for next flush

    input logic [HALF_CMD_ARG_WIDTH-1:0] flush_num_blocks,
    input logic [HALF_CMD_ARG_WIDTH-1:0] flush_num_commands,
    input logic [CCI_ADDR_WIDTH-1:0]     flush_base_addr,

    // arbiter
    input logic                              rd_req_ready,
    output logic                             rd_req_valid,
    output logic [RD_REQ_ARB_DATA_WIDTH-1:0] rd_req,

    // core cache interface (cci)
    input logic                         cci_rd_rsp_fire,
    input logic [CCI_RD_QUEUE_TAGW-1:0] cci_rd_rsp_tag,
    input logic                         cci_rdq_pop,
    
    input  t_ccip_clData c0_data,
    input  logic         c0TxAlmFull
);

    // states
    localparam STATE_IDLE  = 0;
    localparam STATE_FETCH = 1;
    localparam STATE_WIDTH = `CLOG2(STATE_FETCH+1);
    
    reg [STATE_WIDTH-1:0] state;

    // Flush registers
    reg [HALF_CMD_ARG_WIDTH-1:0] num_blocks;
    reg [HALF_CMD_ARG_WIDTH-1:0] num_commands;
    // reg [CCI_ADDR_WIDTH-1:0]     base_addr;

    wire cmd_fetch_done;    
    assign flush_ready = (state == STATE_IDLE);

    wire flush_fire;
    assign flush_fire = flush_ready && flush_valid;

    // COMMAND FETCH FSM 
    always @(posedge clk) begin
        if (reset) begin
            state    <= STATE_IDLE;
        end else begin
            case (state)
                STATE_IDLE: begin
                    if (flush_fire) begin
                    `ifdef DBG_TRACE_AFU
                        `TRACE(2, ("%t [COMMAND BUFFER HW] AFU: Goto STATE CMD_FETCH blocks=%0d cmds=%0d base=0x%h\n",
                            $time, flush_num_blocks, flush_num_commands, flush_base_addr))
                    `endif
                        state        <= STATE_FETCH;
                        num_blocks   <= flush_num_blocks;
                        num_commands <= flush_num_commands;
                        // base_addr    <= flush_base_addr;
                    end
                end

                STATE_FETCH: begin
                    if (cmd_fetch_done) begin
                        state <= STATE_IDLE;
                    `ifdef DBG_TRACE_AFU
                        `TRACE(2, ("%t: [COMMAND BUFFER HW] AFU: STATE_FETCH Goto STATE IDLE\n", $time))
                    `endif
                    end
                end

                default:;
            endcase
        end
    end

    // CCI-P Read Controller
    reg cci_rd_req_valid, cci_rd_req_wait;
    wire [CCI_RD_QUEUE_TAGW-1:0] cci_rd_req_tag;
    wire cci_rd_req_fire;
    reg [CCI_ADDR_WIDTH-1:0] cci_rd_req_ctr;
    wire [CCI_ADDR_WIDTH-1:0] cci_rd_req_ctr_next;
    reg [CCI_RD_QUEUE_TAGW-1:0] cci_rd_rsp_ctr;
    wire cci_pending_reads_empty;

    t_ccip_clAddr cci_rd_req_addr;

    // Connections to VX_pending_size
    wire [`CLOG2(CCI_RD_QUEUE_SIZE+1)-1:0] cci_pending_reads;
    wire cci_pending_reads_full;
    
    // Unused
    `UNUSED_VAR(cci_rd_rsp_tag);
    `UNUSED_VAR(c0_data);
    `UNUSED_VAR (cci_pending_reads)
    `UNUSED_VAR (cci_rd_req_ctr_next)

    assign cci_rd_req_fire = cci_rd_req_valid && !(cci_rd_req_wait || cci_pending_reads_full);
    assign cci_rd_req_ctr_next = cci_rd_req_ctr + CCI_ADDR_WIDTH'(cci_rd_req_fire ? 1 : 0);
    assign cci_rd_req_tag = CCI_RD_QUEUE_TAGW'(cci_rd_req_ctr);

    assign cmd_fetch_done = (HALF_CMD_ARG_WIDTH'(cci_rd_req_ctr) == num_blocks) && cci_pending_reads_empty;
    assign rd_req_valid = (STATE_FETCH == state); // TODO: maybe consider CCI_RD_RSP_FIRE?

    assign rd_req = {cci_rd_req_fire, cci_rd_req_addr, cci_rd_req_tag, num_commands};

    // VX_pending_size
    VX_pending_size #(
        .SIZE (CCI_RD_QUEUE_SIZE)
    ) fetch_pending_size (
        .clk   (clk),
        .reset (reset),
        .incr  (cci_rd_req_fire),
        .decr  (cci_rdq_pop),
        .empty (cci_pending_reads_empty),
        `UNUSED_PIN (alm_empty),
        .full  (cci_pending_reads_full),
        `UNUSED_PIN (alm_full),
        .size  (cci_pending_reads)
    );

    // Send read requests to CCI
    always @(posedge clk) begin

        // Case 0: Reset
        if (reset) begin
            cci_rd_req_valid <= 0;
            cci_rd_req_wait  <= 0;
        end else begin

            // Case 1: If Idle now + flush fire (changed from cmd is mem write)
            if ((STATE_IDLE == state) && flush_fire) begin
                cci_rd_req_valid <= (flush_num_blocks != 0); // use flush_num_blocks while in IDLE && fire
                cci_rd_req_wait  <= 0;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU FETCH: Read Request Valid: num_blocks=0x%0d\n", $time, flush_num_blocks))
            `endif
            end

            // Backpressure if the read request arbiter is not ready
            if ((STATE_FETCH == state) && (HALF_CMD_ARG_WIDTH'(cci_rd_req_ctr_next) != num_blocks) &&
                !c0TxAlmFull && rd_req_ready) begin
                cci_rd_req_valid <= 1'b1;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU FETCH: Read Request Valid: no back pressure\n", $time))
            `endif
            end

            // Check: Begin or End request batch
            if (cci_rd_req_fire && (cci_rd_req_tag == CCI_RD_QUEUE_TAGW'(CCI_RD_WINDOW_SIZE-1))) begin
                cci_rd_req_wait <= 1; // end current request batch
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU FETCH: Begin or end request batch\n", $time))
            `endif                
            end

            if (cci_rd_rsp_fire && (cci_rd_rsp_ctr == CCI_RD_QUEUE_TAGW'(CCI_RD_WINDOW_SIZE-1))) begin
                cci_rd_req_wait <= 0; // begin new request batch
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU FETCH: Begin request batch\n", $time))
            `endif      
            end
        end

        // TODO: move to else block of reset
        // Case 1: If Idle now + flush fire (changed from cmd is mem write)
        if ((STATE_IDLE == state) && flush_fire) begin
            cci_rd_req_addr    <= flush_base_addr; // use flush_base_addr while in IDLE && fire
            cci_rd_req_ctr     <= '0;
            cci_rd_rsp_ctr     <= '0;

            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU FETCH: CCI Rd Req: addr=0x%0h\n", $time, flush_base_addr))
            `endif

        end

        // Debug: Ready to fire 
        if (cci_rd_req_fire) begin
            cci_rd_req_addr <= cci_rd_req_addr + 1;
            cci_rd_req_ctr  <= cci_rd_req_ctr + $bits(cci_rd_req_ctr)'(1);
        // `ifdef DBG_TRACE_AFU
        //     `TRACE(2, ("%t: AFU: CCI Rd Req: addr=0x%0h, tag=0x%0h, rem=%0d, pending=%0d\n", $time, cci_rd_req_addr, cci_rd_req_tag, (num_blocks - HALF_CMD_ARG_WIDTH'(cci_rd_req_ctr) - 1), cci_pending_reads))
        // `endif
        end

        // Case: Ready to fire ==> Add to Addr base
        if (cci_rd_rsp_fire) begin
            cci_rd_rsp_ctr <= cci_rd_rsp_ctr + CCI_RD_QUEUE_TAGW'(1);
        // `ifdef DBG_TRACE_AFU
        //     `TRACE(2, ("%t: AFU: CCI Rd Rsp: idx=%0d, ctr=%0d, data=0x%h\n", $time, cci_rd_rsp_tag, cci_rd_rsp_ctr, c0_data))
        // `endif
        end

        // TODO: replace with cmd queue
        if (cci_rdq_pop) begin
        // `ifdef DBG_TRACE_AFU
        //     `TRACE(2, ("%t: AFU: CCI Rd Queue Pop: pending=%0d\n", $time, cci_pending_reads))
        // `endif
        end

    end


endmodule
