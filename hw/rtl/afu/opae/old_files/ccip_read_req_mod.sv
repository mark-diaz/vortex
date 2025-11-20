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

`ifndef PLATFORM_MEMORY_INTERLEAVE
`define PLATFORM_MEMORY_INTERLEAVE 1
`endif

module ccip_read_req_packet import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(

    parameter CCI_RD_WINDOW_SIZE = 8,
    parameter CCI_ADDR_WIDTH     = $bits(t_ccip_clAddr),
    parameter CCI_RD_QUEUE_SIZE  = 2 * CCI_RD_WINDOW_SIZE,
    parameter CCI_RD_QUEUE_TAGW  = `CLOG2(CCI_RD_WINDOW_SIZE)

) (
    // global signals
    input wire clk,
    input wire reset,

    // Input
    input  t_ccip_clAddr    io_addr,
    input  logic            [CCI_ADDR_WIDTH-1:0] data_size, 

    input  logic            [CCI_RD_QUEUE_TAGW-1:0] cci_rd_req_tag,
    input  logic            [CCI_RD_QUEUE_TAGW-1:0] cci_rd_rsp_tag,
    input  logic            cci_rd_rsp_fire,

    input  t_ccip_clData    c0_data,
    input  logic            c0TxAlmFull,

    input  logic            cci_rdq_pop,

    // Additional Input Controls
    input  logic            valid,


    // Output
    output logic            output_cci_rd_req_fire,
    output t_ccip_clAddr    output_cci_rd_req_addr,
    output logic            [CCI_ADDR_WIDTH-1:0] output_cci_rd_req_ctr,

    // Addiional Output for ending
    output logic            ended

);
    // CCI-P Read Request ///////////////////////////////////////////////////////////


    // Variables
    reg cci_rd_req_valid, cci_rd_req_wait;
    wire cci_rd_req_fire;
    reg [CCI_ADDR_WIDTH-1:0] cci_rd_req_ctr;
    wire [CCI_ADDR_WIDTH-1:0] cci_rd_req_ctr_next;
    reg [CCI_RD_QUEUE_TAGW-1:0] cci_rd_rsp_ctr;
 
    t_ccip_clAddr cci_rd_req_addr;


    // Unused
    `UNUSED_VAR(cci_rd_rsp_tag);
    `UNUSED_VAR(c0_data);

    // Connections to VX_pending_size
    wire [`CLOG2(CCI_RD_QUEUE_SIZE+1)-1:0] cci_pending_reads;
    wire cci_pending_reads_full;


    // Conditions for read request fire
    assign cci_rd_req_fire = cci_rd_req_valid && !(cci_rd_req_wait || cci_pending_reads_full);
    assign cci_rd_req_ctr_next = cci_rd_req_ctr + CCI_ADDR_WIDTH'(cci_rd_req_fire ? 1 : 0);

    // VX_pending_size
    VX_pending_size #(
        .SIZE (CCI_RD_QUEUE_SIZE)
    ) cci_rd_pending_size (
        .clk   (clk),
        .reset (reset),
        .incr  (cci_rd_req_fire),
        .decr  (cci_rdq_pop),
        `UNUSED_PIN (empty),
        `UNUSED_PIN (alm_empty),
        .full  (cci_pending_reads_full),
        `UNUSED_PIN (alm_full),
        .size  (cci_pending_reads)
    );
    `UNUSED_VAR (cci_pending_reads)


    reg start, start_next;

    // Send read requests to CCI
    always @(posedge clk) begin

        // Case 0: Reset
        if (reset) begin
            cci_rd_req_valid <= 0;
            cci_rd_req_wait  <= 0;

            start <= 0;
            start_next <= 0;
            ended <= 0;

        end else if (valid) begin

            start <= start_next;

            // Start here when valid
            if ((valid)
             && (start == 1'b0)) begin
                cci_rd_req_valid <= (data_size != 0);
                cci_rd_req_wait  <= 0;

                // Update to ready in next CC  
                start_next <= 1'b1;
                ended <= 0;
            end

            // Update Valid
            cci_rd_req_valid <= (start)
                             && (cci_rd_req_ctr_next != data_size)
                             && !c0TxAlmFull;

            // Check: Begin or End request batch
            if (cci_rd_req_fire
             && (cci_rd_req_tag == CCI_RD_QUEUE_TAGW'(CCI_RD_WINDOW_SIZE-1))) begin
                cci_rd_req_wait <= 1; // end current request batch
            end

            if (cci_rd_rsp_fire
             && (cci_rd_rsp_ctr == CCI_RD_QUEUE_TAGW'(CCI_RD_WINDOW_SIZE-1))) begin
                cci_rd_req_wait <= 0; // begin new request batch


                ended <= 0;
            end
        end

        // Start here when valid 
        if ((valid)
         && (start == 1'b0)) begin
            cci_rd_req_addr    <= io_addr;
            cci_rd_req_ctr     <= '0;
            cci_rd_rsp_ctr     <= '0;
        end

        // Fire Request
        if (cci_rd_req_fire) begin
            cci_rd_req_addr <= cci_rd_req_addr + 1;
            cci_rd_req_ctr  <= cci_rd_req_ctr + $bits(cci_rd_req_ctr)'(1);


          // Debug     
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: CCI Rd Req: addr=0x%0h, tag=0x%0h, rem=%0d, pending=%0d\n", $time, cci_rd_req_addr, cci_rd_req_tag, (data_size - cci_rd_req_ctr - 1), cci_pending_reads))
            `endif
        end

        // Receive Request 
        if (cci_rd_rsp_fire) begin
            cci_rd_rsp_ctr <= cci_rd_rsp_ctr + CCI_RD_QUEUE_TAGW'(1);

            // Debug
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: CCI Rd Rsp: idx=%0d, ctr=%0d, data=0x%h\n", $time, cci_rd_rsp_tag, cci_rd_rsp_ctr, c0_data))
            `endif
        end


        // Pop Debug
        if (cci_rdq_pop) begin
            // Debug
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: CCI Rd Queue Pop: pending=%0d\n", $time, cci_pending_reads))
            `endif
        end


    end


    // Output Connections
    assign output_cci_rd_req_fire = cci_rd_req_fire;
    assign output_cci_rd_req_addr = cci_rd_req_addr;
    assign output_cci_rd_req_ctr = cci_rd_req_ctr;


endmodule
