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

module ccip_write_req import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(

    parameter CCI_ADDR_WIDTH     = $bits(t_ccip_clAddr),
    parameter CCI_DATA_WIDTH     = $bits(t_ccip_clData),

    parameter CCI_RW_PENDING_SIZE= 256,

    parameter STATE_IDLE         = 0,
    parameter STATE_DCR_WRITE    = 4,
    parameter STATE_WIDTH        = `CLOG2(STATE_DCR_WRITE+1),

    parameter CMD_TYPE_WIDTH     = `CLOG2(`AFU_IMAGE_CMD_MAX_VALUE+1),
    parameter CMD_MEM_READ       = `AFU_IMAGE_CMD_MEM_READ
) (
    // global signals
    input wire clk,
    input wire reset,

    // Input
    input  logic            [CMD_TYPE_WIDTH-1:0] cmd_type,
    input  t_ccip_clAddr    cmd_io_addr,
    input  logic            [CCI_ADDR_WIDTH-1:0] cmd_mem_addr,
    input  logic            [CCI_ADDR_WIDTH-1:0] cmd_data_size,

    input  logic            cci_mem_rd_req_fire,
    input  logic            cci_mem_rd_rsp_fire,
    input  logic            cci_wr_rsp_fire,

    input  logic            [CCI_ADDR_WIDTH-1:0] cci_mem_rsp_tag,
    input  logic            [CCI_DATA_WIDTH-1:0] cci_mem_rsp_data,

    input  logic            [STATE_WIDTH-1:0] state,
    input  t_ccip_clData    c1_data,

    input  logic            [`CLOG2(CCI_RW_PENDING_SIZE+1)-1:0] cci_pending_writes,

    // Output
    output logic            output_cci_wr_req_fire,
    output logic            output_cci_wr_req_done,

    output logic            [CCI_ADDR_WIDTH-1:0] output_cci_mem_rd_req_ctr,
    output logic            [CCI_ADDR_WIDTH-1:0] output_cci_mem_rd_req_addr,
    output logic            [CCI_ADDR_WIDTH-1:0] output_cci_wr_req_ctr,
    output logic            output_cci_mem_rd_req_done,
    output t_ccip_clAddr    output_cci_wr_req_addr,
    output t_ccip_clData    output_cci_wr_req_data

);
    // CCI-P Write Request ///////////////////////////////////////////////////////////


    // Variables
    reg [CCI_ADDR_WIDTH-1:0] cci_mem_rd_req_ctr;
    reg [CCI_ADDR_WIDTH-1:0] cci_mem_rd_req_addr;
    reg cci_mem_rd_req_done;

    reg [CCI_ADDR_WIDTH-1:0] cci_wr_req_ctr;
    reg           cci_wr_req_fire;
    t_ccip_clAddr cci_wr_req_addr;
    t_ccip_clData cci_wr_req_data;
    reg cci_wr_req_done;


    // Unused Variables
    `UNUSED_VAR(c1_data);
    `UNUSED_VAR(cci_pending_writes);


    // Send write requests to CCI
    always @(posedge clk) begin
        if (reset) begin
            cci_wr_req_fire <= 0;
        end else begin
            cci_wr_req_fire <= cci_mem_rd_rsp_fire;
        end

        if ((STATE_IDLE == state)
        &&  (CMD_MEM_READ == cmd_type)) begin
            cci_mem_rd_req_ctr  <= '0;
            cci_mem_rd_req_addr <= cmd_mem_addr;
            cci_mem_rd_req_done <= 0;
            cci_wr_req_ctr      <= cmd_data_size;
            cci_wr_req_done     <= 0;
        end

        if (cci_mem_rd_req_fire) begin
            cci_mem_rd_req_addr <= cci_mem_rd_req_addr + CCI_ADDR_WIDTH'(1);
            cci_mem_rd_req_ctr  <= cci_mem_rd_req_ctr + CCI_ADDR_WIDTH'(1);
            if (cci_mem_rd_req_ctr == (cmd_data_size-1)) begin
                cci_mem_rd_req_done <= 1;
            end
        end

        cci_wr_req_addr <= cmd_io_addr + t_ccip_clAddr'(cci_mem_rsp_tag);
        cci_wr_req_data <= t_ccip_clData'(cci_mem_rsp_data);



        if (cci_wr_req_fire) begin
            `ASSERT(cci_wr_req_ctr != 0, ("runtime error"));
            cci_wr_req_ctr <= cci_wr_req_ctr - CCI_ADDR_WIDTH'(1);
            if (cci_wr_req_ctr == CCI_ADDR_WIDTH'(1)) begin
            cci_wr_req_done <= 1;
            end
        `ifdef DBG_TRACE_AFU
            `TRACE(2, ("%t: AFU: CCI Wr Req: addr=0x%0h, rem=%0d, pending=%0d, data=0x%h\n", $time, cci_wr_req_addr, (cci_wr_req_ctr - 1), cci_pending_writes, c1_data))
        `endif
        end

        if (cci_wr_rsp_fire) begin
        `ifdef DBG_TRACE_AFU
            `TRACE(2, ("%t: AFU: CCI Wr Rsp: pending=%0d\n", $time, cci_pending_writes))
        `endif
        end
    end


    // Output Wires
    assign output_cci_wr_req_fire = cci_wr_req_fire;
    assign output_cci_wr_req_done = cci_wr_req_done;
    assign output_cci_mem_rd_req_ctr = cci_mem_rd_req_ctr;
    assign output_cci_mem_rd_req_addr = cci_mem_rd_req_addr;
    assign output_cci_mem_rd_req_done = cci_mem_rd_req_done;
    assign output_cci_wr_req_ctr = cci_wr_req_ctr;

    assign output_cci_wr_req_addr = cci_wr_req_addr;
    assign output_cci_wr_req_data = cci_wr_req_data;




endmodule
