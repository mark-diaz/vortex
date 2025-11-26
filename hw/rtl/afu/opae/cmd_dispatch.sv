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

module cmd_dispatch import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(

    parameter CCI_ADDR_WIDTH      = $bits(t_ccip_clAddr),
    parameter RESET_CTR_WIDTH     = `CLOG2(`RESET_DELAY+1),

    parameter STATE_IDLE         = 0,
    parameter STATE_MEM_WRITE    = 1,
    parameter STATE_MEM_READ     = 2,
    parameter STATE_RUN          = 3,
    parameter STATE_DCR_WRITE    = 4,
    parameter STATE_WIDTH        = `CLOG2(STATE_DCR_WRITE+1),

    parameter CMD_MEM_READ       = `AFU_IMAGE_CMD_MEM_READ,
    parameter CMD_MEM_WRITE      = `AFU_IMAGE_CMD_MEM_WRITE,
    parameter CMD_DCR_WRITE      = `AFU_IMAGE_CMD_DCR_WRITE,
    parameter CMD_RUN            = `AFU_IMAGE_CMD_RUN,
    parameter CMD_TYPE_WIDTH     = `CLOG2(`AFU_IMAGE_CMD_MAX_VALUE+1)
) (
    // global signals
    input wire clk,
    input wire reset,

    // Input 
    input  logic            [CMD_TYPE_WIDTH-1:0] cmd_type,
    input  logic            cmd_mem_rd_done,
    input  logic            cmd_mem_wr_done,

    input  logic            vx_busy,

    input  t_ccip_clAddr    cmd_io_addr,
    input  logic            [CCI_ADDR_WIDTH-1:0] cmd_mem_addr,
    input  logic            [CCI_ADDR_WIDTH-1:0] cmd_data_size,
    
    input  logic            [VX_DCR_ADDR_WIDTH-1:0] cmd_dcr_addr,
    input  logic            [VX_DCR_DATA_WIDTH-1:0] cmd_dcr_data,

    // Output
    output logic            [STATE_WIDTH-1:0] output_state,
    output logic            output_vx_reset
);

    // Silence unused parameter warnings (if any) without breaking port list syntax
    // Moved from inside the port list where it was invalid.
    localparam int _unused_cmd_mem_read = CMD_MEM_READ;


    // Unused Variables
    `UNUSED_VAR(cmd_io_addr);
    `UNUSED_VAR(cmd_mem_addr);
    `UNUSED_VAR(cmd_data_size);
    `UNUSED_VAR(cmd_dcr_addr);
    `UNUSED_VAR(cmd_dcr_data);

    // COMMAND FSM (Cmd Dispatcher --> Own Module)
    reg [STATE_WIDTH-1:0] state;

    reg [RESET_CTR_WIDTH-1:0] vx_reset_ctr = 0;
    reg  vx_busy_wait = 0;
    reg  vx_reset = 1;

    always @(posedge clk) begin
        if (reset) begin
            state    <= STATE_IDLE;
            vx_reset <= 1;
        end else begin
            case (state)
            STATE_IDLE: begin
                case (cmd_type) 
                    // Zuoning, this look at cmd_type register and decide whether it is a mem read or mem write

                CMD_MEM_READ: begin
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: Goto STATE MEM_READ: ia=0x%0h addr=0x%0h size=%0d\n", $time, cmd_io_addr, cmd_mem_addr, cmd_data_size))
                `endif
                    state <= STATE_MEM_READ;
                end

                CMD_MEM_WRITE: begin
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: Goto STATE MEM_WRITE: ia=0x%0h addr=0x%0h size=%0d\n", $time, cmd_io_addr, cmd_mem_addr, cmd_data_size))
                `endif
                    state <= STATE_MEM_WRITE;
                end

                CMD_DCR_WRITE: begin
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: Goto STATE DCR_WRITE: addr=0x%0h data=%0d\n", $time, cmd_dcr_addr, cmd_dcr_data))
                `endif
                    state <= STATE_DCR_WRITE;
                end

                CMD_RUN: begin
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: Goto STATE RUN\n", $time))
                `endif
                    state <= STATE_RUN;
                    vx_reset_ctr <= RESET_CTR_WIDTH'(`RESET_DELAY-1);
                    vx_reset <= 1;
                end
                default: begin
                    state <= state;
                end
                endcase
            end
            
            STATE_MEM_READ: begin
                if (cmd_mem_rd_done) begin
                    state <= STATE_IDLE;
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: Goto STATE IDLE\n", $time))
                `endif
                end
            end
            
            STATE_MEM_WRITE: begin
                if (cmd_mem_wr_done) begin
                    state <= STATE_IDLE;
                end
            end
            
            STATE_DCR_WRITE: begin
                state <= STATE_IDLE;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: Goto STATE IDLE\n", $time))
            `endif
            end
            
            STATE_RUN: begin
                if (vx_reset) begin
                    // wait until the reset network is ready
					          if (vx_reset_ctr == RESET_CTR_WIDTH'(0)) begin
            					`ifdef DBG_TRACE_AFU
        	    					`TRACE(2, ("%t: AFU: Begin execution\n", $time))
            					`endif
          						vx_busy_wait <= 1;
					          	vx_reset <= 0;
          					end
                end else begin
                    if (vx_busy_wait) begin
          						// wait until processor goes busy
					          	if (vx_busy) begin
          							vx_busy_wait <= 0;
          						end
          					end else begin
          						// wait until the processor is not busy
          						if (~vx_busy) begin
            						`ifdef DBG_TRACE_AFU
            							`TRACE(2, ("%t: AFU: End execution\n", $time))
                          `TRACE(2, ("%t: AFU: Goto STATE IDLE\n", $time))
            						`endif
        							state <= STATE_IDLE;
          						end
					          end
                end
            end
            default:;
            endcase

      			if (vx_reset_ctr != RESET_CTR_WIDTH'(0)) begin
			      	vx_reset_ctr <= vx_reset_ctr - RESET_CTR_WIDTH'(1);
      			end
        end
    end

    // Output
    assign output_state = state;
    assign output_vx_reset = vx_reset;

endmodule
