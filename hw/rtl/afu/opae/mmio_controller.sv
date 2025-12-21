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

module mmio_controller import ccip_if_pkg::*; import local_mem_cfg_pkg::*; import VX_gpu_pkg::*; #(
    parameter STATE_DCR_WRITE     = 4,
    parameter STATE_WIDTH         = `CLOG2(STATE_DCR_WRITE+1),    
    parameter COUT_QUEUE_DATAW    = `CLOG2(VX_MEM_BYTEEN_WIDTH) + 8,
    parameter CCI_ADDR_WIDTH      = $bits(t_ccip_clAddr),
    parameter HALF_CMD_ARG_WIDTH  = 32
) (
    input  logic clk,
    input  logic reset,

    // CCI-P MMIO request
    input  t_if_ccip_c0_Rx cp2af_sRxPort_c0,

    // CCI-P MMIO response
    output t_if_ccip_c2_Tx af2cp_sTxPort_c2,

    // Inputs
    input  logic [STATE_WIDTH-1:0]      state,
    input  logic                        cout_q_empty_all,
    input  logic [COUT_QUEUE_DATAW-1:0] cout_q_dout_s,
    input  logic [127:0]                afu_id,
    input  logic [63:0]                 dev_caps,
    input  logic [63:0]                 isa_caps,

`ifdef SCOPE
    input  logic [63:0] cmd_scope_rdata,
`endif

    // Flush Queue Outputs
    output logic                          flush_fire,
    output logic [HALF_CMD_ARG_WIDTH-1:0] flush_num_blocks,
    output logic [HALF_CMD_ARG_WIDTH-1:0] flush_num_commands,
    output logic [CCI_ADDR_WIDTH-1:0]     flush_base_addr
);

    localparam AFU_ID_L           = 16'h0002;      // AFU ID Lower
    localparam AFU_ID_H           = 16'h0004;      // AFU ID Higher

    localparam MMIO_CMD_BUFFER_FLUSH     = `AFU_IMAGE_MMIO_CMD_BUFFER_FLUSH;
    localparam MMIO_CMD_BUFFER_BASE_ADDR = `AFU_IMAGE_MMIO_CMD_BUFFER_BASE_ADDR;
    localparam MMIO_CMD_BUFFER_READ_IDX  = `AFU_IMAGE_MMIO_CMD_BUFFER_READ_IDX;
    `UNUSED_PARAM(MMIO_CMD_BUFFER_READ_IDX)
    
    localparam MMIO_STATUS        = `AFU_IMAGE_MMIO_STATUS;

    localparam MMIO_DEV_CAPS      = `AFU_IMAGE_MMIO_DEV_CAPS;
    localparam MMIO_ISA_CAPS      = `AFU_IMAGE_MMIO_ISA_CAPS;

`ifdef SCOPE

    localparam MMIO_SCOPE_READ  = `AFU_IMAGE_MMIO_SCOPE_READ;
    localparam MMIO_SCOPE_WRITE = `AFU_IMAGE_MMIO_SCOPE_WRITE;
`endif

    // MMIO controller ////////////////////////////////////////////////////////
    `UNUSED_VAR(cp2af_sRxPort_c0)

    // Decode MMIO request header
    t_ccip_c0_ReqMmioHdr mmio_req_hdr;
    assign mmio_req_hdr = t_ccip_c0_ReqMmioHdr'(cp2af_sRxPort_c0.hdr[$bits(t_ccip_c0_ReqMmioHdr)-1:0]);
    `UNUSED_VAR(mmio_req_hdr)

    t_if_ccip_c2_Tx mmio_rsp;
    assign af2cp_sTxPort_c2 = mmio_rsp;

    // Handle MMIO read requests
    always @(posedge clk) begin
        if (reset) begin
            mmio_rsp.mmioRdValid <= 0;
        end else begin
            mmio_rsp.mmioRdValid <= cp2af_sRxPort_c0.mmioRdValid;
        end

        mmio_rsp.hdr.tid <= mmio_req_hdr.tid;

        if (cp2af_sRxPort_c0.mmioRdValid) begin
            case (mmio_req_hdr.address)
            // AFU header
            16'h0000: mmio_rsp.data <= {
                4'b0001, // Feature type = AFU
                8'b0,    // reserved
                4'b0,    // afu minor revision = 0
                7'b0,    // reserved
                1'b1,    // end of DFH list = 1
                24'b0,   // next DFH offset = 0
                4'b0,    // afu major revision = 0
                12'b0    // feature ID = 0
            };
            AFU_ID_L: mmio_rsp.data <= afu_id[63:0];   // afu id low
            AFU_ID_H: mmio_rsp.data <= afu_id[127:64]; // afu id hi
            16'h0006: mmio_rsp.data <= 64'h0; // next AFU
            16'h0008: mmio_rsp.data <= 64'h0; // reserved
            MMIO_STATUS: begin
                mmio_rsp.data <= 64'({cout_q_dout_s, ~cout_q_empty_all, 8'(state)});
            `ifdef DBG_TRACE_AFU
                if (state != STATE_WIDTH'(mmio_rsp.data)) begin
                    `TRACE(2, ("%t: AFU: MMIO_STATUS: addr=0x%0h, state=%0d\n", $time, mmio_req_hdr.address, state))
                end
            `endif
            end
            `ifdef SCOPE
            MMIO_SCOPE_READ: begin
                mmio_rsp.data <= cmd_scope_rdata;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: MMIO_SCOPE_READ: data=0x%h\n", $time, cmd_scope_rdata))
            `endif
            end
            `endif
            MMIO_DEV_CAPS: begin
                mmio_rsp.data <= dev_caps;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: MMIO_DEV_CAPS: data=0x%h\n", $time, dev_caps))
            `endif
            end
            MMIO_ISA_CAPS: begin
                mmio_rsp.data <= isa_caps;
            `ifdef DBG_TRACE_AFU
                if (state != STATE_WIDTH'(mmio_rsp.data)) begin
                    `TRACE(2, ("%t: AFU: MMIO_ISA_CAPS: data=%0d\n", $time, isa_caps))
                end
            `endif
            end
            default: begin
                mmio_rsp.data <= 64'h0;
            `ifdef DBG_TRACE_AFU
                `TRACE(2, ("%t: AFU: Unknown MMIO Rd: addr=0x%0h\n", $time, mmio_req_hdr.address))
            `endif
            end
            endcase
        end
    end

    // Handle MMIO write requests
    always @(posedge clk) begin
        if (reset) begin
            flush_fire <= 0;
        end
        else begin
        
            flush_fire <= 0;

            if (cp2af_sRxPort_c0.mmioWrValid) begin
                case (mmio_req_hdr.address)
                MMIO_CMD_BUFFER_FLUSH: begin
                    flush_fire <= 1;
                    flush_num_blocks   <= cp2af_sRxPort_c0.data[63:32];
                    flush_num_commands <= cp2af_sRxPort_c0.data[31:0];
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t AFU: MMIO_CMD_BUFFER_FLUSH: NUM_BLOCKS = 0x%h \n", $time, cp2af_sRxPort_c0.data[63:32]))
                    `TRACE(2, ("%t AFU: MMIO_CMD_BUFFER_FLUSH: NUM_COMMANDS = 0x%h \n", $time, cp2af_sRxPort_c0.data[31:0]))
                `endif
                end
                MMIO_CMD_BUFFER_BASE_ADDR: begin
                    flush_base_addr <= CCI_ADDR_WIDTH'(cp2af_sRxPort_c0.data);
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t AFU: MMIO_CMD_BUFFER_BASE_ADDR: data=0x%h \n", $time, 64'(cp2af_sRxPort_c0.data)))
                `endif                    
                end
                `ifdef SCOPE
                MMIO_SCOPE_WRITE: begin
                `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: AFU: MMIO_SCOPE_WRITE: data=0x%h\n", $time, 64'(cp2af_sRxPort_c0.data)))
                `endif
                end
                `endif
                default: begin
                    `ifdef DBG_TRACE_AFU
                    `TRACE(2, ("%t: Unknown MMIO Wr: addr=0x%0h, data=0x%h\n", $time, mmio_req_hdr.address, 64'(cp2af_sRxPort_c0.data)))
                    `endif
                end
                endcase
            end
        end
    end

endmodule
