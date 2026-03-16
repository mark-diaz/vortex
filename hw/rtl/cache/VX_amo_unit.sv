`include "VX_cache_define.vh"

/*
 * VX_amo_unit: RISC-V Atomic Memory Operation (AMO) Unit
 *
 * Implements the logic for RISC-V 'A' extension instructions.
 * It intercepts requests to a cache bank, handles AMO operations through a state machine,
 * and passes non-AMO requests through.
 */

module VX_amo_unit import VX_gpu_pkg::*; #(
    parameter BANK_ID             = 0,
    // parameter `STRING INSTANCE_ID = "",
    // Bus Parameters
    parameter TAG_WIDTH          = 0,
    parameter WORD_WIDTH         = 0,
    parameter ADDR_WIDTH         = 0,
    // parameter OWNER_ID_WIDTH     = 0,
    // parameter MEM_FLAGS_WIDTH    = 0,
    parameter WORD_SEL_WIDTH     = 0,
    parameter REQ_SEL_WIDTH      = 0,
    // parameter CS_LINE_ADDR_WIDTH = 0,
    parameter WORD_SIZE          = 0
) (
    input wire clk,
    input wire reset,

    // Interface from Core/XBar
    input  wire                         core_req_valid,
    input  wire [TAG_WIDTH-1:0]         core_req_tag,
    input  wire [`UP(ADDR_WIDTH)-1:0]   core_req_addr,
    input  wire                         core_req_rw,
    input  wire [WORD_SIZE-1:0]         core_req_byteen,
    input  wire [WORD_WIDTH-1:0]        core_req_data,
    input  wire [`UP(MEM_FLAGS_WIDTH)-1:0] core_req_flags,
    input  wire [WORD_SEL_WIDTH-1:0]    core_req_wsel,
    input  wire [REQ_SEL_WIDTH-1:0]     core_req_idx,
    output wire                         core_req_ready,

    // Interface to Core/XBar
    output wire                         core_rsp_valid,
    output wire [TAG_WIDTH-1:0]         core_rsp_tag,
    output wire [WORD_WIDTH-1:0]        core_rsp_data,
    output wire [REQ_SEL_WIDTH-1:0]     core_rsp_idx,
    input  wire                         core_rsp_ready,

    // Interface to Cache Bank
    output wire                         cache_req_valid,
    output wire [TAG_WIDTH-1:0]         cache_req_tag,
    output wire [`UP(ADDR_WIDTH)-1:0]   cache_req_addr,
    output wire                         cache_req_rw,
    output wire [WORD_SIZE-1:0]         cache_req_byteen,
    output wire [WORD_WIDTH-1:0]        cache_req_data,
    output wire [`UP(MEM_FLAGS_WIDTH)-1:0] cache_req_flags,
    output wire [WORD_SEL_WIDTH-1:0]    cache_req_wsel,
    output wire [REQ_SEL_WIDTH-1:0]     cache_req_idx,
    input  wire                         cache_req_ready,

    // Interface from Cache Bank
    input  wire                         cache_rsp_valid,
    input  wire [TAG_WIDTH-1:0]         cache_rsp_tag,
    input  wire [WORD_WIDTH-1:0]        cache_rsp_data,
    input  wire [REQ_SEL_WIDTH-1:0]     cache_rsp_idx,
    output wire                         cache_rsp_ready
);
    // AMO-related flags from the incoming request
    wire is_amo = core_req_valid && core_req_flags[MEM_REQ_FLAG_AMO];
    wire [4:0] amo_op = core_req_flags[MEM_REQ_FLAG_AMO_OP +: MEM_REQ_FLAG_AMO_OP_BITS];
    wire [OWNER_ID_WIDTH-1:0] amo_owner_id = core_req_flags[MEM_REQ_FLAG_OWNER_ID +: OWNER_ID_WIDTH];

    // State Definition
    localparam AMO_IDLE        = 0;
    localparam AMO_READ        = 1;
    localparam AMO_WAIT_READ   = 2;
    localparam AMO_WRITE       = 3;
    localparam AMO_RSP         = 4;
    localparam AMO_STATE_WIDTH = `CLOG2(AMO_RSP+1);

    logic [AMO_STATE_WIDTH-1:0] amo_state, amo_state_next;

    // Latched AMO request buffer
    struct packed {
        logic [TAG_WIDTH-1:0]         tag;
        logic [`UP(ADDR_WIDTH)-1:0]   addr;
        logic [OWNER_ID_WIDTH-1:0]    owner_id;
        logic [WORD_SIZE-1:0]         byteen;
        logic [WORD_WIDTH-1:0]        data;
        logic [WORD_SEL_WIDTH-1:0]    wsel;
        logic [REQ_SEL_WIDTH-1:0]     idx;
        logic [4:0]                   amo_op;
    } amo_req_buf;

    localparam AMO_RST_SIZE = 1;
    localparam RST_IDX_BITS = `UP(`CLOG2(AMO_RST_SIZE));
    wire [RST_IDX_BITS-1:0] rst_idx = amo_req_buf.owner_id[RST_IDX_BITS-1:0];

    // Reservation Status Table: Per bank in the AMO unit
    typedef struct packed {
        logic                      valid;
        logic [`UP(ADDR_WIDTH)-1:0]   addr;
        logic [OWNER_ID_WIDTH-1:0] owner_id;
    } rst_entry_t;

    // Table declaration
    rst_entry_t [AMO_RST_SIZE-1:0] rst;


    wire sc_addr_match     = (rst[rst_idx].addr == amo_req_buf.addr);
    wire sc_owner_id_match = (rst[rst_idx].owner_id == amo_req_buf.owner_id);
    wire sc_valid          = rst[rst_idx].valid;

    wire sc_success        = sc_valid && sc_addr_match && sc_owner_id_match;

    // FSM sequential logic
    always_ff @(posedge clk) begin
        if (reset) begin
            amo_state <= AMO_IDLE;
        end else begin
            amo_state <= amo_state_next;
        end
    end

    // FSM combinational logic
    always_comb begin
        amo_state_next = amo_state;
        case (amo_state)
            AMO_IDLE: begin
                if (core_req_valid && core_req_ready && is_amo) begin
                    case (amo_op) 
                        AMO_LR: begin
                            amo_state_next = AMO_READ;
                        end
                        AMO_SC: begin
                            amo_state_next = sc_success ? AMO_WRITE : AMO_RSP;
                        end
                        default: begin
                            // read-modify-write amo
                            amo_state_next = AMO_READ;
                        end
                    endcase
                end
            end
            
            // Read 
            AMO_READ: begin
                if (cache_req_valid && cache_req_ready) begin
                    amo_state_next = AMO_WAIT_READ;
                end
            end

            // wait for read
            AMO_WAIT_READ: begin
                if (cache_rsp_valid && cache_rsp_ready) begin
                    if (amo_req_buf.amo_op == AMO_LR) begin
                        // add entry to rst
                        amo_state_next = AMO_RSP;
                    end
                    else begin
                        // read-modify-write amo
                        amo_state_next = AMO_WRITE;
                    end
                end
            end

            AMO_WRITE: begin
                if (cache_req_valid && cache_req_ready) begin
                    amo_state_next = AMO_RSP;
                end
            end

            AMO_RSP: begin
                if (core_rsp_valid && core_rsp_ready) begin
                    amo_state_next = AMO_IDLE;
                end
            end
            default:
               amo_state_next = AMO_IDLE; 
        endcase
    end

    // Latch incoming AMO request into buffer
    always_ff @(posedge clk) begin
        if (reset) begin
            amo_req_buf <= '0;
        end else if (core_req_valid && core_req_ready && is_amo) begin
            amo_req_buf.tag        <= core_req_tag;
            amo_req_buf.addr       <= core_req_addr;
            amo_req_buf.owner_id   <= amo_owner_id;
            amo_req_buf.byteen     <= core_req_byteen;
            amo_req_buf.data       <= core_req_data;
            amo_req_buf.wsel       <= core_req_wsel;
            amo_req_buf.idx        <= core_req_idx;
            amo_req_buf.amo_op     <= amo_op;
            `ifdef SIMULATION // TODO: see if this is right
                `TRACE(2, ("%t: AMO op latched: addr=0x%0h, op=%0d, owner_id=%0d, data=0x%0h\n", $time, core_req_addr, amo_op, amo_owner_id, core_req_data))
            `endif
        end
    end

    logic [WORD_WIDTH-1:0] amo_rsp_data;
    logic [WORD_WIDTH-1:0] amo_result;

    // TODO: read-modify-write: the AMO_unit will contain an ALU
    assign amo_result = 0;

    // perform load-reserved
    always_ff @(posedge clk) begin
        if (amo_state == AMO_WAIT_READ && cache_rsp_valid && cache_rsp_ready) begin
            amo_rsp_data <= (amo_req_buf.amo_op == AMO_LR) ? cache_rsp_data : amo_result;
        end
    end

    //Snoop invalidation — check all entries in parallel
    wire [AMO_RST_SIZE-1:0] snoop_hit;

    for (genvar i = 0; i < AMO_RST_SIZE; i++) begin : g_snoop
        assign snoop_hit[i] = rst[i].valid && (rst[i].addr == cache_req_addr);
    end

    wire snoop_valid = cache_req_valid && cache_req_ready && cache_req_rw;

    // Update reservation status table (RST)
    always_ff @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < AMO_RST_SIZE; i++) begin
                rst[i].valid <= 1'b0;
            end
        end
        // create entry for lr
        else if (amo_state == AMO_WAIT_READ && cache_rsp_valid && cache_rsp_ready && amo_req_buf.amo_op == AMO_LR) begin
            rst[rst_idx].valid     <= 1'b1;
            rst[rst_idx].addr      <= amo_req_buf.addr;
            rst[rst_idx].owner_id  <= amo_req_buf.owner_id;
        end
        // invalidate rst entry after successful sc
        else if (core_req_valid && core_req_ready && is_amo && sc_success) begin
            rst[rst_idx].valid <= 1'b0;
        end

        // Snoop invalidation — overwrites LR write
        for (int i = 0; i < AMO_RST_SIZE; i++) begin
            if (snoop_hit[i] && snoop_valid) begin
                rst[i].valid <= 1'b0;
            end
        end
    end

    // Core Request Interface
    assign core_req_ready = (amo_state == AMO_IDLE) ? (is_amo ? 1'b1 : cache_req_ready) : 1'b0;

    // Core Response Interface
    assign core_rsp_valid = (amo_state == AMO_IDLE) ? cache_rsp_valid : (amo_state == AMO_RSP);
    assign core_rsp_tag   = (amo_state == AMO_IDLE) ? cache_rsp_tag : amo_req_buf.tag;
    assign core_rsp_idx   = (amo_state == AMO_IDLE) ? cache_rsp_idx : amo_req_buf.idx;
    assign core_rsp_data  = (amo_state == AMO_IDLE) ? cache_rsp_data : (amo_req_buf.amo_op == AMO_SC) ? 
                            {{WORD_WIDTH-1{1'b0}}, ~sc_success} : // SC returns 0 for success, 1 for failure
                            amo_rsp_data ; // Other AMOs return old value

    // Cache Request Interface
    assign cache_req_valid  = (amo_state == AMO_IDLE) ? (core_req_valid && !is_amo) :
                              (amo_state == AMO_READ)  ? 1'b1 :
                              (amo_state == AMO_WRITE) ? 1'b1 : 0;
    assign cache_req_addr   = (amo_state == AMO_IDLE) ? core_req_addr : amo_req_buf.addr;
    assign cache_req_rw     = (amo_state == AMO_IDLE) ? core_req_rw :
                              (amo_state == AMO_READ)  ? 0 : // Read operation
                              (amo_state == AMO_WRITE) ? (amo_req_buf.amo_op == AMO_SC ? sc_success : 1'b1) : 0; // Don't write on SC failure
    assign cache_req_byteen = (amo_state == AMO_IDLE) ? core_req_byteen : amo_req_buf.byteen;
    assign cache_req_data   = (amo_state == AMO_IDLE) ? core_req_data :
                              (amo_state == AMO_WRITE) ? amo_req_buf.data : '0;
    assign cache_req_tag    = (amo_state == AMO_IDLE) ? core_req_tag : amo_req_buf.tag;
    assign cache_req_wsel   = (amo_state == AMO_IDLE) ? core_req_wsel : amo_req_buf.wsel;
    assign cache_req_idx    = (amo_state == AMO_IDLE) ? core_req_idx : amo_req_buf.idx;
    assign cache_req_flags  = (amo_state == AMO_IDLE) ? core_req_flags : '{default:0}; // Strip AMO flags for cache bank

    // Cache Response Interface
    assign cache_rsp_ready = (amo_state == AMO_IDLE) ? core_rsp_ready : (amo_state == AMO_WAIT_READ);

`ifdef SIMULATION
    always_ff @(posedge clk) begin
        if (is_amo || amo_state != AMO_IDLE) begin
            $display("%t: [VX_amo_unit-%0d] State: %s, is_amo=%b, core_req_v=%b, core_req_r=%b, cache_req_v=%b, cache_req_r=%b, core_rsp_v=%b, core_rsp_r=%b, cache_rsp_v=%b, cache_rsp_r=%b, amo_op=%h, addr=%h, owner_id=%h, wdata=%h, rdata=%h, new_data_result=%h, sc_succ=%b",
                $time, BANK_ID, amo_state, is_amo, core_req_valid, core_req_ready, cache_req_valid, cache_req_ready, core_rsp_valid, core_rsp_ready, cache_rsp_valid, cache_rsp_ready, amo_req_buf.amo_op, amo_req_buf.addr, amo_req_buf.owner_id, amo_req_buf.data, amo_rsp_data, amo_result, sc_success);
        end
    end
`endif

endmodule
