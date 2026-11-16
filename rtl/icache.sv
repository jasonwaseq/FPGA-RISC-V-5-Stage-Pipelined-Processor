// =============================================================================
// Instruction cache: direct mapped, 32-byte lines, tags and data in block RAM.
//
// The single-ported SPRAM main memory and the serial SPI flash both sit behind
// this cache, so the core sees a one-cycle instruction memory regardless of
// where the code lives. Data accesses keep priority on the RAM port; the
// refill simply uses the cycles they leave free.
//
// Lookup is synchronous: `rd_addr` is the address that `pc` will hold in the
// next cycle, and `hit` / `instr` are valid for `pc`.
//
//   IDLE   : serve hits. A miss latches the line address and starts a refill.
//   REFILL : request the 8 words of the line through the pipelined refill
//            port, write them as they arrive, then write the tag.
//   RELOOK : one cycle to re-read the RAMs for `pc` (the core holds rd_addr at
//            pc while busy; a write and a read of the same entry in the same
//            cycle is never relied on).
//   FLUSH  : clear every tag. Entered on reset and on FENCE.I.
// =============================================================================
`default_nettype none

module icache #(
    parameter int LINES_LOG2 = 8,
    parameter int AW         = 25
) (
    input  wire                  clk,
    input  wire                  rst,
    // Lookup
    input  wire  [AW-1:2]        rd_addr,
    input  wire  [AW-1:2]        pc,
    input  wire                  req,
    input  wire                  kill,       // redirect this cycle: ignore a miss
    output wire                  hit,
    output wire  [31:0]          instr,
    output wire                  busy,
    input  wire                  flush,
    // Refill port: gnt accepts an address, rvalid returns data in order
    output wire                  rf_req,
    output wire  [AW-1:2]        rf_addr,
    input  wire                  rf_gnt,
    input  wire                  rf_rvalid,
    input  wire  [31:0]          rf_rdata,
    output wire                  ev_miss
);

    localparam int WORDS_LOG2 = 3;                          // 8 words per line
    localparam int IDXW       = LINES_LOG2 + WORDS_LOG2;    // data RAM address
    localparam int TAGW       = AW - 2 - IDXW;

    typedef enum logic [1:0] {S_FLUSH, S_IDLE, S_REFILL, S_RELOOK} state_t;
    state_t state;

    (* no_rw_check *) logic [31:0]   data_mem [0:(1 << IDXW) - 1];
    (* no_rw_check *) logic [TAGW:0] tag_mem  [0:(1 << LINES_LOG2) - 1];

    logic [31:0]   data_q;
    logic [TAGW:0] tag_q;

    logic [AW-1:2+WORDS_LOG2] line;           // line being refilled
    logic [WORDS_LOG2:0]      req_cnt;        // addresses sent
    logic [WORDS_LOG2-1:0]    wr_cnt;         // words written
    logic [LINES_LOG2-1:0]    flush_cnt;
    logic                     flush_pend;

    wire [LINES_LOG2-1:0] line_idx = line[2+WORDS_LOG2 +: LINES_LOG2];
    wire [TAGW-1:0]       pc_tag   = pc[AW-1 -: TAGW];
    wire                  last_wr  = rf_rvalid && (wr_cnt == {WORDS_LOG2{1'b1}});

    assign hit   = (state == S_IDLE) && tag_q[TAGW] && (tag_q[TAGW-1:0] == pc_tag);
    assign instr = data_q;
    assign busy  = (state != S_IDLE);

    wire miss = (state == S_IDLE) && req && !hit && !kill && !flush && !flush_pend;
    assign ev_miss = miss;

    assign rf_req  = (state == S_REFILL) && !req_cnt[WORDS_LOG2];
    assign rf_addr = {line, req_cnt[WORDS_LOG2-1:0]};

    wire sweep = (state == S_FLUSH);

    // ---- RAMs ------------------------------------------------------------------
    wire                  tag_we    = sweep || (state == S_REFILL && last_wr);
    wire [LINES_LOG2-1:0] tag_waddr = sweep ? flush_cnt : line_idx;
    wire [TAGW:0]         tag_wdata = sweep ? '0 : {1'b1, line[AW-1 -: TAGW]};

    always_ff @(posedge clk) begin
        if (tag_we) tag_mem[tag_waddr] <= tag_wdata;
        tag_q <= tag_mem[rd_addr[2+WORDS_LOG2 +: LINES_LOG2]];
    end

    always_ff @(posedge clk) begin
        if (state == S_REFILL && rf_rvalid) data_mem[{line_idx, wr_cnt}] <= rf_rdata;
        data_q <= data_mem[rd_addr[2 +: IDXW]];
    end

    // ---- Control ---------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            state      <= S_FLUSH;
            flush_cnt  <= '0;
            flush_pend <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (flush || flush_pend) begin
                        state      <= S_FLUSH;
                        flush_cnt  <= '0;
                        flush_pend <= 1'b0;
                    end else if (miss) begin
                        state   <= S_REFILL;
                        line    <= pc[AW-1:2+WORDS_LOG2];
                        req_cnt <= '0;
                        wr_cnt  <= '0;
                    end
                end
                S_REFILL: begin
                    if (flush) flush_pend <= 1'b1;
                    if (rf_req && rf_gnt) req_cnt <= req_cnt + 1'b1;
                    if (rf_rvalid) wr_cnt <= wr_cnt + 1'b1;
                    if (last_wr) state <= S_RELOOK;
                end
                S_RELOOK: begin
                    if (flush || flush_pend) begin
                        state      <= S_FLUSH;
                        flush_cnt  <= '0;
                        flush_pend <= 1'b0;
                    end else begin
                        state <= S_IDLE;
                    end
                end
                default: begin      // S_FLUSH
                    flush_cnt <= flush_cnt + 1'b1;
                    if (flush_cnt == {LINES_LOG2{1'b1}}) state <= S_RELOOK;
                end
            endcase
        end
    end

`ifndef SYNTHESIS
    initial begin
        for (int i = 0; i < (1 << LINES_LOG2); i++) tag_mem[i] = '0;
    end
`endif

endmodule

// =============================================================================
// Branch target buffer: direct mapped, 2-bit counter per entry, block RAM.
//
// Holds the target of branches, JAL and JALR. It is purely a hint: the core
// checks both the direction and the target of every prediction in X, so a
// stale entry or a partial-tag alias costs a misprediction and nothing else.
// That is what allows the short tag and needs no flush when code changes.
// =============================================================================
module btb #(
    parameter int LOG2 = 8,
    parameter int AW   = 25,
    parameter int TAGW = 6
) (
    input  wire            clk,
    input  wire  [AW-1:2]  rd_addr,
    input  wire  [AW-1:2]  pc,
    output wire            hit,
    output wire  [1:0]     ctr,
    output wire  [AW-1:2]  target,
    input  wire            upd_we,
    input  wire            upd_valid,
    input  wire  [AW-1:2]  upd_pc,
    input  wire  [AW-1:2]  upd_target,
    input  wire  [1:0]     upd_ctr
);

    localparam int TGTW = AW - 2;
    localparam int EW   = 1 + 2 + TAGW + TGTW;      // valid, counter, tag, target

    (* no_rw_check *) logic [EW-1:0] mem [0:(1 << LOG2) - 1];
    logic [EW-1:0]   q, wdata_q;
    logic [LOG2-1:0] waddr_q;
    logic            we_q;

    // The tag also folds in the address bits that tell RAM, ROM and flash
    // apart, so code in different regions does not alias onto one entry.
    function automatic [TAGW-1:0] tag_of(input [AW-1:2] a);
        tag_of = a[2 + LOG2 +: TAGW] ^ TAGW'({a[24], a[20]});
    endfunction

    wire [LOG2-1:0] waddr = upd_pc[2 +: LOG2];
    wire [EW-1:0]   wdata = {upd_valid, upd_ctr, tag_of(upd_pc), upd_target};

    // A block RAM returns undefined data for an entry that is read in the
    // cycle it is written, so that case is bypassed with a copy of the write
    // data. It is not a corner case: the branch that closes a short loop is
    // fetched again just as its own update is being written. The collision is
    // detected a cycle later, against `pc`, to keep it off the address path.
    always_ff @(posedge clk) begin
        if (upd_we) mem[waddr] <= wdata;
        q       <= mem[rd_addr[2 +: LOG2]];
        we_q    <= upd_we;
        waddr_q <= waddr;
        wdata_q <= wdata;
    end

    wire          collide = we_q && (waddr_q == pc[2 +: LOG2]);
    wire [EW-1:0] e       = collide ? wdata_q : q;

    assign hit    = e[EW-1] && (e[TGTW +: TAGW] == tag_of(pc));
    assign ctr    = e[EW-2 -: 2];
    assign target = e[TGTW-1:0];

    initial begin
        for (int i = 0; i < (1 << LOG2); i++) mem[i] = '0;
    end

endmodule

// =============================================================================
// Register file: 32 x 32, two synchronous read ports, one write port.
// Maps to block RAM. x0 is never written (the core gates the write enable) and
// the RAM powers up as zero, so it reads as zero without a mux.
// =============================================================================
module regfile (
    input  wire         clk,
    input  wire  [4:0]  ra1,
    input  wire  [4:0]  ra2,
    output logic [31:0] rd1,
    output logic [31:0] rd2,
    input  wire         we,
    input  wire  [4:0]  wa,
    input  wire  [31:0] wd
);

    (* no_rw_check *) logic [31:0] mem [0:31];

    always_ff @(posedge clk) begin
        if (we) mem[wa] <= wd;
        rd1 <= mem[ra1];
        rd2 <= mem[ra2];
    end

    initial begin
        for (int i = 0; i < 32; i++) mem[i] = 32'd0;
    end

endmodule

`default_nettype wire
