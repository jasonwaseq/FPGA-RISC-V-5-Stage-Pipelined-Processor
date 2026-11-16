// =============================================================================
// Machine-mode CSRs, trap entry/return, interrupt arbitration and counters.
//
// All state changes happen for the instruction in M (the commit stage):
//   en    CSRRW/S/C[I] commits
//   trap  exception or interrupt taken: mepc/mcause/mtval, MIE -> MPIE, MIE = 0
//   mret  MPIE -> MIE
//
// Implemented CSRs
//   mstatus (MIE, MPIE; MPP reads 11)   misa (RV32IM, read-only)
//   mie / mip (MSI, MTI, MEI)           mtvec (direct mode)
//   mscratch, mepc, mcause, mtval
//   mcycle[h], minstret[h] and their user aliases cycle/time/instret[h]
//   mhpmcounter3..6: I-cache misses, mispredictions, conditional branches,
//                    interlock stall cycles
//   mvendorid, marchid, mimpid, mhartid (zero)
// The counters are free running and read-only (writes are ignored). Any other
// address, or a write to a read-only address, is an illegal instruction; that
// check is made one stage earlier through chk_*.
// =============================================================================
`default_nettype none

module csr #(
    parameter logic [31:0] MTVEC_RESET = 32'h0010_0008
) (
    input  wire         clk,
    input  wire         rst,
    // Legality check for the CSR instruction in X
    input  wire  [11:0] chk_addr,
    input  wire         chk_wen,
    output wire         chk_ok,
    // Access by the CSR instruction committing in M
    input  wire         en,
    input  wire  [11:0] addr,
    input  wire  [1:0]  op,          // 01 write, 10 set, 11 clear
    input  wire         wen,
    input  wire  [31:0] wdata,
    output logic [31:0] rdata,
    // Trap entry / return
    input  wire         trap,
    input  wire  [4:0]  trap_cause,  // {interrupt, code}
    input  wire  [31:2] trap_pc,
    input  wire  [31:0] trap_val,
    input  wire         mret,
    output wire  [31:2] mtvec,
    output wire  [31:2] mepc,
    // Interrupts
    input  wire         irq_timer,
    input  wire         irq_soft,
    input  wire         irq_ext,
    output wire         irq_pending,
    output wire  [3:0]  irq_cause,
    // Counters
    input  wire         retire,
    input  wire  [3:0]  events,
    output wire  [63:0] cycle_count
);

    localparam logic [11:0] MSTATUS = 12'h300, MISA = 12'h301, MIE = 12'h304,
                            MTVEC = 12'h305, MSCRATCH = 12'h340, MEPC = 12'h341,
                            MCAUSE = 12'h342, MTVAL = 12'h343, MIP = 12'h344;

    function automatic logic known(input logic [11:0] a);
        case (a)
            MSTATUS, MISA, MIE, MTVEC, MSCRATCH, MEPC, MCAUSE, MTVAL, MIP,
            12'hB00, 12'hB02, 12'hB03, 12'hB04, 12'hB05, 12'hB06,
            12'hB80, 12'hB82,
            12'hC00, 12'hC01, 12'hC02, 12'hC80, 12'hC81, 12'hC82,
            12'hF11, 12'hF12, 12'hF13, 12'hF14: known = 1'b1;
            default:                            known = 1'b0;
        endcase
    endfunction

    assign chk_ok = known(chk_addr) && !(chk_wen && chk_addr[11:10] == 2'b11);

    // ---- State -------------------------------------------------------------------
    logic        mstatus_mie, mstatus_mpie;
    logic        mie_msie, mie_mtie, mie_meie;
    logic [31:2] mtvec_q, mepc_q;
    logic [31:0] mscratch_q, mtval_q;
    logic        mcause_int;
    logic [3:0]  mcause_code;
    logic [63:0] mcycle_q, minstret_q;
    logic [31:0] hpm3_q, hpm4_q, hpm5_q, hpm6_q;

    assign mtvec       = mtvec_q;
    assign mepc        = mepc_q;
    assign cycle_count = mcycle_q;

    // ---- Interrupts --------------------------------------------------------------
    wire take_ext   = irq_ext   && mie_meie;
    wire take_soft  = irq_soft  && mie_msie;
    wire take_timer = irq_timer && mie_mtie;

    assign irq_pending = mstatus_mie && (take_ext || take_soft || take_timer);
    assign irq_cause   = take_ext ? 4'd11 : take_soft ? 4'd3 : 4'd7;

    // ---- Read --------------------------------------------------------------------
    always_comb begin
        case (addr)
            MSTATUS:          rdata = {19'd0, 2'b11, 3'd0, mstatus_mpie, 3'd0, mstatus_mie, 3'd0};
            MISA:             rdata = 32'h4000_1100;                 // RV32, I + M
            MIE:              rdata = {20'd0, mie_meie, 3'd0, mie_mtie, 3'd0, mie_msie, 3'd0};
            MTVEC:            rdata = {mtvec_q, 2'b00};
            MSCRATCH:         rdata = mscratch_q;
            MEPC:             rdata = {mepc_q, 2'b00};
            MCAUSE:           rdata = {mcause_int, 27'd0, mcause_code};
            MTVAL:            rdata = mtval_q;
            MIP:              rdata = {20'd0, irq_ext, 3'd0, irq_timer, 3'd0, irq_soft, 3'd0};
            12'hB00, 12'hC00, 12'hC01: rdata = mcycle_q[31:0];
            12'hB80, 12'hC80, 12'hC81: rdata = mcycle_q[63:32];
            12'hB02, 12'hC02: rdata = minstret_q[31:0];
            12'hB82, 12'hC82: rdata = minstret_q[63:32];
            12'hB03:          rdata = hpm3_q;
            12'hB04:          rdata = hpm4_q;
            12'hB05:          rdata = hpm5_q;
            12'hB06:          rdata = hpm6_q;
            default:          rdata = 32'd0;
        endcase
    end

    // ---- Write -------------------------------------------------------------------
    logic [31:0] wval;
    always_comb begin
        case (op)
            2'b10:   wval = rdata | wdata;
            2'b11:   wval = rdata & ~wdata;
            default: wval = wdata;
        endcase
    end

    wire wr = en && wen;

    always_ff @(posedge clk) begin
        if (rst) begin
            mstatus_mie  <= 1'b0;
            mstatus_mpie <= 1'b0;
            mie_msie     <= 1'b0;
            mie_mtie     <= 1'b0;
            mie_meie     <= 1'b0;
            mtvec_q      <= MTVEC_RESET[31:2];
            mcause_int   <= 1'b0;
            mcause_code  <= 4'd0;
        end else if (trap) begin
            mstatus_mpie <= mstatus_mie;
            mstatus_mie  <= 1'b0;
            mepc_q       <= trap_pc;
            mcause_int   <= trap_cause[4];
            mcause_code  <= trap_cause[3:0];
            mtval_q      <= trap_val;
        end else if (mret) begin
            mstatus_mie  <= mstatus_mpie;
            mstatus_mpie <= 1'b1;
        end else if (wr) begin
            case (addr)
                MSTATUS: begin
                    mstatus_mie  <= wval[3];
                    mstatus_mpie <= wval[7];
                end
                MIE: begin
                    mie_msie <= wval[3];
                    mie_mtie <= wval[7];
                    mie_meie <= wval[11];
                end
                MTVEC:    mtvec_q    <= wval[31:2];
                MSCRATCH: mscratch_q <= wval;
                MEPC:     mepc_q     <= wval[31:2];
                MCAUSE: begin
                    mcause_int  <= wval[31];
                    mcause_code <= wval[3:0];
                end
                MTVAL:    mtval_q    <= wval;
                default: ;
            endcase
        end
    end

    // ---- Counters ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            mcycle_q   <= 64'd0;
            minstret_q <= 64'd0;
            hpm3_q     <= 32'd0;
            hpm4_q     <= 32'd0;
            hpm5_q     <= 32'd0;
            hpm6_q     <= 32'd0;
        end else begin
            mcycle_q <= mcycle_q + 64'd1;
            if (retire)    minstret_q <= minstret_q + 64'd1;
            if (events[0]) hpm3_q <= hpm3_q + 32'd1;
            if (events[1]) hpm4_q <= hpm4_q + 32'd1;
            if (events[2]) hpm5_q <= hpm5_q + 32'd1;
            if (events[3]) hpm6_q <= hpm6_q + 32'd1;
        end
    end

endmodule

`default_nettype wire
