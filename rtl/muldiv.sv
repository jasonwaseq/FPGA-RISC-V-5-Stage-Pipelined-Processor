// =============================================================================
// M extension datapath
//
//   MUL     Four 16x16 partial products are registered at the end of X (they
//           map onto the iCE40 DSP blocks, output register included). The low
//           word is summed in M with two 16-bit adders and joins the other
//           late results in W, so MUL flows through the pipeline like a load.
//   MULH*   Same partial products, summed over three more cycles while X
//           holds. The unsigned high word is corrected for signed operands.
//   DIV/REM Restoring divider, one bit per cycle, on magnitudes. Division by
//           zero and signed overflow fall out of the algorithm with the
//           results the ISA requires.
//
// `a` and `b` are the forwarded operands of the instruction in X. They stay
// stable for as long as X is held.
// =============================================================================
`default_nettype none

module muldiv (
    input  wire         clk,
    input  wire         rst,
    input  wire  [31:0] a,
    input  wire  [31:0] b,
    input  wire  [2:0]  funct3,
    input  wire         req,        // MULH* / DIV* / REM* is valid in X
    input  wire         flush,      // the instruction in X is squashed
    input  wire         advance,    // X moves on once `done` is set
    output logic        done,
    output wire  [31:0] mul_lo,     // valid in M for the MUL that just left X
    output logic [31:0] result      // MULH* / DIV* / REM*, valid with `done`
);

    // ---- Partial products (DSP) ------------------------------------------------
    logic [31:0] pp_ll, pp_lh, pp_hl, pp_hh;

    always_ff @(posedge clk) begin
        pp_ll <= a[15:0]  * b[15:0];
        pp_lh <= a[15:0]  * b[31:16];
        pp_hl <= a[31:16] * b[15:0];
        pp_hh <= a[31:16] * b[31:16];
    end

    wire [15:0] lo_hi = pp_ll[31:16] + pp_lh[15:0] + pp_hl[15:0];
    assign mul_lo = {lo_hi, pp_ll[15:0]};

    // ---- Control ---------------------------------------------------------------
    typedef enum logic [2:0] {S_IDLE, S_MUL1, S_MUL2, S_MUL3, S_DIV, S_FIN} state_t;
    state_t state;

    wire is_div    = funct3[2];
    wire is_rem    = funct3[1];
    wire is_signed = !funct3[0];             // DIV, REM

    // MULH: both signed. MULHSU: a signed. MULHU: neither.
    wire a_neg_mul = a[31] && (funct3[1:0] == 2'b01 || funct3[1:0] == 2'b10);
    wire b_neg_mul = b[31] && (funct3[1:0] == 2'b01);

    logic [33:0] mid;
    logic [31:0] hi;

    logic [31:0] rem, quo, dvs;
    logic [5:0]  cnt;
    logic        neg_q, neg_r;

    wire [31:0] a_abs = (is_signed && a[31]) ? -a : a;
    wire [31:0] b_abs = (is_signed && b[31]) ? -b : b;

    // One division step: shift the next dividend bit in, subtract if it fits.
    wire [32:0] rem_sh = {rem, quo[31]};
    wire [33:0] diff   = {1'b0, rem_sh} - {2'b00, dvs};
    wire        fits   = !diff[33];

    wire [31:0] fin_val = is_rem ? rem : quo;
    wire        fin_neg = is_rem ? neg_r : neg_q;

    always_ff @(posedge clk) begin
        if (rst || flush) begin
            state <= S_IDLE;
            done  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    if (done) begin
                        if (advance) done <= 1'b0;
                    end else if (req) begin
                        if (is_div) begin
                            state <= S_DIV;
                            rem   <= 32'd0;
                            quo   <= a_abs;
                            dvs   <= b_abs;
                            cnt   <= 6'd31;
                            neg_q <= is_signed && (a[31] != b[31]) && (b != 32'd0);
                            neg_r <= is_signed && a[31];
                        end else begin
                            state <= S_MUL1;
                        end
                    end
                end
                S_MUL1: begin
                    mid   <= {2'b00, pp_lh} + {2'b00, pp_hl} + {18'd0, pp_ll[31:16]};
                    state <= S_MUL2;
                end
                S_MUL2: begin
                    hi    <= pp_hh + {14'd0, mid[33:16]};
                    state <= S_MUL3;
                end
                S_MUL3: begin
                    result <= hi - (a_neg_mul ? b : 32'd0) - (b_neg_mul ? a : 32'd0);
                    done   <= 1'b1;
                    state  <= S_IDLE;
                end
                S_DIV: begin
                    rem <= fits ? diff[31:0] : rem_sh[31:0];
                    quo <= {quo[30:0], fits};
                    cnt <= cnt - 6'd1;
                    if (cnt == 6'd0) state <= S_FIN;
                end
                default: begin      // S_FIN
                    result <= fin_neg ? -fin_val : fin_val;
                    done   <= 1'b1;
                    state  <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
