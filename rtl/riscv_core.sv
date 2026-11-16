// =============================================================================
// RV32IM_Zicsr_Zifencei 5-stage pipelined core
//   F: Fetch   D: Decode   X: Execute   M: Memory / commit   W: Writeback
//
// The stage boundaries are drawn for the iCE40UP5K, where a LUT plus its
// routing costs about 3 ns and the RAM blocks sit at the edge of the die:
// every signal that leaves X is registered, and nothing that comes out of a
// memory is used for a decision in the same cycle.
//
//   * F  - PC, instruction cache and branch target buffer. All three are block
//          RAMs addressed with the *next* PC, so a correctly predicted taken
//          branch costs no bubble.
//   * D  - decode, immediate generation, register file (block RAM, read
//          address taken straight from the I-cache output), late-result
//          interlock.
//   * X  - bypass network, ALU, barrel shifter, branch resolution, multiplier
//          partial products (DSP), iterative divider. The address adder also
//          drives the RAM directly, so a load is read speculatively here.
//   * M  - commit point. Load alignment, store, CSR access, traps, interrupts,
//          MRET, and the redirect for a misprediction found in X. Nothing
//          younger than M has changed architectural state, so every trap is
//          precise.
//   * W  - register write. W never stalls.
//
//   Hazards
//     dependent ALU ops ............ 0 cycles (bypass from M, W and post-W)
//     load / MUL / CSR -> use ...... 1 cycle  (late results are ready in W)
//     load right behind a store .... 1 cycle  (one RAM port)
//     predicted branch / jump ...... 0 cycles
//     mispredicted branch / jump ... 3 cycles (resolved in X, redirect from M)
//     MULH* 4 cycles, DIV/REM 34 cycles (X holds)
//     trap / MRET / FENCE.I ........ 3 cycles
//
//   Memory interfaces
//     rf_*  I-cache refill port (pipelined request / response)
//     dm_*  tightly coupled RAM: loads are addressed from X, stores from M,
//           read data is used in M. Never stalls.
//     db_*  everything else (ROM, flash, MMIO): req/ack, the pipeline waits
//
//   Address map assumed by the core
//     0x0000_0000 - 0x0001_FFFF  RAM (fast port), executable
//     0x0010_0000 - 0x001F_FFFF  boot ROM, read / execute
//     0x0100_0000 - 0x01FF_FFFF  SPI flash, read / execute
//     0x8000_0000 - 0x8000_FFFF  memory-mapped I/O, data only
//   Anything else raises an access fault.
// =============================================================================
`default_nettype none

module riscv_core #(
    parameter logic [31:0] RESET_PC      = 32'h0010_0000,
    parameter logic [31:0] MTVEC_RESET   = 32'h0010_0008,
    parameter bit          BPRED         = 1'b1,   // 0: static not-taken
    parameter int          IC_LINES_LOG2 = 8,      // I-cache lines (32 bytes each)
    parameter int          BTB_LOG2      = 8
) (
    input  wire         clk,
    input  wire         rst,
    // I-cache refill port
    output wire         rf_req,
    output wire  [24:2] rf_addr,
    input  wire         rf_gnt,
    input  wire         rf_rvalid,
    input  wire  [31:0] rf_rdata,
    // Write data and byte enables (shared by the fast and slow ports)
    output logic [31:0] d_wdata,
    output logic [3:0]  d_be,
    // Fast data port: RAM
    output wire         dm_en,
    output wire         dm_we,
    output wire  [16:2] dm_addr,
    output wire         dm_wbank,       // dm_addr[16] of a store
    input  wire  [31:0] dm_rdata,
    // Slow data port: ROM, flash, MMIO
    output wire  [31:0] db_addr,
    output wire         db_req,
    output wire         db_we,
    input  wire         db_ack,
    input  wire  [31:0] db_rdata,
    // Interrupts (level)
    input  wire         irq_timer,
    input  wire         irq_soft,
    input  wire         irq_ext,
    // Free-running cycle counter (mcycle), also used as mtime
    output wire  [63:0] cycle_count
);

    localparam int AW = 25;                 // fetchable address bits

    localparam logic [6:0] OP_LUI    = 7'b0110111;
    localparam logic [6:0] OP_AUIPC  = 7'b0010111;
    localparam logic [6:0] OP_JAL    = 7'b1101111;
    localparam logic [6:0] OP_JALR   = 7'b1100111;
    localparam logic [6:0] OP_BRANCH = 7'b1100011;
    localparam logic [6:0] OP_LOAD   = 7'b0000011;
    localparam logic [6:0] OP_STORE  = 7'b0100011;
    localparam logic [6:0] OP_IMM    = 7'b0010011;
    localparam logic [6:0] OP_REG    = 7'b0110011;
    localparam logic [6:0] OP_MISC   = 7'b0001111;
    localparam logic [6:0] OP_SYSTEM = 7'b1110011;

    localparam logic [3:0] EXC_FETCH_ALIGN = 4'd0;
    localparam logic [3:0] EXC_FETCH_FAULT = 4'd1;
    localparam logic [3:0] EXC_ILLEGAL     = 4'd2;
    localparam logic [3:0] EXC_BREAK       = 4'd3;
    localparam logic [3:0] EXC_LOAD_ALIGN  = 4'd4;
    localparam logic [3:0] EXC_LOAD_FAULT  = 4'd5;
    localparam logic [3:0] EXC_STORE_ALIGN = 4'd6;
    localparam logic [3:0] EXC_STORE_FAULT = 4'd7;
    localparam logic [3:0] EXC_ECALL_M     = 4'd11;

    // -------------------------------------------------------------------------
    // Pipeline control
    //   hold_m : the instruction in M is waiting for the slow data bus
    //   hold_x : X cannot advance (M is held, or the divider is busy)
    //   hold_d : D cannot advance (X is held, or late-result interlock)
    //   m_redirect : trap, MRET, FENCE.I or a misprediction, issued from M;
    //                squashes F, D and X
    // -------------------------------------------------------------------------
    logic        hold_m, hold_x, hold_d;
    logic        x_busy, x_ramwait, load_use;
    logic        m_redirect;
    logic [31:2] m_redirect_pc;

    // =========================================================================
    // FETCH
    // =========================================================================
    logic [31:2]   pc_f;
    logic [AW-1:2] rd_addr;               // lookup address for the next cycle
    wire  [31:2]   pc_f_inc = pc_f + 30'd1;
    wire           f_fault  = |pc_f[31:AW];        // outside the fetchable space

    wire           ic_hit, ic_busy, ic_miss;
    wire  [31:0]   ic_instr;
    wire           m_fencei_fire;

    wire           btb_hit;
    wire  [1:0]    btb_ctr;
    wire  [AW-1:2] btb_target;

    // Branch target buffer update, registered in EX/MEM
    wire           btb_we;
    logic [31:2]   m_pc;
    logic          m_btb_we, m_btb_valid;
    logic [1:0]    m_btb_ctr;
    logic [AW-1:2] m_btb_target;

    wire         pred_tk   = BPRED && btb_hit && btb_ctr[1] && !f_fault;
    wire [31:2]  pred_next = pred_tk ? {{(32-AW){1'b0}}, btb_target} : pc_f_inc;
    wire         f_valid   = ic_hit || f_fault;
    wire         f_fire    = f_valid && !hold_d;

    // The block RAMs must be given the address that pc_f will hold next cycle.
    // The hit signal is deliberately kept out of this path: on a miss the
    // cache goes busy and the lookup is repeated once the line is in.
    always_comb begin
        if (m_redirect)                rd_addr = m_redirect_pc[AW-1:2];
        else if (hold_d || ic_busy)    rd_addr = pc_f[AW-1:2];
        else                           rd_addr = pred_next[AW-1:2];
    end

    always_ff @(posedge clk) begin
        if (rst)             pc_f <= RESET_PC[31:2];
        else if (m_redirect) pc_f <= m_redirect_pc;
        else if (f_fire)     pc_f <= pred_next;
    end

    icache #(.LINES_LOG2(IC_LINES_LOG2), .AW(AW)) u_icache (
        .clk       (clk),
        .rst       (rst),
        .rd_addr   (rd_addr),
        .pc        (pc_f[AW-1:2]),
        .req       (!f_fault),
        .kill      (m_redirect),
        .hit       (ic_hit),
        .instr     (ic_instr),
        .busy      (ic_busy),
        .flush     (m_fencei_fire),
        .rf_req    (rf_req),
        .rf_addr   (rf_addr),
        .rf_gnt    (rf_gnt),
        .rf_rvalid (rf_rvalid),
        .rf_rdata  (rf_rdata),
        .ev_miss   (ic_miss)
    );

    generate
        if (BPRED) begin : g_btb
            btb #(.LOG2(BTB_LOG2), .AW(AW)) u_btb (
                .clk        (clk),
                .rd_addr    (rd_addr),
                .pc         (pc_f[AW-1:2]),
                .hit        (btb_hit),
                .ctr        (btb_ctr),
                .target     (btb_target),
                .upd_we     (btb_we),
                .upd_valid  (m_btb_valid),
                .upd_pc     (m_pc[AW-1:2]),
                .upd_target (m_btb_target),
                .upd_ctr    (m_btb_ctr)
            );
        end else begin : g_nobtb
            assign btb_hit    = 1'b0;
            assign btb_ctr    = 2'b00;
            assign btb_target = '0;
        end
    endgenerate

    // =========================================================================
    // IF/ID
    // =========================================================================
    logic          d_valid, d_fault, d_pred_tk, d_btb_hit;
    logic [1:0]    d_btb_ctr;
    logic [AW-1:2] d_pred_target;
    logic [31:2]   d_pc;
    logic [31:0]   d_instr;

    always_ff @(posedge clk) begin
        if (rst || m_redirect) d_valid <= 1'b0;
        else if (!hold_d)      d_valid <= f_valid;

        if (!hold_d && f_valid) begin
            d_instr       <= ic_instr;
            d_pc          <= pc_f;
            d_fault       <= f_fault;
            d_pred_tk     <= pred_tk;
            d_btb_hit     <= BPRED && btb_hit && !f_fault;
            d_btb_ctr     <= btb_ctr;
            d_pred_target <= btb_target;
        end
    end

    // =========================================================================
    // DECODE
    // =========================================================================
    wire [6:0] d_opc = d_instr[6:0];
    wire [2:0] d_f3  = d_instr[14:12];
    wire [6:0] d_f7  = d_instr[31:25];
    wire [4:0] d_rs1 = d_instr[19:15];
    wire [4:0] d_rs2 = d_instr[24:20];
    wire [4:0] d_rd  = d_instr[11:7];

    wire [31:0] imm_i = {{20{d_instr[31]}}, d_instr[31:20]};
    wire [31:0] imm_s = {{20{d_instr[31]}}, d_instr[31:25], d_instr[11:7]};
    wire [31:0] imm_b = {{20{d_instr[31]}}, d_instr[7], d_instr[30:25], d_instr[11:8], 1'b0};
    wire [31:0] imm_u = {d_instr[31:12], 12'd0};
    wire [31:0] imm_j = {{12{d_instr[31]}}, d_instr[19:12], d_instr[20], d_instr[30:21], 1'b0};

    logic        dc_legal, dc_ecall, dc_ebreak;
    logic        dc_use_rs1, dc_use_rs2, dc_reg_write;
    logic        dc_is_load, dc_is_store, dc_is_branch, dc_is_jal, dc_is_jalr;
    logic        dc_alu_imm, dc_alu_sub, dc_sra, dc_res_pc4;
    logic [2:0]  dc_alu_op;
    logic        dc_op1_pc, dc_op1_zero, dc_op1_zimm;
    logic        dc_is_csr, dc_csr_wen, dc_is_mul, dc_is_mdslow, dc_is_mret, dc_is_fencei;
    logic [31:0] dc_imm;

    always_comb begin
        dc_legal     = 1'b0;
        dc_ecall     = 1'b0;
        dc_ebreak    = 1'b0;
        dc_use_rs1   = 1'b0;
        dc_use_rs2   = 1'b0;
        dc_reg_write = 1'b0;
        dc_is_load   = 1'b0;
        dc_is_store  = 1'b0;
        dc_is_branch = 1'b0;
        dc_is_jal    = 1'b0;
        dc_is_jalr   = 1'b0;
        dc_alu_imm   = 1'b1;
        dc_alu_sub   = 1'b0;
        dc_sra       = 1'b0;
        dc_res_pc4   = 1'b0;
        dc_alu_op    = 3'b000;
        dc_op1_pc    = 1'b0;
        dc_op1_zero  = 1'b0;
        dc_op1_zimm  = 1'b0;
        dc_is_csr    = 1'b0;
        dc_csr_wen   = 1'b0;
        dc_is_mul    = 1'b0;
        dc_is_mdslow = 1'b0;
        dc_is_mret   = 1'b0;
        dc_is_fencei = 1'b0;
        dc_imm       = 32'd0;

        case (d_opc)
            OP_LUI: begin                       // 0 + imm
                dc_legal     = 1'b1;
                dc_reg_write = 1'b1;
                dc_op1_zero  = 1'b1;
                dc_imm       = imm_u;
            end
            OP_AUIPC: begin                     // pc + imm
                dc_legal     = 1'b1;
                dc_reg_write = 1'b1;
                dc_op1_pc    = 1'b1;
                dc_imm       = imm_u;
            end
            OP_JAL: begin
                dc_legal     = 1'b1;
                dc_reg_write = 1'b1;
                dc_is_jal    = 1'b1;
                dc_res_pc4   = 1'b1;
                dc_imm       = imm_j;
            end
            OP_JALR: begin
                dc_legal     = (d_f3 == 3'b000);
                dc_reg_write = 1'b1;
                dc_use_rs1   = 1'b1;
                dc_is_jalr   = 1'b1;
                dc_res_pc4   = 1'b1;
                dc_imm       = imm_i;
            end
            OP_BRANCH: begin                    // compare with the ALU subtractor
                dc_legal     = (d_f3[2:1] != 2'b01);
                dc_use_rs1   = 1'b1;
                dc_use_rs2   = 1'b1;
                dc_is_branch = 1'b1;
                dc_alu_imm   = 1'b0;
                dc_alu_sub   = 1'b1;
                dc_imm       = imm_b;
            end
            OP_LOAD: begin
                dc_legal     = (d_f3 == 3'b000) || (d_f3 == 3'b001) || (d_f3 == 3'b010) ||
                               (d_f3 == 3'b100) || (d_f3 == 3'b101);
                dc_reg_write = 1'b1;
                dc_use_rs1   = 1'b1;
                dc_is_load   = 1'b1;
                dc_imm       = imm_i;
            end
            OP_STORE: begin
                dc_legal     = (d_f3 == 3'b000) || (d_f3 == 3'b001) || (d_f3 == 3'b010);
                dc_use_rs1   = 1'b1;
                dc_use_rs2   = 1'b1;
                dc_is_store  = 1'b1;
                dc_imm       = imm_s;
            end
            OP_IMM: begin
                dc_legal     = (d_f3 == 3'b001) ? (d_f7 == 7'h00) :
                               (d_f3 == 3'b101) ? (d_f7 == 7'h00 || d_f7 == 7'h20) : 1'b1;
                dc_reg_write = 1'b1;
                dc_use_rs1   = 1'b1;
                dc_alu_op    = d_f3;
                dc_alu_sub   = (d_f3 == 3'b010) || (d_f3 == 3'b011);
                dc_sra       = d_instr[30];
                dc_imm       = imm_i;
            end
            OP_REG: begin
                dc_reg_write = 1'b1;
                dc_use_rs1   = 1'b1;
                dc_use_rs2   = 1'b1;
                dc_alu_imm   = 1'b0;
                if (d_f7 == 7'h01) begin        // M extension
                    dc_legal     = 1'b1;
                    dc_is_mul    = (d_f3 == 3'b000);
                    dc_is_mdslow = (d_f3 != 3'b000);
                end else begin
                    dc_legal   = (d_f7 == 7'h00) ||
                                 (d_f7 == 7'h20 && (d_f3 == 3'b000 || d_f3 == 3'b101));
                    dc_alu_op  = d_f3;
                    dc_alu_sub = (d_f3 == 3'b000 && d_instr[30]) ||
                                 (d_f3 == 3'b010) || (d_f3 == 3'b011);
                    dc_sra     = d_instr[30];
                end
            end
            OP_MISC: begin
                if (d_f3 == 3'b000) begin       // FENCE: no-op (single hart, in order)
                    dc_legal = 1'b1;
                end else if (d_f3 == 3'b001) begin  // FENCE.I: flush I-cache, refetch
                    dc_legal     = 1'b1;
                    dc_is_fencei = 1'b1;
                    dc_res_pc4   = 1'b1;
                end
            end
            OP_SYSTEM: begin
                if (d_f3 == 3'b000) begin
                    if (d_instr[31:7] == 25'h000_0000)      dc_ecall  = 1'b1;
                    else if (d_instr[31:7] == 25'h000_2000) dc_ebreak = 1'b1;
                    else if (d_instr[31:7] == 25'h060_4000) begin   // MRET
                        dc_legal   = 1'b1;
                        dc_is_mret = 1'b1;
                    end else if (d_instr[31:7] == 25'h020_A000) begin  // WFI: no-op
                        dc_legal = 1'b1;
                    end
                end else if (d_f3 != 3'b100) begin  // CSRRW/S/C[I]; address checked in X
                    dc_legal     = 1'b1;
                    dc_is_csr    = 1'b1;
                    dc_reg_write = 1'b1;
                    dc_use_rs1   = !d_f3[2];
                    dc_op1_zimm  = d_f3[2];
                    dc_csr_wen   = (d_f3[1:0] == 2'b01) || (d_rs1 != 5'd0);
                end
            end
            default: ;
        endcase
    end

    // An instruction that raises an exception carries no other side effects.
    wire        d_exc   = d_fault || !dc_legal;
    wire [3:0]  d_cause = d_fault   ? EXC_FETCH_FAULT :
                          dc_ecall  ? EXC_ECALL_M :
                          dc_ebreak ? EXC_BREAK : EXC_ILLEGAL;
    wire        d_ok        = !d_exc;
    wire        d_use_rs1   = d_ok && dc_use_rs1 && (d_rs1 != 5'd0);
    wire        d_use_rs2   = d_ok && dc_use_rs2 && (d_rs2 != 5'd0);
    wire        d_reg_write = d_ok && dc_reg_write && (d_rd != 5'd0);
    wire        d_late      = d_ok && (dc_is_load || dc_is_csr || dc_is_mul || dc_is_mdslow);

    // ---- Register file ------------------------------------------------------
    // Block RAM: the read address comes straight from the I-cache output (or
    // from the held instruction when D is stalled) and the data is valid in D.
    // The RAM does not see a write made on the same edge as the read. That
    // writer is in the post-W register (W2) while its reader is in D, so one
    // bypass here covers it; a writer that is still in W now will be in W2
    // when the reader reaches X, where the bypass network picks it up.
    logic        wb_we;
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;
    logic        w2_we;
    logic [4:0]  w2_rd;
    logic [31:0] w2_data;
    wire  [31:0] rf_rd1, rf_rd2;

    regfile u_regfile (
        .clk (clk),
        .ra1 (hold_d ? d_rs1 : ic_instr[19:15]),
        .ra2 (hold_d ? d_rs2 : ic_instr[24:20]),
        .rd1 (rf_rd1),
        .rd2 (rf_rd2),
        .we  (wb_we),
        .wa  (wb_rd),
        .wd  (wb_data)
    );

    always_ff @(posedge clk) begin
        w2_we   <= wb_we && !rst;
        w2_rd   <= wb_rd;
        w2_data <= wb_data;
    end

    wire [31:0] d_rs1_val = (w2_we && w2_rd == d_rs1) ? w2_data : rf_rd1;
    wire [31:0] d_rs2_val = (w2_we && w2_rd == d_rs2) ? w2_data : rf_rd2;

    // Operand 1 is preloaded for instructions that do not read rs1, so X needs
    // no operand-select mux in front of the adder.
    logic [31:0] d_op1;
    always_comb begin
        if (dc_op1_pc)        d_op1 = {d_pc, 2'b00};          // AUIPC
        else if (dc_op1_zero) d_op1 = 32'd0;                  // LUI
        else if (dc_op1_zimm) d_op1 = {27'd0, d_rs1};         // CSRR?I
        else                  d_op1 = d_rs1_val;
    end

    // =========================================================================
    // ID/EX
    // =========================================================================
    logic        x_valid;
    logic [31:2] x_pc;
    logic [31:0] x_imm, x_rs1_val, x_rs2_val;
    logic [4:0]  x_rs1, x_rs2;
    logic [2:0]  x_funct3, x_alu_op;
    logic        x_use_rs1, x_use_rs2, x_late;
    logic        x_is_load, x_is_store, x_is_branch, x_is_jal, x_is_jalr;
    logic        x_alu_imm, x_alu_sub, x_sra, x_res_pc4;
    logic        x_is_csr, x_csr_wen, x_is_mul, x_is_mdslow, x_is_mret, x_is_fencei;
    logic [11:0] x_csr_addr;
    logic        x_exc;
    logic [3:0]  x_cause;
    logic        x_pred_tk, x_btb_hit;
    logic [1:0]  x_btb_ctr;
    logic [AW-1:2] x_pred_target;

    logic [31:0] x_a, x_b;              // forwarded operands
    wire         d_to_x = d_valid && !load_use;

    // Bypass selects, worked out here and registered so that X starts with a
    // plain 4:1 mux. When this instruction is in X, today's X, M and W
    // instructions will be in M, W and the post-W register.
    //   0: value read in D   1: EX/MEM   2: MEM/WB   3: post-W
    logic [1:0] x_fwd_a, x_fwd_b;
    logic       x_reg_write, m_reg_write, w_reg_write;
    logic [4:0] x_rd, m_rd, w_rd;

    function automatic [1:0] fwd_sel(input logic use_rs, input logic [4:0] rs);
        if (!use_rs)                         fwd_sel = 2'd0;
        else if (x_reg_write && x_rd == rs)  fwd_sel = 2'd1;
        else if (m_reg_write && m_rd == rs)  fwd_sel = 2'd2;
        else if (w_reg_write && w_rd == rs)  fwd_sel = 2'd3;
        else                                 fwd_sel = 2'd0;
    endfunction

    always_ff @(posedge clk) begin
        if (rst || m_redirect) begin
            x_valid <= 1'b0;
        end else if (!hold_x) begin
            x_valid <= d_to_x;
        end

        if (rst || m_redirect || (!hold_x && !d_to_x)) begin
            x_use_rs1    <= 1'b0;
            x_use_rs2    <= 1'b0;
            x_reg_write  <= 1'b0;
            x_late       <= 1'b0;
            x_is_load    <= 1'b0;
            x_is_store   <= 1'b0;
            x_is_branch  <= 1'b0;
            x_is_jal     <= 1'b0;
            x_is_jalr    <= 1'b0;
            x_is_csr     <= 1'b0;
            x_is_mul     <= 1'b0;
            x_is_mdslow  <= 1'b0;
            x_is_mret    <= 1'b0;
            x_is_fencei  <= 1'b0;
            x_exc        <= 1'b0;
            x_pred_tk    <= 1'b0;
            x_btb_hit    <= 1'b0;
        end else if (!hold_x) begin
            x_use_rs1    <= d_use_rs1;
            x_use_rs2    <= d_use_rs2;
            x_reg_write  <= d_reg_write;
            x_late       <= d_late;
            x_is_load    <= d_ok && dc_is_load;
            x_is_store   <= d_ok && dc_is_store;
            x_is_branch  <= d_ok && dc_is_branch;
            x_is_jal     <= d_ok && dc_is_jal;
            x_is_jalr    <= d_ok && dc_is_jalr;
            x_is_csr     <= d_ok && dc_is_csr;
            x_is_mul     <= d_ok && dc_is_mul;
            x_is_mdslow  <= d_ok && dc_is_mdslow;
            x_is_mret    <= d_ok && dc_is_mret;
            x_is_fencei  <= d_ok && dc_is_fencei;
            x_exc        <= d_exc;
            x_pred_tk    <= d_pred_tk;
            x_btb_hit    <= d_btb_hit;
        end

        if (!hold_x) begin
            x_pc       <= d_pc;
            x_imm      <= dc_imm;
            x_rs1_val  <= d_op1;
            x_rs2_val  <= d_rs2_val;
            x_rs1      <= d_rs1;
            x_rs2      <= d_rs2;
            x_rd       <= d_rd;
            x_funct3   <= d_f3;
            x_alu_op   <= dc_alu_op;
            x_alu_imm  <= dc_alu_imm;
            x_alu_sub  <= dc_alu_sub;
            x_sra      <= dc_sra;
            x_res_pc4  <= dc_res_pc4;
            x_csr_wen  <= dc_csr_wen;
            x_csr_addr <= d_instr[31:20];
            x_cause    <= d_cause;
            x_btb_ctr  <= d_btb_ctr;
            x_pred_target <= d_pred_target;
            x_fwd_a    <= fwd_sel(d_use_rs1, d_rs1);
            x_fwd_b    <= fwd_sel(d_use_rs2, d_rs2);
        end else begin
            // X is held while the stages ahead keep draining: capture the
            // forwarded operands, so they stay correct once their producers
            // have moved on, and stop forwarding.
            x_rs1_val  <= x_a;
            x_rs2_val  <= x_b;
            x_fwd_a    <= 2'd0;
            x_fwd_b    <= 2'd0;
        end
    end

    // =========================================================================
    // EX/MEM and MEM/WB declarations
    // =========================================================================
    logic        m_valid;
    logic [31:0] m_result, m_store_data;
    logic [2:0]  m_funct3;
    logic        m_is_load, m_is_store;
    logic        m_is_csr, m_csr_wen, m_is_mul, m_is_mdslow, m_is_mret, m_is_fencei;
    logic [11:0] m_csr_addr;
    logic        m_exc;
    logic [3:0]  m_cause;
    logic        m_in_ram, m_is_branch;
    logic        m_pred_tk, m_taken, m_tgt_match;
    logic [31:2] m_redir_pc;

    logic        w_valid;
    logic [31:0] w_result;

    // =========================================================================
    // EXECUTE
    // =========================================================================
    // ---- Bypass network ---------------------------------------------------------
    // Every source is a register. A late result (load / MUL / DIV / CSR) is
    // never in M while its consumer is in X: the interlock in D holds the
    // consumer back one cycle, and by then the value is in w_result.
    always_comb begin
        case (x_fwd_a)
            2'd1:    x_a = m_result;
            2'd2:    x_a = w_result;
            2'd3:    x_a = w2_data;
            default: x_a = x_rs1_val;
        endcase
        case (x_fwd_b)
            2'd1:    x_b = m_result;
            2'd2:    x_b = w_result;
            2'd3:    x_b = w2_data;
            default: x_b = x_rs2_val;
        endcase
    end

    // ---- ALU --------------------------------------------------------------------
    // One adder/subtractor serves ADD, SUB, SLT[U], address generation, the JALR
    // target and the branch comparison.
    wire [31:0] alu_b   = x_alu_imm ? x_imm : x_b;
    wire [32:0] alu_sum = {1'b0, x_a} + {1'b0, alu_b ^ {32{x_alu_sub}}} + {32'd0, x_alu_sub};
    wire        alu_ltu = !alu_sum[32];                       // valid when subtracting
    wire        alu_lt  = (x_a[31] != alu_b[31]) ? x_a[31] : alu_sum[31];

    // Barrel shifter: one right shifter, left shifts are bit-reversed around it.
    function automatic [31:0] bitrev(input [31:0] v);
        for (int i = 0; i < 32; i++) bitrev[i] = v[31 - i];
    endfunction

    wire        sh_left = (x_alu_op == 3'b001);
    wire [31:0] sh_in   = sh_left ? bitrev(x_a) : x_a;
    wire [32:0] sh_ext  = {x_sra && x_a[31] && !sh_left, sh_in};
    wire [32:0] sh_res  = $signed(sh_ext) >>> alu_b[4:0];
    wire [31:0] sh_out  = sh_left ? bitrev(sh_res[31:0]) : sh_res[31:0];

    logic [31:0] alu_out;
    always_comb begin
        case (x_alu_op)
            3'b000:  alu_out = alu_sum[31:0];
            3'b010:  alu_out = {31'd0, alu_lt};
            3'b011:  alu_out = {31'd0, alu_ltu};
            3'b100:  alu_out = x_a ^ alu_b;
            3'b110:  alu_out = x_a | alu_b;
            3'b111:  alu_out = x_a & alu_b;
            default: alu_out = sh_out;                        // 001 SLL, 101 SRL/SRA
        endcase
    end

    wire [31:2] x_pc_inc = x_pc + 30'd1;
    wire [31:0] x_result = x_res_pc4 ? {x_pc_inc, 2'b00} : alu_out;

    // ---- Branch / jump resolution ---------------------------------------------
    wire br_eq = (x_a == x_b);
    logic br_cond;
    always_comb begin
        case (x_funct3)
            3'b000:  br_cond =  br_eq;
            3'b001:  br_cond = !br_eq;
            3'b100:  br_cond =  alu_lt;
            3'b101:  br_cond = !alu_lt;
            3'b110:  br_cond =  alu_ltu;
            default: br_cond = !alu_ltu;
        endcase
    end

    wire [31:0] x_pc_imm  = {x_pc, 2'b00} + x_imm;
    wire [31:0] x_target  = x_is_jalr ? {alu_sum[31:1], 1'b0} : x_pc_imm;
    wire        x_is_cf   = x_is_branch || x_is_jal || x_is_jalr;
    wire        x_taken   = x_is_jal || x_is_jalr || (x_is_branch && br_cond);

    // The branch target buffer is only a hint: a "taken" prediction is right
    // only if this is a taken control transfer to exactly the predicted
    // address. Anything else (wrong direction, wrong or stale target, an alias
    // on an instruction that is not a branch at all) is refetched. The
    // decision is registered and the redirect is issued from M.
    wire        x_tgt_fetchable = (x_target[31:AW] == '0);
    wire        x_tgt_match = x_tgt_fetchable && (x_target[AW-1:2] == x_pred_target);
    wire [31:2] x_next_pc   = x_taken ? x_target[31:2] : x_pc_inc;

    // Table update: allocate on a taken transfer, train the counter on a hit,
    // drop an entry that hit on something that is not a control transfer.
    wire [1:0] ctr_next = x_taken ? ((x_btb_ctr == 2'd3) ? 2'd3 : x_btb_ctr + 2'd1)
                                  : ((x_btb_ctr == 2'd0) ? 2'd0 : x_btb_ctr - 2'd1);
    wire       x_btb_we    = x_is_cf ? (x_taken || x_btb_hit) : x_btb_hit;
    wire       x_btb_valid = x_is_cf && x_tgt_fetchable;
    wire [1:0] x_btb_ctr_n = x_btb_hit ? ctr_next : (x_is_branch ? 2'd2 : 2'd3);

    // ---- Multiply / divide ------------------------------------------------------
    wire [31:0] mul_lo, md_result;
    wire        md_done;

    muldiv u_muldiv (
        .clk     (clk),
        .rst     (rst),
        .a       (x_a),
        .b       (x_b),
        .funct3  (x_funct3),
        .req     (x_valid && x_is_mdslow),
        .flush   (m_redirect),
        .advance (!hold_m),
        .done    (md_done),
        .mul_lo  (mul_lo),
        .result  (md_result)
    );

    assign x_busy = x_valid && x_is_mdslow && !md_done;

    // ---- RAM port -------------------------------------------------------------------
    // A load is read here, straight from the address adder, so its data can be
    // aligned in M and bypassed from a register. The read is speculative; RAM
    // reads have no side effects. A store is written from M, at commit, and
    // owns the port for that cycle: a load right behind it waits one cycle.
    wire m_store_ram = m_valid && m_is_store && m_in_ram;
    assign x_ramwait = x_valid && x_is_load && m_store_ram;
    assign dm_en     = m_store_ram || (x_valid && x_is_load);
    assign dm_addr   = m_store_ram ? m_result[16:2] : alu_sum[16:2];
    assign dm_wbank  = m_result[16];

    // ---- Exceptions found in X ---------------------------------------------------
    wire csr_chk_ok;
    wire x_csr_bad  = x_is_csr && !csr_chk_ok;
    wire x_jmp_bad  = x_taken && x_target[1];             // IALIGN = 32
    wire x_exc_out  = x_exc || x_csr_bad || x_jmp_bad;
    wire [3:0] x_cause_out = x_exc     ? x_cause :
                             x_csr_bad ? EXC_ILLEGAL : EXC_FETCH_ALIGN;

    // =========================================================================
    // HAZARD UNIT
    // =========================================================================
    // The interlock feeds the fetch address, so it uses a shallow decode of
    // its own: which register fields the opcode can read at all. It may stall
    // an instruction that did not need it (an illegal one, say) for the one
    // cycle the late producer spends in X; it never misses a real dependence.
    wire haz_rs1 = (d_rs1 != 5'd0) &&
                   !(d_opc == OP_LUI || d_opc == OP_AUIPC || d_opc == OP_JAL);
    wire haz_rs2 = (d_rs2 != 5'd0) &&
                   (d_opc == OP_BRANCH || d_opc == OP_STORE || d_opc == OP_REG);

    assign load_use = d_valid && x_valid && x_late &&
                      ((haz_rs1 && d_rs1 == x_rd) || (haz_rs2 && d_rs2 == x_rd));
    assign hold_x   = hold_m || x_busy || x_ramwait;
    assign hold_d   = hold_x || load_use;

    // =========================================================================
    // EX/MEM
    // =========================================================================
    wire x_to_m = x_valid && !x_busy && !x_ramwait;

    always_ff @(posedge clk) begin
        if (rst || m_redirect) begin
            m_valid <= 1'b0;
        end else if (!hold_m) begin
            m_valid <= x_to_m;
        end

        if (rst || m_redirect || (!hold_m && !x_to_m)) begin
            m_reg_write <= 1'b0;
            m_is_load   <= 1'b0;
            m_is_store  <= 1'b0;
            m_is_csr    <= 1'b0;
            m_is_mul    <= 1'b0;
            m_is_mdslow <= 1'b0;
            m_is_mret   <= 1'b0;
            m_is_fencei <= 1'b0;
            m_is_branch <= 1'b0;
            m_exc       <= 1'b0;
            m_pred_tk   <= 1'b0;
            m_taken     <= 1'b0;
            m_btb_we    <= 1'b0;
        end else if (!hold_m) begin
            m_reg_write <= x_reg_write;
            m_is_load   <= x_is_load;
            m_is_store  <= x_is_store;
            m_is_csr    <= x_is_csr;
            m_is_mul    <= x_is_mul;
            m_is_mdslow <= x_is_mdslow;
            m_is_mret   <= x_is_mret;
            m_is_fencei <= x_is_fencei;
            m_is_branch <= x_is_branch;
            m_exc       <= x_exc_out;
            m_pred_tk   <= x_pred_tk;
            m_taken     <= x_taken;
            m_btb_we    <= BPRED && !x_exc && x_btb_we;
        end

        if (!hold_m) begin
            m_pc         <= x_pc;
            m_result     <= x_result;
            m_store_data <= x_b;
            m_rd         <= x_rd;
            m_funct3     <= x_funct3;
            m_csr_wen    <= x_csr_wen;
            m_csr_addr   <= x_csr_addr;
            m_cause      <= x_cause_out;
            m_in_ram     <= (alu_sum[31:17] == 15'd0);
            m_redir_pc   <= x_next_pc;
            m_tgt_match  <= x_tgt_match;
            m_btb_valid  <= x_btb_valid;
            m_btb_ctr    <= x_btb_ctr_n;
            m_btb_target <= x_target[AW-1:2];
        end
    end

    // =========================================================================
    // MEMORY / COMMIT
    // =========================================================================
    // ---- Address decode and data exceptions -----------------------------------
    wire m_mem      = m_is_load || m_is_store;
    wire m_in_rom   = (m_result[31:20] == 12'h001);
    wire m_in_flash = (m_result[31:24] == 8'h01);
    wire m_in_mmio  = (m_result[31:16] == 16'h8000);
    wire m_misalign = (m_funct3[1:0] == 2'b01 && m_result[0]) ||
                      (m_funct3[1:0] == 2'b10 && m_result[1:0] != 2'b00);
    wire m_acc_bad  = !(m_in_ram || m_in_mmio || (m_is_load && (m_in_rom || m_in_flash)));
    wire m_dfault   = m_mem && (m_misalign || m_acc_bad);
    wire [3:0] m_dcause = m_misalign ? (m_is_store ? EXC_STORE_ALIGN : EXC_LOAD_ALIGN)
                                     : (m_is_store ? EXC_STORE_FAULT : EXC_LOAD_FAULT);

    // ---- Traps -------------------------------------------------------------------
    // Interrupts are taken on the instruction in M, which is squashed and
    // restarted after the handler. Not once its bus access has started.
    logic       slow_done, slow_started;
    logic [31:0] slow_rdata;
    wire        irq_pending;
    wire [3:0]  irq_cause;
    wire [31:2] csr_mtvec, csr_mepc;
    wire [31:0] csr_rdata;

    wire m_irq   = irq_pending && !slow_started && !slow_done;
    wire m_trap  = m_valid && (m_irq || m_exc || m_dfault);
    wire m_mret_fire  = m_valid && m_is_mret && !m_trap;
    assign m_fencei_fire = m_valid && m_is_fencei && !m_trap;

    wire [4:0]  m_trap_cause = m_irq ? {1'b1, irq_cause} :
                               m_exc ? {1'b0, m_cause}   : {1'b0, m_dcause};
    wire [31:0] m_trap_val   = m_irq ? 32'd0 :
                               m_exc ? ((m_cause == EXC_FETCH_FAULT) ? {m_pc, 2'b00} : 32'd0)
                                     : m_result;

    // Misprediction: the direction or the target was wrong. FENCE.I refetches
    // what follows it the same way.
    // An instruction that waits on the slow bus redirects only once it is done:
    // a stale prediction on an MMIO store must not squash the store itself.
    wire m_mispred = m_pred_tk ? !(m_taken && m_tgt_match) : m_taken;
    wire m_redir   = m_valid && (m_mispred || m_is_fencei) && !hold_m;

    assign m_redirect    = m_trap || m_mret_fire || m_redir;
    assign m_redirect_pc = m_trap ? csr_mtvec : m_mret_fire ? csr_mepc : m_redir_pc;

    assign btb_we = m_valid && m_btb_we;

    // ---- Data ports ----------------------------------------------------------------
    assign db_addr = m_result;

    always_comb begin
        case (m_funct3[1:0])
            2'b00: begin
                d_wdata = {4{m_store_data[7:0]}};
                d_be    = 4'b0001 << m_result[1:0];
            end
            2'b01: begin
                d_wdata = {2{m_store_data[15:0]}};
                d_be    = m_result[1] ? 4'b1100 : 4'b0011;
            end
            default: begin
                d_wdata = m_store_data;
                d_be    = 4'b1111;
            end
        endcase
    end

    // Fast port: only the write enable depends on the trap logic.
    assign dm_we = m_store_ram && !m_trap;

    // Slow port: hold M until the access is acknowledged. The request is a
    // register: the first cycle in M decides whether the instruction traps,
    // and from then on it cannot be interrupted.
    wire m_slow = m_valid && m_mem && !m_in_ram;
    assign db_req = slow_started && !slow_done;
    assign db_we  = m_is_store;
    // No trap term here: a trap can only hit the first cycle of the access,
    // and its redirect overrides the hold everywhere it matters.
    assign hold_m = m_slow && !slow_done;

    always_ff @(posedge clk) begin
        if (rst) begin
            slow_done    <= 1'b0;
            slow_started <= 1'b0;
        end else begin
            slow_done <= 1'b0;
            if (db_req && db_ack) begin
                slow_done    <= 1'b1;
                slow_started <= 1'b0;
                slow_rdata   <= db_rdata;
            end else if (m_slow && !m_trap && !slow_done) begin
                slow_started <= 1'b1;
            end
        end
    end

    // ---- Load alignment (shared function, used in M for slow loads and in W) --
    function automatic [31:0] load_align(input [31:0] word, input [1:0] off, input [2:0] f3);
        logic [7:0]  b;
        logic [15:0] h;
        begin
            case (off)
                2'd0:    b = word[7:0];
                2'd1:    b = word[15:8];
                2'd2:    b = word[23:16];
                default: b = word[31:24];
            endcase
            h = off[1] ? word[31:16] : word[15:0];
            case (f3)
                3'b000:  load_align = {{24{b[7]}}, b};
                3'b001:  load_align = {{16{h[15]}}, h};
                3'b100:  load_align = {24'd0, b};
                3'b101:  load_align = {16'd0, h};
                default: load_align = word;
            endcase
        end
    endfunction

    // ---- CSRs --------------------------------------------------------------------
    // Instructions are counted as they commit, so a read of minstret sees
    // exactly the instructions that precede it.
    wire m_commit = m_valid && !hold_m && !m_trap;
    wire [3:0] perf_events;

    csr #(.MTVEC_RESET(MTVEC_RESET)) u_csr (
        .clk         (clk),
        .rst         (rst),
        .chk_addr    (x_csr_addr),
        .chk_wen     (x_csr_wen),
        .chk_ok      (csr_chk_ok),
        .en          (m_valid && m_is_csr && !m_trap),
        .addr        (m_csr_addr),
        .op          (m_funct3[1:0]),
        .wen         (m_csr_wen),
        .wdata       (m_result),
        .rdata       (csr_rdata),
        .trap        (m_trap),
        .trap_cause  (m_trap_cause),
        .trap_pc     (m_pc),
        .trap_val    (m_trap_val),
        .mret        (m_mret_fire),
        .mtvec       (csr_mtvec),
        .mepc        (csr_mepc),
        .irq_timer   (irq_timer),
        .irq_soft    (irq_soft),
        .irq_ext     (irq_ext),
        .irq_pending (irq_pending),
        .irq_cause   (irq_cause),
        .retire      (m_commit),
        .events      (perf_events),
        .cycle_count (cycle_count)
    );

    // Performance events: I-cache miss, misprediction, conditional branch,
    // interlock stall cycle.
    assign perf_events = {load_use && !hold_x,
                          m_valid && m_is_branch && !m_trap,
                          m_valid && m_mispred && !m_trap,
                          ic_miss};

    // =========================================================================
    // MEM/WB
    // =========================================================================
    // Late results are merged here so W holds a single registered value.
    wire [31:0] m_ldata = load_align(m_in_ram ? dm_rdata : slow_rdata, m_result[1:0], m_funct3);

    logic [31:0] m_wdata;
    always_comb begin
        if (m_is_csr)         m_wdata = csr_rdata;
        else if (m_is_mul)    m_wdata = mul_lo;
        else if (m_is_mdslow) m_wdata = md_result;
        else if (m_is_load)   m_wdata = m_ldata;
        else                  m_wdata = m_result;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            w_valid     <= 1'b0;
            w_reg_write <= 1'b0;
        end else begin
            w_valid     <= m_commit;
            w_reg_write <= m_commit && m_reg_write;
        end
        w_rd     <= m_rd;
        w_result <= m_wdata;
    end

    // =========================================================================
    // WRITEBACK
    // =========================================================================
    assign wb_we   = w_reg_write;
    assign wb_rd   = w_rd;
    assign wb_data = w_result;

endmodule

`default_nettype wire
