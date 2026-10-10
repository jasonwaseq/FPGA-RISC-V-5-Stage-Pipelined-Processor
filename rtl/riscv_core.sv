// =============================================================================
// RV32I 5-Stage Pipelined Processor
//   F: Fetch   D: Decode   X: Execute   M: Memory   W: Writeback
//
// Microarchitecture
//   * In-order, single-issue, static not-taken branch prediction
//   * Full bypass network: EX/MEM -> X and MEM/WB -> X forwarding,
//     plus W -> D write-through on the register file
//   * Hazard unit: 1-cycle load-use interlock; branches/jumps resolve in X
//     and flush the two younger instructions (2-cycle taken-branch penalty)
//   * Instruction and data memories are synchronous (iCE40 BRAM):
//       - imem is addressed with the *next* PC so the instruction for
//         f_pc is on imem_data during the F cycle
//       - dmem is addressed in M and the load data is used in W
//   * EBREAK/ECALL stop fetch, drain the pipeline and freeze the core
//   * Performance counters: cycles and retired instructions (for CPI)
//
// Parameter FORWARDING = 0 replaces the bypass network with a pure interlock
// (stall until the producer writes back). It exists only to measure what the
// forwarding paths buy; the default build uses FORWARDING = 1.
// =============================================================================

module riscv_core #(
    parameter bit FORWARDING = 1'b1
) (
    input  logic        clk,
    input  logic        rst_n,       // Pipeline reset (also used for halt)
    input  logic        rf_rst_n,    // Register file reset (only on hard reset)
    // Instruction memory interface (synchronous read)
    output logic [31:0] imem_addr,
    input  logic [31:0] imem_data,
    // Data memory interface (synchronous read)
    output logic [31:0] dmem_addr,
    output logic [31:0] dmem_wdata,
    input  logic [31:0] dmem_rdata,
    output logic        dmem_we,
    output logic [3:0]  dmem_be,
    // Debug interface (register reads are valid while the pipeline is held
    // in reset, i.e. while the processor is halted)
    input  logic [4:0]  dbg_reg_addr,
    output logic [31:0] dbg_reg_data,
    // Status / performance counters
    output logic        halted,
    output logic [31:0] perf_cycles,
    output logic [31:0] perf_instret
);

    // -------------------------------------------------------------------------
    // Opcodes
    // -------------------------------------------------------------------------
    localparam logic [6:0] OP_LUI    = 7'b0110111;
    localparam logic [6:0] OP_AUIPC  = 7'b0010111;
    localparam logic [6:0] OP_JAL    = 7'b1101111;
    localparam logic [6:0] OP_JALR   = 7'b1100111;
    localparam logic [6:0] OP_BRANCH = 7'b1100011;
    localparam logic [6:0] OP_LOAD   = 7'b0000011;
    localparam logic [6:0] OP_STORE  = 7'b0100011;
    localparam logic [6:0] OP_IMM    = 7'b0010011;
    localparam logic [6:0] OP_REG    = 7'b0110011;
    localparam logic [6:0] OP_SYSTEM = 7'b1110011;

    localparam logic [31:0] NOP = 32'h0000_0013;   // addi x0, x0, 0

    // Result source selected in X
    typedef enum logic [1:0] {RES_ALU, RES_IMM, RES_PC4} res_t;

    // -------------------------------------------------------------------------
    // Hazard / control signals (driven further down)
    // -------------------------------------------------------------------------
    logic        stall;        // load-use (or no-forwarding) interlock
    logic        redirect;     // taken branch / jump resolved in X
    logic [31:0] redirect_pc;
    logic        stop;         // EBREAK/ECALL has passed X: fetch is stopped
    logic        sys_x;        // EBREAK/ECALL in X this cycle: kill younger

    // =========================================================================
    // Register file (2 read ports + write-through, debug read shares port 2)
    // =========================================================================
    logic [31:0] regs [0:31];
    logic [4:0]  wb_rd;
    logic [31:0] wb_data;
    logic        wb_we;

    always_ff @(posedge clk) begin
        if (!rf_rst_n) begin
            for (int i = 0; i < 32; i++) regs[i] <= 32'd0;
        end else if (wb_we) begin
            regs[wb_rd] <= wb_data;
        end
    end

    // =========================================================================
    // FETCH
    // =========================================================================
    logic [31:0] f_pc, f_pc_next;

    always_comb begin
        if (redirect)             f_pc_next = redirect_pc;
        else if (stall || stop || sys_x) f_pc_next = f_pc;
        else                      f_pc_next = f_pc + 32'd4;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) f_pc <= 32'd0;
        else        f_pc <= f_pc_next;
    end

    // imem is synchronous: present the address of the instruction we want on
    // imem_data in the *next* cycle. Held at 0 during reset so the first
    // instruction is ready when reset is released.
    assign imem_addr = rst_n ? f_pc_next : 32'd0;

    // =========================================================================
    // IF/ID
    // =========================================================================
    logic        d_valid;
    logic [31:0] d_pc, d_instr;

    always_ff @(posedge clk) begin
        if (!rst_n || redirect || stop || sys_x) begin
            d_valid <= 1'b0;
            d_instr <= NOP;
            d_pc    <= 32'd0;
        end else if (!stall) begin
            d_valid <= 1'b1;
            d_instr <= imem_data;
            d_pc    <= f_pc;
        end
    end

    // =========================================================================
    // DECODE
    // =========================================================================
    logic [6:0]  d_opcode;
    logic [2:0]  d_funct3;
    logic [4:0]  d_rs1, d_rs2, d_rd;
    logic [31:0] d_imm;
    logic        d_use_rs1, d_use_rs2, d_reg_write;
    logic        d_is_load, d_is_store, d_is_branch, d_is_jal, d_is_jalr;
    logic        d_is_sys, d_alu_imm, d_sub_sra;
    res_t        d_res_sel;
    logic        d_pc_base;        // ALU operand A = PC (AUIPC)
    logic [31:0] d_rs1_val, d_rs2_val;

    assign d_opcode = d_instr[6:0];
    assign d_funct3 = d_instr[14:12];
    assign d_rs1    = d_instr[19:15];
    assign d_rs2    = d_instr[24:20];
    assign d_rd     = d_instr[11:7];

    always_comb begin
        d_use_rs1   = 1'b0;
        d_use_rs2   = 1'b0;
        d_reg_write = 1'b0;
        d_is_load   = 1'b0;
        d_is_store  = 1'b0;
        d_is_branch = 1'b0;
        d_is_jal    = 1'b0;
        d_is_jalr   = 1'b0;
        d_is_sys    = 1'b0;
        d_alu_imm   = 1'b1;
        d_pc_base   = 1'b0;
        d_sub_sra   = 1'b0;
        d_res_sel   = RES_ALU;
        d_imm       = 32'd0;

        case (d_opcode)
            OP_LUI: begin
                d_reg_write = 1'b1;
                d_res_sel   = RES_IMM;
                d_imm       = {d_instr[31:12], 12'd0};
            end
            OP_AUIPC: begin
                d_reg_write = 1'b1;
                d_pc_base   = 1'b1;
                d_imm       = {d_instr[31:12], 12'd0};
            end
            OP_JAL: begin
                d_reg_write = 1'b1;
                d_is_jal    = 1'b1;
                d_res_sel   = RES_PC4;
                d_imm       = {{12{d_instr[31]}}, d_instr[19:12], d_instr[20],
                               d_instr[30:21], 1'b0};
            end
            OP_JALR: begin
                d_reg_write = 1'b1;
                d_use_rs1   = 1'b1;
                d_is_jalr   = 1'b1;
                d_res_sel   = RES_PC4;
                d_imm       = {{20{d_instr[31]}}, d_instr[31:20]};
            end
            OP_BRANCH: begin
                d_use_rs1   = 1'b1;
                d_use_rs2   = 1'b1;
                d_is_branch = 1'b1;
                d_imm       = {{20{d_instr[31]}}, d_instr[7], d_instr[30:25],
                               d_instr[11:8], 1'b0};
            end
            OP_LOAD: begin
                d_reg_write = 1'b1;
                d_use_rs1   = 1'b1;
                d_is_load   = 1'b1;
                d_imm       = {{20{d_instr[31]}}, d_instr[31:20]};
            end
            OP_STORE: begin
                d_use_rs1   = 1'b1;
                d_use_rs2   = 1'b1;
                d_is_store  = 1'b1;
                d_imm       = {{20{d_instr[31]}}, d_instr[31:25], d_instr[11:7]};
            end
            OP_IMM: begin
                d_reg_write = 1'b1;
                d_use_rs1   = 1'b1;
                d_imm       = {{20{d_instr[31]}}, d_instr[31:20]};
                d_sub_sra   = (d_funct3 == 3'b101) && d_instr[30];   // SRAI
            end
            OP_REG: begin
                d_reg_write = 1'b1;
                d_use_rs1   = 1'b1;
                d_use_rs2   = 1'b1;
                d_alu_imm   = 1'b0;
                d_sub_sra   = d_instr[30];                           // SUB / SRA
            end
            OP_SYSTEM: begin
                d_is_sys    = (d_funct3 == 3'b000);                  // ECALL / EBREAK
            end
            default: ;  // FENCE and unknown opcodes execute as NOPs
        endcase

        if (d_rd == 5'd0) d_reg_write = 1'b0;
        if (d_rs1 == 5'd0) d_use_rs1  = 1'b0;   // x0 never causes a hazard
        if (d_rs2 == 5'd0) d_use_rs2  = 1'b0;
    end

    // Register read with W -> D write-through. Port 2 doubles as the debug
    // read port whenever the core is not running: held in reset by the host,
    // or stopped by EBREAK (only bubbles remain in the pipeline then).
    logic [4:0] rf_raddr2;
    assign rf_raddr2 = (rst_n && !halted) ? d_rs2 : dbg_reg_addr;

    always_comb begin
        d_rs1_val = (d_rs1 == 5'd0) ? 32'd0 : regs[d_rs1];
        d_rs2_val = (rf_raddr2 == 5'd0) ? 32'd0 : regs[rf_raddr2];
        if (wb_we && wb_rd == d_rs1)     d_rs1_val = wb_data;
        if (wb_we && wb_rd == rf_raddr2) d_rs2_val = wb_data;
    end

    assign dbg_reg_data = d_rs2_val;

    // =========================================================================
    // ID/EX
    // =========================================================================
    logic        x_valid;
    logic [31:0] x_pc, x_imm, x_rs1_val, x_rs2_val;
    logic [4:0]  x_rs1, x_rs2, x_rd;
    logic [2:0]  x_funct3;
    logic        x_use_rs1, x_use_rs2, x_reg_write;
    logic        x_is_load, x_is_store, x_is_branch, x_is_jal, x_is_jalr, x_is_sys;
    logic        x_alu_imm, x_pc_base, x_sub_sra;
    res_t        x_res_sel;

    // A bubble is inserted on a load-use stall or when the D instruction is on
    // the wrong path of a taken branch.
    logic d_to_x_valid;
    assign d_to_x_valid = d_valid && !stall && !redirect && !sys_x;

    always_ff @(posedge clk) begin
        if (!rst_n || !d_to_x_valid) begin
            x_valid     <= 1'b0;
            x_reg_write <= 1'b0;
            x_is_load   <= 1'b0;
            x_is_store  <= 1'b0;
            x_is_branch <= 1'b0;
            x_is_jal    <= 1'b0;
            x_is_jalr   <= 1'b0;
            x_is_sys    <= 1'b0;
            x_use_rs1   <= 1'b0;
            x_use_rs2   <= 1'b0;
            x_rd        <= 5'd0;
        end else begin
            x_valid     <= 1'b1;
            x_reg_write <= d_reg_write;
            x_is_load   <= d_is_load;
            x_is_store  <= d_is_store;
            x_is_branch <= d_is_branch;
            x_is_jal    <= d_is_jal;
            x_is_jalr   <= d_is_jalr;
            x_is_sys    <= d_is_sys;
            x_use_rs1   <= d_use_rs1;
            x_use_rs2   <= d_use_rs2;
            x_rd        <= d_rd;
        end
        // Datapath fields need no reset
        x_pc      <= d_pc;
        x_imm     <= d_imm;
        x_rs1_val <= d_rs1_val;
        x_rs2_val <= d_rs2_val;
        x_rs1     <= d_rs1;
        x_rs2     <= d_rs2;
        x_funct3  <= d_funct3;
        x_alu_imm <= d_alu_imm;
        x_pc_base <= d_pc_base;
        x_sub_sra <= d_sub_sra;
        x_res_sel <= d_res_sel;
    end

    // =========================================================================
    // EX/MEM and MEM/WB declarations (needed by the forwarding network)
    // =========================================================================
    logic        m_valid, m_reg_write, m_is_load, m_is_store;
    logic [4:0]  m_rd;
    logic [2:0]  m_funct3;
    logic [31:0] m_result, m_store_data;

    logic        w_valid, w_reg_write, w_is_load, w_is_sys;
    logic [4:0]  w_rd;
    logic [2:0]  w_funct3;
    logic [31:0] w_result;
    logic [31:0] w_load_data;

    // =========================================================================
    // EXECUTE
    // =========================================================================
    // ---- Forwarding network -------------------------------------------------
    // Priority: youngest producer first (EX/MEM, then MEM/WB). A load in EX/MEM
    // never needs forwarding here because the hazard unit has already stalled.
    logic [1:0]  fwd_a, fwd_b;          // 0: ID/EX value, 1: EX/MEM, 2: MEM/WB
    logic [31:0] x_a, x_b;              // forwarded rs1 / rs2 values

    always_comb begin
        fwd_a = 2'd0;
        fwd_b = 2'd0;
        if (FORWARDING) begin
            if (x_use_rs1 && m_reg_write && !m_is_load && m_rd == x_rs1) fwd_a = 2'd1;
            else if (x_use_rs1 && w_reg_write && w_rd == x_rs1)          fwd_a = 2'd2;
            if (x_use_rs2 && m_reg_write && !m_is_load && m_rd == x_rs2) fwd_b = 2'd1;
            else if (x_use_rs2 && w_reg_write && w_rd == x_rs2)          fwd_b = 2'd2;
        end
    end

    always_comb begin
        case (fwd_a)
            2'd1:    x_a = m_result;
            2'd2:    x_a = wb_data;
            default: x_a = x_rs1_val;
        endcase
        case (fwd_b)
            2'd1:    x_b = m_result;
            2'd2:    x_b = wb_data;
            default: x_b = x_rs2_val;
        endcase
    end

    // ---- ALU ------------------------------------------------------------------
    logic [31:0] alu_a, alu_b, alu_out;
    logic [4:0]  shamt;
    logic        is_alu_op;     // R-type / I-type arithmetic (funct3 selects op)

    assign is_alu_op = !(x_is_load || x_is_store || x_is_branch || x_is_jal ||
                         x_is_jalr || x_pc_base);
    assign alu_a = x_pc_base ? x_pc : x_a;
    assign alu_b = x_alu_imm ? x_imm : x_b;
    assign shamt = alu_b[4:0];

    always_comb begin
        if (!is_alu_op) begin
            alu_out = alu_a + alu_b;                       // address / AUIPC
        end else begin
            case (x_funct3)
                3'b000:  alu_out = (x_sub_sra && !x_alu_imm) ? alu_a - alu_b
                                                             : alu_a + alu_b;
                3'b001:  alu_out = alu_a << shamt;
                3'b010:  alu_out = {31'd0, $signed(alu_a) < $signed(alu_b)};
                3'b011:  alu_out = {31'd0, alu_a < alu_b};
                3'b100:  alu_out = alu_a ^ alu_b;
                3'b101:  alu_out = x_sub_sra ? 32'($signed(alu_a) >>> shamt)
                                             : alu_a >> shamt;
                3'b110:  alu_out = alu_a | alu_b;
                default: alu_out = alu_a & alu_b;
            endcase
        end
    end

    // ---- Branch / jump resolution --------------------------------------------
    logic br_eq, br_lt, br_ltu, br_cond;
    assign br_eq  = (x_a == x_b);
    assign br_lt  = ($signed(x_a) < $signed(x_b));
    assign br_ltu = (x_a < x_b);

    always_comb begin
        case (x_funct3)
            3'b000:  br_cond =  br_eq;    // BEQ
            3'b001:  br_cond = !br_eq;    // BNE
            3'b100:  br_cond =  br_lt;    // BLT
            3'b101:  br_cond = !br_lt;    // BGE
            3'b110:  br_cond =  br_ltu;   // BLTU
            3'b111:  br_cond = !br_ltu;   // BGEU
            default: br_cond = 1'b0;
        endcase
    end

    logic [31:0] x_pc_plus_imm, x_jalr_target;
    assign x_pc_plus_imm = x_pc + x_imm;
    assign x_jalr_target = (x_a + x_imm) & ~32'd1;

    assign redirect    = x_valid && (x_is_jal || x_is_jalr || (x_is_branch && br_cond));
    assign redirect_pc = x_is_jalr ? x_jalr_target : x_pc_plus_imm;

    // ---- Result ---------------------------------------------------------------
    logic [31:0] x_result;
    always_comb begin
        case (x_res_sel)
            RES_IMM: x_result = x_imm;
            RES_PC4: x_result = x_pc + 32'd4;
            default: x_result = alu_out;
        endcase
    end

    // ---- Stop on EBREAK/ECALL -------------------------------------------------
    // The two younger instructions are squashed and fetch stops; the older
    // ones drain normally and the core halts when EBREAK retires.
    assign sys_x = x_valid && x_is_sys;

    always_ff @(posedge clk) begin
        if (!rst_n)     stop <= 1'b0;
        else if (sys_x) stop <= 1'b1;
    end

    // =========================================================================
    // HAZARD UNIT
    // =========================================================================
    // With forwarding: only a load followed immediately by a consumer stalls
    // (load data is not available until W).
    // Without forwarding: stall while any producer is still in X or M; the
    // W -> D write-through covers the producer in W.
    logic load_use, raw_x, raw_m;
    assign load_use = x_valid && x_is_load && x_reg_write &&
                      ((d_use_rs1 && d_rs1 == x_rd) || (d_use_rs2 && d_rs2 == x_rd));
    assign raw_x    = x_reg_write &&
                      ((d_use_rs1 && d_rs1 == x_rd) || (d_use_rs2 && d_rs2 == x_rd));
    assign raw_m    = m_reg_write &&
                      ((d_use_rs1 && d_rs1 == m_rd) || (d_use_rs2 && d_rs2 == m_rd));

    assign stall = d_valid && (FORWARDING ? load_use : (raw_x || raw_m));

    // =========================================================================
    // EX/MEM
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n || !x_valid) begin
            m_valid     <= 1'b0;
            m_reg_write <= 1'b0;
            m_is_load   <= 1'b0;
            m_is_store  <= 1'b0;
            m_rd        <= 5'd0;
        end else begin
            m_valid     <= 1'b1;
            m_reg_write <= x_reg_write;
            m_is_load   <= x_is_load;
            m_is_store  <= x_is_store;
            m_rd        <= x_rd;
        end
        m_result     <= x_result;
        m_store_data <= x_b;
        m_funct3     <= x_funct3;
    end

    // m_is_sys travels with the instruction so W can count it as retired
    logic m_is_sys;
    always_ff @(posedge clk) begin
        if (!rst_n || !x_valid) m_is_sys <= 1'b0;
        else                    m_is_sys <= x_is_sys;
    end

    // =========================================================================
    // MEMORY
    // =========================================================================
    assign dmem_addr = m_result;
    assign dmem_we   = m_valid && m_is_store;

    always_comb begin
        case (m_funct3[1:0])
            2'b00: begin                                    // SB
                dmem_wdata = {4{m_store_data[7:0]}};
                dmem_be    = 4'b0001 << m_result[1:0];
            end
            2'b01: begin                                    // SH
                dmem_wdata = {2{m_store_data[15:0]}};
                dmem_be    = m_result[1] ? 4'b1100 : 4'b0011;
            end
            default: begin                                  // SW
                dmem_wdata = m_store_data;
                dmem_be    = 4'b1111;
            end
        endcase
    end

    // =========================================================================
    // MEM/WB
    // =========================================================================
    logic [1:0] w_byte_off;

    always_ff @(posedge clk) begin
        if (!rst_n || !m_valid) begin
            w_valid     <= 1'b0;
            w_reg_write <= 1'b0;
            w_is_load   <= 1'b0;
            w_is_sys    <= 1'b0;
            w_rd        <= 5'd0;
        end else begin
            w_valid     <= 1'b1;
            w_reg_write <= m_reg_write;
            w_is_load   <= m_is_load;
            w_is_sys    <= m_is_sys;
            w_rd        <= m_rd;
        end
        w_result   <= m_result;
        w_funct3   <= m_funct3;
        w_byte_off <= m_result[1:0];
    end

    // =========================================================================
    // WRITEBACK
    // =========================================================================
    // dmem_rdata holds the word addressed during M (synchronous BRAM read)
    logic [7:0]  ld_byte;
    logic [15:0] ld_half;

    always_comb begin
        case (w_byte_off)
            2'd0:    ld_byte = dmem_rdata[7:0];
            2'd1:    ld_byte = dmem_rdata[15:8];
            2'd2:    ld_byte = dmem_rdata[23:16];
            default: ld_byte = dmem_rdata[31:24];
        endcase
        ld_half = w_byte_off[1] ? dmem_rdata[31:16] : dmem_rdata[15:0];

        case (w_funct3)
            3'b000:  w_load_data = {{24{ld_byte[7]}}, ld_byte};    // LB
            3'b001:  w_load_data = {{16{ld_half[15]}}, ld_half};   // LH
            3'b100:  w_load_data = {24'd0, ld_byte};               // LBU
            3'b101:  w_load_data = {16'd0, ld_half};               // LHU
            default: w_load_data = dmem_rdata;                     // LW
        endcase
    end

    assign wb_we   = w_reg_write;
    assign wb_rd   = w_rd;
    assign wb_data = w_is_load ? w_load_data : w_result;

    // =========================================================================
    // Halt + performance counters
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n)                   halted <= 1'b0;
        else if (w_valid && w_is_sys) halted <= 1'b1;
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            perf_cycles  <= 32'd0;
            perf_instret <= 32'd0;
        end else if (!halted) begin
            perf_cycles  <= perf_cycles + 32'd1;
            if (w_valid) perf_instret <= perf_instret + 32'd1;
        end
    end

endmodule
