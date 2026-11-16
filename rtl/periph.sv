// =============================================================================
// Peripherals: UART, machine timer, GPIO / system information
//
// Every peripheral has the same register interface: `stb` is high for one
// cycle per access, writes take effect on that cycle and `rdata` is the
// combinational read value (the SoC registers it).
// =============================================================================
`default_nettype none

// -----------------------------------------------------------------------------
// UART, 8N1, with a 512-byte receive FIFO in block RAM.
//
//   0x0 DATA    read : next received byte, bit 31 set if the FIFO is empty
//               write: transmit (ignored while the transmitter is busy)
//   0x4 STATUS  bit 0 receive data available, bit 1 transmitter ready,
//               bit 2 receive overrun (sticky, cleared by writing 1)
//   0x8 CTRL    bit 0 interrupt on receive data, bit 1 interrupt on tx ready
// -----------------------------------------------------------------------------
module uart #(
    parameter int CLK_HZ = 12_000_000,
    parameter int BAUD   = 115_200
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         stb,
    input  wire         we,
    input  wire  [3:2]  addr,
    input  wire  [31:0] wdata,
    output logic [31:0] rdata,
    output logic        irq,
    input  wire         rxd,
    output logic        txd
);

    localparam int DIV  = (CLK_HZ + BAUD / 2) / BAUD;       // clocks per bit
    localparam int DIVW = $clog2(DIV);

    // ---- Transmitter -------------------------------------------------------------
    logic [9:0]      tx_sr;
    logic [3:0]      tx_bits;
    logic [DIVW-1:0] tx_div;
    wire             tx_ready = (tx_bits == 4'd0);
    wire             tx_write = stb && we && (addr == 2'd0) && tx_ready;

    always_ff @(posedge clk) begin
        if (rst) begin
            txd     <= 1'b1;
            tx_bits <= 4'd0;
        end else if (tx_write) begin
            tx_sr   <= {1'b1, wdata[7:0], 1'b0};        // stop, data, start
            tx_bits <= 4'd10;
            tx_div  <= DIVW'(DIV - 1);
            txd     <= 1'b0;
        end else if (!tx_ready) begin
            if (tx_div == '0) begin
                tx_div  <= DIVW'(DIV - 1);
                tx_bits <= tx_bits - 4'd1;
                tx_sr   <= {1'b1, tx_sr[9:1]};
                txd     <= (tx_bits == 4'd1) ? 1'b1 : tx_sr[1];
            end else begin
                tx_div <= tx_div - 1'b1;
            end
        end
    end

    // ---- Receiver ----------------------------------------------------------------
    logic [1:0]      rx_sync;
    logic            rx_active;
    logic [3:0]      rx_bits;
    logic [DIVW-1:0] rx_div;
    logic [7:0]      rx_sr;
    logic            rx_push;
    wire             rx_in = rx_sync[1];

    always_ff @(posedge clk) begin
        rx_sync <= {rx_sync[0], rxd};
        rx_push <= 1'b0;
        if (rst) begin
            rx_active <= 1'b0;
        end else if (!rx_active) begin
            if (!rx_in) begin                           // start bit: sample mid-bit
                rx_active <= 1'b1;
                rx_div    <= DIVW'(DIV / 2 - 1);
                rx_bits   <= 4'd0;
            end
        end else if (rx_div == '0) begin
            rx_div  <= DIVW'(DIV - 1);
            rx_bits <= rx_bits + 4'd1;
            if (rx_bits == 4'd0) begin
                if (rx_in) rx_active <= 1'b0;           // glitch, not a start bit
            end else if (rx_bits == 4'd9) begin
                rx_active <= 1'b0;
                rx_push   <= rx_in;                     // drop framing errors / breaks
            end else begin
                rx_sr <= {rx_in, rx_sr[7:1]};
            end
        end else begin
            rx_div <= rx_div - 1'b1;
        end
    end

    // ---- Receive FIFO --------------------------------------------------------------
    (* no_rw_check *) logic [7:0] fifo [0:511];
    logic [9:0] wr_ptr, rd_ptr;
    logic [7:0] rx_data;
    logic       rx_valid, overrun;

    wire fifo_empty = (wr_ptr == rd_ptr);
    wire fifo_full  = (wr_ptr[8:0] == rd_ptr[8:0]) && (wr_ptr[9] != rd_ptr[9]);
    wire rx_read    = stb && !we && (addr == 2'd0) && rx_valid;
    wire fifo_pop   = !fifo_empty && (!rx_valid || rx_read);

    always_ff @(posedge clk) begin
        if (rx_push && !fifo_full) fifo[wr_ptr[8:0]] <= rx_sr;
        if (fifo_pop) rx_data <= fifo[rd_ptr[8:0]];
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr   <= '0;
            rd_ptr   <= '0;
            rx_valid <= 1'b0;
            overrun  <= 1'b0;
        end else begin
            if (rx_push) begin
                if (fifo_full) overrun <= 1'b1;
                else           wr_ptr  <= wr_ptr + 1'b1;
            end
            if (fifo_pop) begin
                rd_ptr   <= rd_ptr + 1'b1;
                rx_valid <= 1'b1;
            end else if (rx_read) begin
                rx_valid <= 1'b0;
            end
            if (stb && we && addr == 2'd1 && wdata[2]) overrun <= 1'b0;
        end
    end

    // ---- Registers -------------------------------------------------------------------
    logic irq_rx_en, irq_tx_en;

    always_ff @(posedge clk) begin
        if (rst) begin
            irq_rx_en <= 1'b0;
            irq_tx_en <= 1'b0;
        end else if (stb && we && addr == 2'd2) begin
            irq_rx_en <= wdata[0];
            irq_tx_en <= wdata[1];
        end
    end

    always_comb begin
        case (addr)
            2'd0:    rdata = {!rx_valid, 23'd0, rx_data};
            2'd1:    rdata = {29'd0, overrun, tx_ready, rx_valid};
            2'd2:    rdata = {30'd0, irq_tx_en, irq_rx_en};
            default: rdata = 32'd0;
        endcase
    end

    always_ff @(posedge clk) irq <= (irq_rx_en && rx_valid) || (irq_tx_en && tx_ready);

endmodule

// -----------------------------------------------------------------------------
// Machine timer (CLINT style). mtime is the core's cycle counter.
//
//   0x00 MTIME_LO   0x04 MTIME_HI     (read-only)
//   0x08 MTIMECMP_LO  0x0C MTIMECMP_HI
//   0x10 MSIP       bit 0: software interrupt
// The timer interrupt is pending while mtime >= mtimecmp.
// -----------------------------------------------------------------------------
module timer (
    input  wire         clk,
    input  wire         rst,
    input  wire         stb,
    input  wire         we,
    input  wire  [4:2]  addr,
    input  wire  [31:0] wdata,
    output logic [31:0] rdata,
    input  wire  [63:0] mtime,
    output logic        irq_timer,
    output logic        irq_soft
);

    logic [63:0] mtimecmp;

    always_ff @(posedge clk) begin
        if (rst) begin
            mtimecmp <= 64'hFFFF_FFFF_FFFF_FFFF;
            irq_soft <= 1'b0;
        end else if (stb && we) begin
            case (addr)
                3'd2: mtimecmp[31:0]  <= wdata;
                3'd3: mtimecmp[63:32] <= wdata;
                3'd4: irq_soft        <= wdata[0];
                default: ;
            endcase
        end
        irq_timer <= !rst && (mtime >= mtimecmp);
    end

    always_comb begin
        case (addr)
            3'd0:    rdata = mtime[31:0];
            3'd1:    rdata = mtime[63:32];
            3'd2:    rdata = mtimecmp[31:0];
            3'd3:    rdata = mtimecmp[63:32];
            3'd4:    rdata = {31'd0, irq_soft};
            default: rdata = 32'd0;
        endcase
    end

endmodule

// -----------------------------------------------------------------------------
// GPIO and system information
//
//   0x0 OUT     LEDs (bit 0: on-board red LED, bits 1-5: PMOD LEDs)
//   0x4 IN      buttons
//   0x8 CLK_HZ  system clock frequency (read-only)
//   0xC ID      "RV5S" (read-only)
// -----------------------------------------------------------------------------
module gpio #(
    parameter int CLK_HZ = 12_000_000,
    parameter int NOUT   = 6,
    parameter int NIN    = 4
) (
    input  wire             clk,
    input  wire             rst,
    input  wire             stb,
    input  wire             we,
    input  wire  [3:2]      addr,
    input  wire  [31:0]     wdata,
    output logic [31:0]     rdata,
    output logic [NOUT-1:0] gpio_out,
    input  wire  [NIN-1:0]  gpio_in
);

    logic [NIN-1:0] in_s0, in_s1;

    always_ff @(posedge clk) begin
        in_s0 <= gpio_in;
        in_s1 <= in_s0;
        if (rst)                          gpio_out <= '0;
        else if (stb && we && addr == 0)  gpio_out <= wdata[NOUT-1:0];
    end

    always_comb begin
        case (addr)
            2'd0:    rdata = 32'(gpio_out);
            2'd1:    rdata = 32'(in_s1);
            2'd2:    rdata = CLK_HZ;
            default: rdata = 32'h5256_3553;
        endcase
    end

endmodule

`default_nettype wire
