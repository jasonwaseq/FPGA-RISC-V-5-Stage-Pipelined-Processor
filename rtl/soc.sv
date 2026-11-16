// =============================================================================
// System on chip: core + memory subsystem + peripherals
//
//   0x0000_0000  128 KB RAM      SPRAM, fast data port, I-cache refill
//   0x0010_0000  2 KB boot ROM   block RAM
//   0x0100_0000  16 MB SPI flash read-only, execute in place
//   0x8000_0000  UART            0x8000_1000 timer     0x8000_2000 GPIO / info
//
// Memory ports and who uses them
//
//                    +---------+   refill    +-----------------------------+
//      fetch ------> | I-cache | ----------> | RAM | ROM | flash           |
//                    +---------+             +-----------------------------+
//      data (fast) ---------------------------> RAM            ^     ^
//      data (slow) --------------------------------------------+-----+--> MMIO
//
//   * RAM has one port. A data access always wins it, in the same cycle and
//     without a handshake (a store from M first, then a load from X); a refill
//     takes the cycles in between.
//   * ROM is shared the same way (data access first).
//   * The flash controller serves one request at a time; the data side has
//     priority when both are waiting.
//   * MMIO is reached by the data side only, with a registered read.
// =============================================================================
`default_nettype none

module soc #(
    parameter int          CLK_HZ      = 12_000_000,
    parameter int          BAUD        = 115_200,
    parameter logic [31:0] RESET_PC    = 32'h0010_0000,
    parameter logic [31:0] MTVEC_RESET = 32'h0010_0008,
    parameter bit          BPRED       = 1'b1,
    parameter int          IC_LINES_LOG2 = 8,
    parameter int          FLASH_WAKE_CYCLES = 4096,
    parameter              ROM_FILE    = "firmware/bootrom/bootrom.hex"
) (
    input  wire        clk,
    input  wire        rst,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        flash_csn,
    output wire        flash_sck,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire [6:0]  gpio_out,
    input  wire [3:0]  gpio_in
);

    // ---- Core ------------------------------------------------------------------
    wire        rf_req, rf_gnt, rf_rvalid;
    wire [24:2] rf_addr;
    wire [31:0] rf_rdata;
    wire [31:0] d_addr, d_wdata;
    wire [16:2] dm_addr;
    wire        dm_wbank;
    wire [3:0]  d_be;
    wire        dm_en, dm_we;
    wire [31:0] dm_rdata;
    wire        db_req, db_we, db_ack;
    wire [31:0] db_rdata;
    wire        irq_timer, irq_soft, irq_ext;
    wire [63:0] cycle_count;

    riscv_core #(
        .RESET_PC      (RESET_PC),
        .MTVEC_RESET   (MTVEC_RESET),
        .BPRED         (BPRED),
        .IC_LINES_LOG2 (IC_LINES_LOG2)
    ) u_core (
        .clk         (clk),
        .rst         (rst),
        .rf_req      (rf_req),
        .rf_addr     (rf_addr),
        .rf_gnt      (rf_gnt),
        .rf_rvalid   (rf_rvalid),
        .rf_rdata    (rf_rdata),
        .d_wdata     (d_wdata),
        .d_be        (d_be),
        .dm_en       (dm_en),
        .dm_we       (dm_we),
        .dm_addr     (dm_addr),
        .dm_wbank    (dm_wbank),
        .dm_rdata    (dm_rdata),
        .db_addr     (d_addr),
        .db_req      (db_req),
        .db_we       (db_we),
        .db_ack      (db_ack),
        .db_rdata    (db_rdata),
        .irq_timer   (irq_timer),
        .irq_soft    (irq_soft),
        .irq_ext     (irq_ext),
        .cycle_count (cycle_count)
    );

    // Region of the line being refilled (constant for the whole refill) and of
    // the slow data access.
    wire rf_flash = rf_addr[24];
    wire rf_rom   = !rf_addr[24] && rf_addr[20];
    wire rf_ram   = !rf_addr[24] && !rf_addr[20];

    wire db_mmio  = d_addr[31];
    wire db_flash = !d_addr[31] && d_addr[24];
    wire db_rom   = !d_addr[31] && !d_addr[24];

    // ---- RAM -------------------------------------------------------------------
    wire [31:0] ram_rdata;
    wire        rf_ram_gnt = rf_req && rf_ram && !dm_en;
    logic       rf_ram_rvalid;

    ram128k u_ram (
        .clk   (clk),
        .en    (dm_en || rf_ram_gnt),
        .we    (dm_we),
        .addr  (dm_en ? dm_addr : rf_addr[16:2]),
        .wbank (dm_wbank),
        .wdata (d_wdata),
        .be    (d_be),
        .rdata (ram_rdata)
    );

    assign dm_rdata = ram_rdata;

    // ---- Boot ROM --------------------------------------------------------------
    localparam int ROM_WORDS_LOG2 = 9;

    wire [31:0] rom_rdata;
    logic       rom_dack, rf_rom_rvalid;
    wire        rom_dsel   = db_req && db_rom && !rom_dack;
    wire        rf_rom_gnt = rf_req && rf_rom && !rom_dsel;

    bootrom #(.WORDS_LOG2(ROM_WORDS_LOG2), .INIT_FILE(ROM_FILE)) u_rom (
        .clk   (clk),
        .addr  (rom_dsel ? d_addr[ROM_WORDS_LOG2+1:2] : rf_addr[ROM_WORDS_LOG2+1:2]),
        .rdata (rom_rdata)
    );

    // ---- SPI flash -------------------------------------------------------------
    wire        fl_gnt, fl_rvalid;
    wire [31:0] fl_rdata;
    logic       fl_d_wait;                 // data-side read in progress
    logic       fl_owner_d;
    wire        fl_dreq    = db_req && db_flash && !fl_d_wait;
    wire        fl_ireq    = rf_req && rf_flash;
    wire        rf_fl_gnt  = fl_gnt && !fl_dreq;
    wire        fl_dack    = fl_rvalid && fl_owner_d;

    spi_flash #(.WAKE_CYCLES(FLASH_WAKE_CYCLES)) u_flash (
        .clk        (clk),
        .rst        (rst),
        .req        (fl_dreq || fl_ireq),
        .addr       (fl_dreq ? d_addr[23:2] : rf_addr[23:2]),
        .gnt        (fl_gnt),
        .rvalid     (fl_rvalid),
        .rdata      (fl_rdata),
        .flash_csn  (flash_csn),
        .flash_sck  (flash_sck),
        .flash_mosi (flash_mosi),
        .flash_miso (flash_miso)
    );

    // ---- Memory-mapped I/O -------------------------------------------------------
    logic        mmio_ack;
    logic [31:0] mmio_rdata;
    wire         mmio_stb = db_req && db_mmio && !mmio_ack;
    wire [3:0]   mmio_dev = d_addr[15:12];
    wire [31:0]  uart_rdata, timer_rdata, gpio_rdata;

    uart #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart (
        .clk   (clk),
        .rst   (rst),
        .stb   (mmio_stb && mmio_dev == 4'd0),
        .we    (db_we),
        .addr  (d_addr[3:2]),
        .wdata (d_wdata),
        .rdata (uart_rdata),
        .irq   (irq_ext),
        .rxd   (uart_rx),
        .txd   (uart_tx)
    );

    timer u_timer (
        .clk       (clk),
        .rst       (rst),
        .stb       (mmio_stb && mmio_dev == 4'd1),
        .we        (db_we),
        .addr      (d_addr[4:2]),
        .wdata     (d_wdata),
        .rdata     (timer_rdata),
        .mtime     (cycle_count),
        .irq_timer (irq_timer),
        .irq_soft  (irq_soft)
    );

    gpio #(.CLK_HZ(CLK_HZ)) u_gpio (
        .clk      (clk),
        .rst      (rst),
        .stb      (mmio_stb && mmio_dev == 4'd2),
        .we       (db_we),
        .addr     (d_addr[3:2]),
        .wdata    (d_wdata),
        .rdata    (gpio_rdata),
        .gpio_out (gpio_out),
        .gpio_in  (gpio_in)
    );

    // ---- Handshakes ----------------------------------------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            rf_ram_rvalid <= 1'b0;
            rf_rom_rvalid <= 1'b0;
            rom_dack      <= 1'b0;
            mmio_ack      <= 1'b0;
            fl_d_wait     <= 1'b0;
            fl_owner_d    <= 1'b0;
        end else begin
            rf_ram_rvalid <= rf_ram_gnt;
            rf_rom_rvalid <= rf_rom_gnt;
            rom_dack      <= rom_dsel;
            mmio_ack      <= mmio_stb;
            if (fl_gnt) begin
                fl_owner_d <= fl_dreq;
                fl_d_wait  <= fl_dreq;
            end else if (fl_dack) begin
                fl_d_wait  <= 1'b0;
            end
        end
        if (mmio_stb) begin
            case (mmio_dev)
                4'd0:    mmio_rdata <= uart_rdata;
                4'd1:    mmio_rdata <= timer_rdata;
                4'd2:    mmio_rdata <= gpio_rdata;
                default: mmio_rdata <= 32'd0;
            endcase
        end
    end

    assign rf_gnt    = rf_ram_gnt || rf_rom_gnt || rf_fl_gnt;
    assign rf_rvalid = rf_ram_rvalid || rf_rom_rvalid || (fl_rvalid && !fl_owner_d);
    assign rf_rdata  = rf_flash ? fl_rdata : rf_rom ? rom_rdata : ram_rdata;

    assign db_ack   = mmio_ack || rom_dack || fl_dack;
    assign db_rdata = mmio_ack ? mmio_rdata : rom_dack ? rom_rdata : fl_rdata;

endmodule

`default_nettype wire
