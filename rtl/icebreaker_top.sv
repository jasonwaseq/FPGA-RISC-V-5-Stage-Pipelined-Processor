// =============================================================================
// iCEBreaker v1.1a top level (iCE40UP5K-SG48)
//
// Clock: the 12 MHz oscillator through the PLL, F = 12 MHz * (PLL_DIVF + 1) / 32.
// Reset: PLL lock, the user button, or a break on the UART receive line. The
//        break lets the host take control of the board from any state without
//        touching it; it is detected outside the reset domain.
// =============================================================================
`default_nettype none

module icebreaker_top #(
    parameter int PLL_DIVF = 47,                          // 47: 18 MHz, 63: 24 MHz
    parameter int CLK_HZ   = 375_000 * (PLL_DIVF + 1),
    parameter int BAUD     = 115_200,
    parameter bit BPRED    = 1'b1,
    parameter     ROM_FILE = "firmware/bootrom/bootrom.hex"
) (
    input  wire CLK,            // 12 MHz
    input  wire BTN_N,          // user button, active low
    output wire LEDR_N,
    output wire LEDG_N,
    output wire LED1, LED2, LED3, LED4, LED5,
    input  wire BTN1, BTN2, BTN3,
    input  wire RX,
    output wire TX,
    output wire FLASH_SCK,
    output wire FLASH_SSB,
    output wire FLASH_IO0,      // MOSI
    input  wire FLASH_IO1       // MISO
);

    // ---- Clock -------------------------------------------------------------------
    wire clk, pll_locked;

    SB_PLL40_PAD #(
        .FEEDBACK_PATH ("SIMPLE"),
        .DIVR          (4'd0),
        .DIVF          (7'(PLL_DIVF)),
        .DIVQ          (3'd5),
        .FILTER_RANGE  (3'd1)
    ) u_pll (
        .PACKAGEPIN    (CLK),
        .PLLOUTGLOBAL  (clk),
        .LOCK          (pll_locked),
        .RESETB        (1'b1),
        .BYPASS        (1'b0)
    );

    // ---- Reset -------------------------------------------------------------------
    // A break is the receive line held low for far longer than any character
    // (about 22 ms here).
    localparam int BRKW = $clog2(CLK_HZ / 45);

    logic [1:0]      rx_s, btn_s;
    logic [BRKW-1:0] brk_cnt = '0;
    logic [11:0]     rst_cnt = '0;
    logic            run     = 1'b0;        // flip-flops power up as zero
    wire             rst     = !run;
    wire             brk     = brk_cnt[BRKW-1];

    always_ff @(posedge clk) begin
        rx_s  <= {rx_s[0], RX};
        btn_s <= {btn_s[0], BTN_N};

        if (rx_s[1])   brk_cnt <= '0;
        else if (!brk) brk_cnt <= brk_cnt + 1'b1;

        if (!pll_locked || !btn_s[1] || brk) begin
            rst_cnt <= '0;
            run     <= 1'b0;
        end else if (!rst_cnt[11]) begin
            rst_cnt <= rst_cnt + 1'b1;
        end else begin
            run <= 1'b1;
        end
    end

    // ---- System --------------------------------------------------------------------
    wire [6:0] gpio_out;

    soc #(
        .CLK_HZ   (CLK_HZ),
        .BAUD     (BAUD),
        .BPRED    (BPRED),
        .ROM_FILE (ROM_FILE)
    ) u_soc (
        .clk        (clk),
        .rst        (rst),
        .uart_tx    (TX),
        .uart_rx    (RX),
        .flash_csn  (FLASH_SSB),
        .flash_sck  (FLASH_SCK),
        .flash_mosi (FLASH_IO0),
        .flash_miso (FLASH_IO1),
        .gpio_out   (gpio_out),
        .gpio_in    ({!btn_s[1], BTN3, BTN2, BTN1})
    );

    assign LEDR_N = !gpio_out[0];
    assign LEDG_N = !gpio_out[1];
    assign {LED5, LED4, LED3, LED2, LED1} = gpio_out[6:2];

endmodule

`default_nettype wire
