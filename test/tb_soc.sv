// Simulation top level: the SoC with a behavioural SPI flash attached.
// The UART pins are driven by the testbench; programs can also be placed
// directly in `u_soc.u_ram.mem` and started through RESET_PC.
`default_nettype none

module tb_soc #(
    parameter int          CLK_HZ      = 80_000,       // short time-outs in simulation
    parameter int          BAUD        = 10_000,       // 8 clocks per bit
    parameter logic [31:0] RESET_PC    = 32'h0010_0000,
    parameter logic [31:0] MTVEC_RESET = 32'h0010_0008,
    parameter bit          BPRED       = 1'b1,
    parameter              ROM_FILE    = "firmware/bootrom/bootrom.hex"
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       uart_rx,
    output wire       uart_tx,
    output wire [5:0] gpio_out,
    input  wire [3:0] gpio_in
);
    wire flash_csn, flash_sck, flash_mosi, flash_miso;

    soc #(
        .CLK_HZ            (CLK_HZ),
        .BAUD              (BAUD),
        .RESET_PC          (RESET_PC),
        .MTVEC_RESET       (MTVEC_RESET),
        .BPRED             (BPRED),
        .FLASH_WAKE_CYCLES (16),
        .ROM_FILE          (ROM_FILE)
    ) u_soc (.*);

    spiflash_model u_flash (
        .csn  (flash_csn),
        .sck  (flash_sck),
        .mosi (flash_mosi),
        .miso (flash_miso)
    );
endmodule

// -----------------------------------------------------------------------------
// SPI flash model: release from power-down (ABh) and READ (03h), mode 0.
// Like the real part after iCE40 configuration, it starts in deep power-down
// and ignores everything except ABh until it has been woken.
// -----------------------------------------------------------------------------
module spiflash_model #(
    parameter int SIZE_LOG2 = 21          // 2 MB modelled; addresses wrap
) (
    input  wire csn,
    input  wire sck,
    input  wire mosi,
    output reg  miso
);
    reg [7:0]  mem [0:(1 << SIZE_LOG2) - 1];
    reg        awake = 1'b0;
    reg [7:0]  cmd;
    reg [23:0] addr;
    reg [7:0]  sr;
    integer    nbits;

    integer i;
    initial begin
        for (i = 0; i < (1 << SIZE_LOG2); i = i + 1) mem[i] = 8'hFF;
        miso = 1'b0;
    end

    always @(negedge csn) nbits = 0;

    always @(posedge csn) begin
        if (nbits >= 8 && cmd == 8'hAB) awake = 1'b1;
    end

    always @(posedge sck) if (!csn) begin
        if (nbits < 8)       cmd  = {cmd[6:0], mosi};
        else if (nbits < 32) addr = {addr[22:0], mosi};
        nbits = nbits + 1;
    end

    always @(negedge sck) if (!csn) begin
        if (awake && cmd == 8'h03 && nbits >= 32) begin
            if ((nbits - 32) % 8 == 0) begin
                sr   = mem[addr[SIZE_LOG2-1:0]];
                addr = addr + 24'd1;
            end
            miso <= sr[7];
            sr   = {sr[6:0], 1'b0};
        end
    end
endmodule

`default_nettype wire
