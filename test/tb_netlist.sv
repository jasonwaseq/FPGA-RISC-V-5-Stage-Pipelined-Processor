// Gate-level smoke test: the synthesised SoC netlist (iCE40 primitives,
// including the SPRAM, block RAM and DSP models) boots its ROM and prints the
// banner. Catches differences between RTL simulation and what Yosys builds.
// Built and run by synth/netlist_sim.sh.
`timescale 1ns / 1ps

module tb_netlist;
    localparam int BIT = 8;             // clocks per UART bit (CLK_HZ / BAUD)

    reg clk = 0, rst = 1;
    wire uart_tx, flash_csn, flash_sck, flash_mosi, flash_miso;
    wire [5:0] gpio_out;

    always #5 clk = ~clk;

    soc u_soc (
        .clk(clk), .rst(rst), .uart_tx(uart_tx), .uart_rx(1'b1),
        .flash_csn(flash_csn), .flash_sck(flash_sck), .flash_mosi(flash_mosi),
        .flash_miso(flash_miso), .gpio_out(gpio_out), .gpio_in(4'b0)
    );

    spiflash_model u_flash (.csn(flash_csn), .sck(flash_sck), .mosi(flash_mosi), .miso(flash_miso));

    reg [8*64-1:0] text = 0;
    integer        count = 0, i;
    reg [7:0]      rxb;

    initial begin
        repeat (20) @(posedge clk);
        rst = 0;
    end

    // UART receiver
    always begin
        @(negedge uart_tx);
        repeat (BIT + BIT / 2) @(posedge clk);
        for (i = 0; i < 8; i = i + 1) begin
            rxb[i] = uart_tx;
            repeat (BIT) @(posedge clk);
        end
        $write("%c", rxb);
        text  = {text[8*63-1:0], rxb};
        count = count + 1;
    end

    initial begin
        repeat (60000) @(posedge clk);
        $display("\n[%0d bytes received]", count);
        if (count >= 20) $display("NETLIST BOOT OK");
        else             $display("NETLIST BOOT FAILED");
        $finish;
    end
endmodule
