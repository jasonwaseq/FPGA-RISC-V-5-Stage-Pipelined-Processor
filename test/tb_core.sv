// Simulation wrapper: core + the same synchronous memories used on the FPGA.
// Cocotb loads programs by writing imem.mem[] directly.
module tb_core #(
    parameter bit FORWARDING = 1'b1
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        rf_rst_n,
    input  logic [4:0]  dbg_reg_addr,
    output logic [31:0] dbg_reg_data,
    output logic        halted,
    output logic [31:0] perf_cycles,
    output logic [31:0] perf_instret
);
    logic [31:0] imem_addr, imem_data;
    logic [31:0] dmem_addr, dmem_wdata, dmem_rdata;
    logic        dmem_we;
    logic [3:0]  dmem_be;

    riscv_core #(.FORWARDING(FORWARDING)) cpu (.*);

    memory #(.DEPTH(1024)) imem (
        .clk(clk), .addr(imem_addr), .wdata(32'd0), .rdata(imem_data),
        .we(1'b0), .be(4'b0000)
    );

    memory #(.DEPTH(1024)) dmem (
        .clk(clk), .addr(dmem_addr), .wdata(dmem_wdata), .rdata(dmem_rdata),
        .we(dmem_we), .be(dmem_be)
    );
endmodule
