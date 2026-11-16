// =============================================================================
// Main memory and boot ROM
// =============================================================================
`default_nettype none

// -----------------------------------------------------------------------------
// 128 KB main memory: the four 256 kbit single-port RAMs of the iCE40UP5K.
// Two 16-bit halves make a 32-bit word, two such pairs make two 64 KB banks.
// Synchronous: the word addressed in one cycle is on rdata in the next, and is
// only guaranteed there for that one cycle.
// -----------------------------------------------------------------------------
module ram128k (
    input  wire         clk,
    input  wire         en,
    input  wire         we,
    input  wire  [14:0] addr,       // word address
    input  wire         wbank,      // addr[14] for a write (known a stage early)
    input  wire  [31:0] wdata,
    input  wire  [3:0]  be,
    output wire  [31:0] rdata
);

`ifdef SYNTHESIS
    wire [31:0] q [0:1];
    logic       bank_q;

    always_ff @(posedge clk) if (en) bank_q <= addr[14];

    // Each MASKWREN bit covers one nibble
    wire [3:0] mask_lo = {be[1], be[1], be[0], be[0]};
    wire [3:0] mask_hi = {be[3], be[3], be[2], be[2]};

    genvar g;
    generate
        // Both banks are read on every access and the registered bank bit
        // picks the result, which keeps the bank decode off the address path;
        // only the write strobe is steered.
        for (g = 0; g < 2; g++) begin : bank
            wire cs  = en;
            wire wen = we && (wbank == g);
            SB_SPRAM256KA lo (
                .ADDRESS    (addr[13:0]),
                .DATAIN     (wdata[15:0]),
                .MASKWREN   (mask_lo),
                .WREN       (wen),
                .CHIPSELECT (cs),
                .CLOCK      (clk),
                .STANDBY    (1'b0),
                .SLEEP      (1'b0),
                .POWEROFF   (1'b1),
                .DATAOUT    (q[g][15:0])
            );
            SB_SPRAM256KA hi (
                .ADDRESS    (addr[13:0]),
                .DATAIN     (wdata[31:16]),
                .MASKWREN   (mask_hi),
                .WREN       (wen),
                .CHIPSELECT (cs),
                .CLOCK      (clk),
                .STANDBY    (1'b0),
                .SLEEP      (1'b0),
                .POWEROFF   (1'b1),
                .DATAOUT    (q[g][31:16])
            );
        end
    endgenerate

    assign rdata = q[bank_q];
`else
    // Simulation model. The output is unknown after a write or an idle cycle,
    // which catches any logic that relies on more than the hardware promises.
    logic [31:0] mem [0:32767];
    logic [31:0] q;

    always_ff @(posedge clk) begin
        if (en && we) begin
            if (be[0]) mem[addr][7:0]   <= wdata[7:0];
            if (be[1]) mem[addr][15:8]  <= wdata[15:8];
            if (be[2]) mem[addr][23:16] <= wdata[23:16];
            if (be[3]) mem[addr][31:24] <= wdata[31:24];
        end
        q <= (en && !we) ? mem[addr] : 32'hxxxx_xxxx;
    end

    assign rdata = q;

    initial begin
        for (int i = 0; i < 32768; i++) mem[i] = 32'd0;
    end
`endif

endmodule

// -----------------------------------------------------------------------------
// Boot ROM: block RAM initialised from the first-stage bootloader image.
// -----------------------------------------------------------------------------
module bootrom #(
    parameter int WORDS_LOG2 = 9,              // 512 words = 2 KB
    parameter     INIT_FILE  = "firmware/bootrom/bootrom.hex"
) (
    input  wire                   clk,
    input  wire  [WORDS_LOG2-1:0] addr,
    output logic [31:0]           rdata
);

    logic [31:0] mem [0:(1 << WORDS_LOG2) - 1];

    // Unconditional on purpose: Yosys drops the initialisation if it sits
    // behind a test on the file name.
    initial $readmemh(INIT_FILE, mem);

    always_ff @(posedge clk) rdata <= mem[addr];

endmodule

`default_nettype wire
