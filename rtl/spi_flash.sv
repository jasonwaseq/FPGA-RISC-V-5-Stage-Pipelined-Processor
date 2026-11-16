// =============================================================================
// Memory-mapped SPI flash reader (execute in place)
//
// Presents the configuration flash as read-only memory, one 32-bit word per
// request. SPI mode 0, SCK = clk / 2, standard READ (03h).
//
//   * After reset the flash is released from deep power-down (ABh): the iCE40
//     puts it to sleep once the bitstream has been loaded.
//   * A request for the word that follows the previous one continues the open
//     read instead of sending a new command, so a sequential burst (an I-cache
//     line, a memcpy from flash) costs 64 clocks per word after the first.
//     Between words the clock is simply paused with chip select held low.
//   * A request for the word that was read last is answered from the shift
//     register, so byte-by-byte reads cost one transfer per word.
//
// `gnt` accepts a request; `rvalid` returns its data. One request at a time.
// =============================================================================
`default_nettype none

module spi_flash #(
    parameter int WAKE_CYCLES = 4096           // tRES1 after the ABh command
) (
    input  wire         clk,
    input  wire         rst,
    input  wire         req,
    input  wire  [23:2] addr,
    output wire         gnt,
    output logic        rvalid,
    output wire  [31:0] rdata,
    output logic        flash_csn,
    output logic        flash_sck,
    output wire         flash_mosi,
    input  wire         flash_miso
);

    typedef enum logic [2:0] {S_WAKE, S_WAIT, S_IDLE, S_CMD, S_DATA, S_PAUSE} state_t;
    state_t state;

    logic [31:0] out_sr;        // command / address shifted out MSB first
    logic [31:0] in_sr;
    logic [5:0]  bits;          // bits left in the current phase
    logic [23:2] cur_addr;      // word held in in_sr while a read is open
    localparam int WCW = $clog2(WAKE_CYCLES + 1);
    logic [WCW-1:0] wait_cnt;
    logic        shifting;

    assign flash_mosi = out_sr[31];

    // The flash sends the lowest-addressed byte first, most significant bit
    // first; the bus is little endian.
    assign rdata = {in_sr[7:0], in_sr[15:8], in_sr[23:16], in_sr[31:24]};

    wire sequential = (addr == cur_addr + 1'b1);
    wire same       = (addr == cur_addr);
    assign gnt = req && ((state == S_IDLE) || (state == S_PAUSE && (sequential || same)));

    // One bit takes two clocks: SCK rises (flash and controller sample), then
    // falls (both sides shift).
    wire bit_done = shifting && flash_sck;
    wire last_bit = bit_done && (bits == 6'd1);

    always_ff @(posedge clk) begin
        rvalid <= 1'b0;

        if (rst) begin
            state     <= S_WAKE;
            flash_csn <= 1'b1;
            flash_sck <= 1'b0;
            shifting  <= 1'b0;
            out_sr    <= 32'hAB00_0000;
            bits      <= 6'd8;
            wait_cnt  <= '0;
        end else begin
            if (shifting) begin
                flash_sck <= !flash_sck;
                if (!flash_sck) begin
                    in_sr <= {in_sr[30:0], flash_miso};
                end else begin
                    out_sr <= {out_sr[30:0], 1'b0};
                    bits   <= bits - 6'd1;
                end
            end

            case (state)
                S_WAKE: begin
                    if (!shifting && flash_csn) begin
                        flash_csn <= 1'b0;
                        shifting  <= 1'b1;
                    end else if (last_bit) begin
                        shifting  <= 1'b0;
                        flash_csn <= 1'b1;
                        state     <= S_WAIT;
                    end
                end
                S_WAIT: begin
                    wait_cnt <= wait_cnt + 1'b1;
                    if (wait_cnt == WCW'(WAKE_CYCLES)) state <= S_IDLE;
                end
                S_IDLE: begin
                    if (req) begin
                        flash_csn <= 1'b0;
                        shifting  <= 1'b1;
                        out_sr    <= {8'h03, addr, 2'b00};
                        bits      <= 6'd32;
                        cur_addr  <= addr;
                        state     <= S_CMD;
                    end
                end
                S_CMD: begin
                    if (last_bit) begin
                        bits  <= 6'd32;
                        state <= S_DATA;
                    end
                end
                S_DATA: begin
                    if (last_bit) begin
                        shifting  <= 1'b0;
                        rvalid    <= 1'b1;
                        state     <= S_PAUSE;
                    end
                end
                S_PAUSE: begin
                    if (req) begin
                        if (same) begin
                            rvalid   <= 1'b1;
                        end else if (sequential) begin
                            shifting <= 1'b1;
                            bits     <= 6'd32;
                            cur_addr <= addr;
                            state    <= S_DATA;
                        end else begin
                            flash_csn <= 1'b1;      // end the read, start over
                            state     <= S_IDLE;
                        end
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
