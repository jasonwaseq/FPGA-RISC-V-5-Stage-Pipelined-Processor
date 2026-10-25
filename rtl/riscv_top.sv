// RISC-V Top Level for iCEBreaker FPGA
// Integrates the processor core with UART for communication

module riscv_top (
    input  logic       clk,        // 12 MHz clock
    input  logic       rst_n,      // Active-low reset button
    output logic [4:0] led,        // Status LEDs
    output logic       uart_tx,    // UART transmit
    input  logic       uart_rx     // UART receive
);

    // =========================================================================
    // UART Configuration (115200 baud @ 12 MHz)
    // prescale = clk_freq / (baud * 8) = 12000000 / (115200 * 8) = 13
    // =========================================================================
    localparam UART_PRESCALE = 16'd13;

    // =========================================================================
    // Internal Signals
    // =========================================================================
    logic        rst;
    logic [31:0] imem_addr, imem_data;
    logic [31:0] dmem_addr, dmem_wdata, dmem_rdata;
    logic        dmem_we;
    logic [3:0]  dmem_be;
    
    // UART signals
    logic [7:0]  uart_rx_data, uart_tx_data;
    logic        uart_rx_valid, uart_rx_ready;
    logic        uart_tx_valid, uart_tx_ready;
    
    assign rst = ~rst_n;

    // =========================================================================
    // Control State Machine
    // Commands:
    //   0xAA <addr_hi> <addr_lo> <d3> <d2> <d1> <d0> : Write instruction
    //   0xBB <reg>                                   : Read register
    //   0xCC                                         : Run processor
    //   0xDD                                         : Halt processor
    //   0xEE                                         : Reset processor
    //   0xBC <idx>                                   : Read status/counter
    //        idx 0 = cycles, 1 = instructions retired, 2 = halted (EBREAK)
    // =========================================================================
    typedef enum logic [2:0] {
        ST_IDLE,
        ST_WRITE_ADDR_HI,
        ST_WRITE_ADDR_LO,
        ST_WRITE_DATA,
        ST_READ_REG,
        ST_WAIT_REG,
        ST_SEND_RESPONSE
    } state_t;
    
    state_t       state;
    logic [7:0]   cmd;
    logic [15:0]  write_addr;
    logic [31:0]  write_data;
    logic [2:0]   byte_cnt;
    logic         cpu_running;
    logic         cpu_rst;
    logic [4:0]   dbg_reg_addr;
    logic [31:0]  dbg_reg_data;
    logic [31:0]  response;
    logic [2:0]   resp_cnt;  // Changed to 3 bits for 0-4 counting
    logic         rd_perf;   // 1: ST_WAIT_REG returns a counter, not a register
    logic [1:0]   perf_idx;
    logic         cpu_halted;
    logic [31:0]  perf_cycles, perf_instret;
    logic [31:0]  perf_value;

    always_comb begin
        case (perf_idx)
            2'd0:    perf_value = perf_cycles;
            2'd1:    perf_value = perf_instret;
            2'd2:    perf_value = {31'd0, cpu_halted};
            default: perf_value = 32'd0;
        endcase
    end
    
    // Instruction memory write interface
    logic        imem_we;
    logic [31:0] imem_waddr, imem_wdata;
    logic [3:0]  imem_be;

    // =========================================================================
    // LED Status Display
    // =========================================================================
    assign led[0] = cpu_running;
    assign led[1] = uart_rx_valid;
    assign led[2] = uart_tx_valid;
    assign led[3] = (state != ST_IDLE);
    assign led[4] = cpu_halted;    // Program finished (EBREAK)

    // =========================================================================
    // Command Processing State Machine
    // =========================================================================
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state        <= ST_IDLE;
            cmd          <= 8'd0;
            write_addr   <= 16'd0;
            write_data   <= 32'd0;
            byte_cnt     <= 3'd0;
            cpu_running  <= 1'b0;
            cpu_rst      <= 1'b1;
            dbg_reg_addr <= 5'd0;
            uart_rx_ready<= 1'b1;
            uart_tx_valid<= 1'b0;
            uart_tx_data <= 8'd0;
            imem_we      <= 1'b0;
            resp_cnt     <= 3'd0;
            response     <= 32'd0;
            rd_perf      <= 1'b0;
            perf_idx     <= 2'd0;
        end else begin
            imem_we <= 1'b0;
            cpu_rst <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    uart_rx_ready <= 1'b1;
                    if (uart_rx_valid && uart_rx_ready) begin
                        cmd <= uart_rx_data;
                        case (uart_rx_data)
                            8'hAA: begin
                                state <= ST_WRITE_ADDR_HI;
                                uart_rx_ready <= 1'b0;
                            end
                            8'hBB: begin
                                state <= ST_READ_REG;
                                rd_perf <= 1'b0;
                                uart_rx_ready <= 1'b0;
                            end
                            8'hBC: begin
                                state <= ST_READ_REG;
                                rd_perf <= 1'b1;
                                uart_rx_ready <= 1'b0;
                            end
                            8'hCC: begin
                                cpu_running <= 1'b1;
                                // Send ACK
                                uart_tx_data <= 8'hCC;
                                uart_tx_valid <= 1'b1;
                            end
                            8'hDD: begin
                                cpu_running <= 1'b0;
                                // Send ACK
                                uart_tx_data <= 8'hDD;
                                uart_tx_valid <= 1'b1;
                            end
                            8'hEE: begin
                                cpu_running <= 1'b0;
                                cpu_rst <= 1'b1;
                                // Send ACK
                                uart_tx_data <= 8'hEE;
                                uart_tx_valid <= 1'b1;
                            end
                            default: ;
                        endcase
                    end
                    // Clear tx_valid once data is accepted
                    if (uart_tx_valid && uart_tx_ready)
                        uart_tx_valid <= 1'b0;
                end
                
                ST_WRITE_ADDR_HI: begin
                    uart_rx_ready <= 1'b1;
                    if (uart_rx_valid && uart_rx_ready) begin
                        write_addr[15:8] <= uart_rx_data;
                        state <= ST_WRITE_ADDR_LO;
                        uart_rx_ready <= 1'b0;
                    end
                end
                
                ST_WRITE_ADDR_LO: begin
                    uart_rx_ready <= 1'b1;
                    if (uart_rx_valid && uart_rx_ready) begin
                        write_addr[7:0] <= uart_rx_data;
                        byte_cnt <= 3'd0;
                        state <= ST_WRITE_DATA;
                        uart_rx_ready <= 1'b0;
                    end
                end
                
                ST_WRITE_DATA: begin
                    uart_rx_ready <= 1'b1;
                    if (uart_rx_valid && uart_rx_ready) begin
                        case (byte_cnt)
                            3'd0: write_data[31:24] <= uart_rx_data;
                            3'd1: write_data[23:16] <= uart_rx_data;
                            3'd2: write_data[15:8]  <= uart_rx_data;
                            3'd3: write_data[7:0]   <= uart_rx_data;
                            default: ;
                        endcase
                        
                        if (byte_cnt == 3'd3) begin
                            // Write to instruction memory
                            imem_we <= 1'b1;
                            // Send ACK
                            uart_tx_data <= 8'hAA;
                            uart_tx_valid <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            byte_cnt <= byte_cnt + 1'b1;
                            uart_rx_ready <= 1'b0;
                        end
                    end
                end
                
                ST_READ_REG: begin
                    uart_rx_ready <= 1'b1;
                    if (uart_rx_valid && uart_rx_ready) begin
                        dbg_reg_addr <= uart_rx_data[4:0];
                        perf_idx <= uart_rx_data[1:0];
                        state <= ST_WAIT_REG;
                        uart_rx_ready <= 1'b0;
                    end
                end
                
                ST_WAIT_REG: begin
                    // Wait one cycle for register file to output the data
                    response <= rd_perf ? perf_value : dbg_reg_data;
                    resp_cnt <= 3'd0;
                    state <= ST_SEND_RESPONSE;
                end
                
                ST_SEND_RESPONSE: begin
                    // Send 4 bytes: [31:24], [23:16], [15:8], [7:0]
                    if (!uart_tx_valid || uart_tx_ready) begin
                        case (resp_cnt)
                            3'd0: uart_tx_data <= response[31:24];
                            3'd1: uart_tx_data <= response[23:16];
                            3'd2: uart_tx_data <= response[15:8];
                            3'd3: uart_tx_data <= response[7:0];
                            default: ;
                        endcase
                        uart_tx_valid <= 1'b1;
                        
                        if (resp_cnt == 3'd3) begin
                            state <= ST_IDLE;
                        end else begin
                            resp_cnt <= resp_cnt + 1'b1;
                        end
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
    
    // Instruction memory write address/data
    assign imem_waddr = {16'd0, write_addr};
    assign imem_wdata = write_data;  // No byte swap needed - data already in correct order
    assign imem_be    = 4'b1111;

    // =========================================================================
    // RISC-V Core
    // rst_n: Pipeline reset - active when halted (cpu_running=0) or cpu_rst
    // rf_rst_n: Register file reset - only on hard reset (cpu_rst or external reset)
    // =========================================================================
    riscv_core cpu (
        .clk          (clk),
        .rst_n        (rst_n & ~cpu_rst & cpu_running),  // Pipeline stops when halted
        .rf_rst_n     (rst_n & ~cpu_rst),                 // Registers preserved on halt
        .imem_addr    (imem_addr),
        .imem_data    (imem_data),
        .dmem_addr    (dmem_addr),
        .dmem_wdata   (dmem_wdata),
        .dmem_rdata   (dmem_rdata),
        .dmem_we      (dmem_we),
        .dmem_be      (dmem_be),
        .dbg_reg_addr (dbg_reg_addr),
        .dbg_reg_data (dbg_reg_data),
        .halted       (cpu_halted),
        .perf_cycles  (perf_cycles),
        .perf_instret (perf_instret)
    );

    // =========================================================================
    // Instruction Memory (1KB = 256 words)
    // =========================================================================
    logic [31:0] imem_rdata_core, imem_rdata_dbg;
    logic        imem_sel;
    
    assign imem_sel = imem_we;  // Debug write takes priority
    
    memory #(
        .DEPTH(256)
    ) imem (
        .clk   (clk),
        .addr  (imem_sel ? imem_waddr : imem_addr),
        .wdata (imem_wdata),
        .rdata (imem_data),
        .we    (imem_we),
        .be    (imem_be)
    );

    // =========================================================================
    // Data Memory (1KB = 256 words)
    // =========================================================================
    memory #(
        .DEPTH(256)
    ) dmem (
        .clk   (clk),
        .addr  (dmem_addr),
        .wdata (dmem_wdata),
        .rdata (dmem_rdata),
        .we    (dmem_we & cpu_running),
        .be    (dmem_be)
    );

    // =========================================================================
    // UART
    // =========================================================================
    uart #(
        .DATA_WIDTH(8)
    ) uart_inst (
        .clk             (clk),
        .rst             (rst),
        .s_axis_tdata    (uart_tx_data),
        .s_axis_tvalid   (uart_tx_valid),
        .s_axis_tready   (uart_tx_ready),
        .m_axis_tdata    (uart_rx_data),
        .m_axis_tvalid   (uart_rx_valid),
        .m_axis_tready   (uart_rx_ready),
        .rxd             (uart_rx),
        .txd             (uart_tx),
        .tx_busy         (),
        .rx_busy         (),
        .rx_overrun_error(),
        .rx_frame_error  (),
        .prescale        (UART_PRESCALE)
    );

endmodule
