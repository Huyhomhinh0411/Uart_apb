// ============================================================
//  apb_uart_sv.sv  —  FULL FEATURED VERSION
//  APB UART  tương thích TI 16550  +  extended ports
//
//  Bản này tích hợp sẵn tất cả extended ports:
//    ✅  APB slave interface (đầy đủ chuẩn AMBA APB)
//    ✅  Register file: RBR,THR,DLL,DLM,IER,IIR,FCR,LCR,MCR,LSR,MSR,SCR
//    ✅  TX FIFO (depth configurable) + TX Serializer FSM
//    ✅  RX 2-FF Sync + RX Deserializer FSM + RX FIFO
//    ✅  Baud Rate Generator (internal, DLL/DLM)
//    ✅  MCR output ports  (cho uart_interface_block)
//    ✅  MSR input port    (từ uart_interface_block)
//    ✅  FIFO status ports (cho uart_dma_handshake)
//    ✅  Separated IRQ sources (cho uart_interrupt_ctrl)
//    ✅  IER output        (cho uart_interrupt_ctrl)
//    ✅  Loopback serial mux nội bộ
//
//  Không cần patch hay wrapper thêm bên ngoài.
//  Synthesizable SystemVerilog 2012, single-clock domain.
// ============================================================

module apb_uart_sv
#(
    parameter int APB_ADDR_WIDTH = 12,  // APB address width (slave 4KB)
    parameter int TX_FIFO_DEPTH  = 16,  // TX FIFO depth (bytes)
    parameter int RX_FIFO_DEPTH  = 16   // RX FIFO depth (bytes)
)(
    // ── Clock & Reset ────────────────────────────────────────
    input  logic                        CLK,
    input  logic                        RSTN,       // Active-low async reset

    // ── APB Slave Interface ───────────────────────────────────
    input  logic [APB_ADDR_WIDTH-1:0]   PADDR,
    input  logic [31:0]                 PWDATA,
    input  logic                        PWRITE,
    input  logic                        PSEL,
    input  logic                        PENABLE,
    output logic [31:0]                 PRDATA,
    output logic                        PREADY,
    output logic                        PSLVERR,

    // ── UART Serial I/O ───────────────────────────────────────
    input  logic                        rx_i,       // Serial RX input (async)
    output logic                        tx_o,       // Serial TX output

    // ── Legacy single event output (backward compat) ─────────
    output logic                        event_o,    // OR of all enabled IRQs

    // ══════════════════════════════════════════════════════════
    // EXTENDED PORTS  (mới so với apb_uart_sv gốc)
    // ══════════════════════════════════════════════════════════

    // ── MCR output (→ uart_interface_block) ───────────────────
    output logic                        mcr_dtr_o,   // MCR[0]
    output logic                        mcr_rts_o,   // MCR[1]
    output logic                        mcr_out1_o,  // MCR[2]
    output logic                        mcr_out2_o,  // MCR[3]
    output logic                        mcr_lb_o,    // MCR[4] loopback

    // ── MSR input (← uart_interface_block) ────────────────────
    input  logic [7:0]                  msr_ext_i,   // MSR[7:0] từ interface block
    input  logic                        msr_read_clr_i, // Pulse xoá delta flags

    // ── IER output (→ uart_interrupt_ctrl) ────────────────────
    output logic [3:0]                  ier_o,       // IER[3:0]

    // ── TX FIFO status (→ uart_dma_handshake) ─────────────────
    output logic                        tx_fifo_not_full_o,
    output logic                        tx_fifo_empty_o,
    output logic [$clog2(TX_FIFO_DEPTH):0] tx_fifo_elements_o,

    // ── RX FIFO status (→ uart_dma_handshake) ─────────────────
    output logic                        rx_fifo_not_empty_o,
    output logic                        rx_fifo_full_o,
    output logic [$clog2(RX_FIFO_DEPTH):0] rx_fifo_elements_o,

    // ── Separated IRQ sources (→ uart_interrupt_ctrl) ─────────
    output logic                        irq_rx_data_avail_o,  // RX FIFO ≥ trigger
    output logic                        irq_rx_timeout_o,     // RX FIFO timeout
    output logic                        irq_tx_empty_o,       // TX FIFO empty
    output logic                        irq_rx_overrun_o,     // OE
    output logic                        irq_rx_parity_o,      // PE
    output logic                        irq_rx_frame_o,       // FE
    output logic                        irq_rx_break_o,       // BI
    output logic                        irq_tx_overflow_o,    // Write-when-full

    // ── MSR read pulse (→ uart_interface_block) ────────────────
    output logic                        msr_read_o    // 1-cycle pulse khi đọc MSR
);

    // ════════════════════════════════════════════════════════
    // LOCAL PARAMETERS  —  Register addresses (PADDR[2:0])
    // ════════════════════════════════════════════════════════
    localparam logic [2:0]
        ADDR_RBR_THR_DLL = 3'h0,
        ADDR_IER_DLM     = 3'h1,
        ADDR_IIR_FCR     = 3'h2,
        ADDR_LCR         = 3'h3,
        ADDR_MCR         = 3'h4,
        ADDR_LSR         = 3'h5,
        ADDR_MSR         = 3'h6,
        ADDR_SCR         = 3'h7;

    // FIFO pointer width
    localparam TX_PTR_W = $clog2(TX_FIFO_DEPTH) + 1;
    localparam RX_PTR_W = $clog2(RX_FIFO_DEPTH) + 1;

    // ════════════════════════════════════════════════════════
    // REGISTER FILE
    // ════════════════════════════════════════════════════════
    logic [7:0]  RBR_q;     // Receive Buffer  (read-only, pop từ RX FIFO)
    logic [7:0]  THR_q;     // TX Holding      (write-only alias, push vào TX FIFO)
    logic [7:0]  DLL_q;     // Divisor Latch Low
    logic [7:0]  DLM_q;     // Divisor Latch High
    logic [7:0]  IER_q;     // Interrupt Enable
    logic [7:0]  IIR_q;     // Interrupt Identification (combinational)
    logic [7:0]  FCR_q;     // FIFO Control
    logic [7:0]  LCR_q;     // Line Control
    logic [7:0]  MCR_q;     // Modem Control
    logic [7:0]  LSR_q;     // Line Status
    logic [7:0]  SCR_q;     // Scratch

    // DLAB: LCR[7] — controls address alias for 0x0 and 0x1
    wire dlab = LCR_q[7];

    // APB strobe signals
    wire wr_en = PSEL & PENABLE & PWRITE;
    wire rd_en = PSEL & PENABLE & ~PWRITE;

    // Always ready — zero wait states
    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    // ════════════════════════════════════════════════════════
    // TX FIFO  (sync, fall-through style)
    // ════════════════════════════════════════════════════════
    logic [7:0]  tx_fifo_mem [0:TX_FIFO_DEPTH-1];
    logic [TX_PTR_W-1:0] tx_wr_ptr, tx_rd_ptr;
    logic        tx_push, tx_pop;
    logic        tx_full, tx_empty;
    logic [TX_PTR_W-1:0] tx_count;

    assign tx_count  = tx_wr_ptr - tx_rd_ptr;
    assign tx_full   = (tx_count == TX_FIFO_DEPTH[TX_PTR_W-1:0]);
    assign tx_empty  = (tx_count == '0);

    // TX FIFO write port (từ APB write THR)
    assign tx_push = wr_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & ~tx_full;
    // Overflow flag: write khi full
    assign irq_tx_overflow_o = wr_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & tx_full;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            tx_wr_ptr <= '0;
        end else if (tx_push) begin
            tx_fifo_mem[tx_wr_ptr[$clog2(TX_FIFO_DEPTH)-1:0]] <= PWDATA[7:0];
            tx_wr_ptr <= tx_wr_ptr + 1'b1;
        end
    end

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN)
            tx_rd_ptr <= '0;
        else if (tx_pop & ~tx_empty)
            tx_rd_ptr <= tx_rd_ptr + 1'b1;
    end

    // TX FIFO flush (FCR write, bit 2 = XFIFOR)
    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            tx_wr_ptr <= '0;
            tx_rd_ptr <= '0;
        end else if (wr_en & (PADDR[2:0] == ADDR_IIR_FCR) & PWDATA[2]) begin
            tx_wr_ptr <= '0;
            tx_rd_ptr <= '0;
        end
    end

    // TX FIFO status outputs
    assign tx_fifo_not_full_o  = ~tx_full;
    assign tx_fifo_empty_o     = tx_empty;
    assign tx_fifo_elements_o  = tx_count;

    // ════════════════════════════════════════════════════════
    // BAUD RATE GENERATOR
    // Divisor = {DLM_q, DLL_q}  (16-bit)
    // baud_tick fires every Divisor cycles
    // baud_tick_x16: 16 ticks per bit (oversampling)
    // ════════════════════════════════════════════════════════
    logic [15:0] baud_div;
    logic [15:0] baud_cnt;
    logic        baud_tick;      // 1 pulse per (Divisor) CLK cycles
    logic [3:0]  baud_x16_cnt;
    logic        baud_tick_x16;  // 16× baud_tick (for RX oversampling)

    assign baud_div = {DLM_q, DLL_q};

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            baud_cnt <= '0;
            baud_tick <= 1'b0;
        end else begin
            if (baud_cnt == (baud_div - 1'b1) || baud_div == '0) begin
                baud_cnt  <= '0;
                baud_tick <= 1'b1;
            end else begin
                baud_cnt  <= baud_cnt + 1'b1;
                baud_tick <= 1'b0;
            end
        end
    end

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            baud_x16_cnt  <= '0;
            baud_tick_x16 <= 1'b0;
        end else begin
            if (baud_tick) begin
                baud_x16_cnt  <= baud_x16_cnt + 1'b1;
                baud_tick_x16 <= 1'b1;
            end else begin
                baud_tick_x16 <= 1'b0;
            end
        end
    end

    // ════════════════════════════════════════════════════════
    // TX SERIALIZER FSM
    // States: IDLE → START → DATA[0..N] → [PARITY] → STOP → IDLE
    // ════════════════════════════════════════════════════════
    typedef enum logic [2:0] {
        TX_IDLE   = 3'd0,
        TX_START  = 3'd1,
        TX_DATA   = 3'd2,
        TX_PARITY = 3'd3,
        TX_STOP   = 3'd4
    } tx_state_e;

    tx_state_e   tx_state;
    logic [7:0]  tx_shift;      // Shift register (LSB first)
    logic [2:0]  tx_bit_cnt;    // Bit counter (0..7)
    logic        tx_parity_bit; // Running parity
    logic        tx_out;        // Raw TX output

    // Word length: LCR[1:0] → 5,6,7,8 bits
    wire [2:0] tx_word_len = {1'b0, LCR_q[1:0]} + 3'd5;

    assign tx_pop = (tx_state == TX_IDLE) & ~tx_empty & baud_tick;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            tx_state     <= TX_IDLE;
            tx_out       <= 1'b1;
            tx_bit_cnt   <= '0;
            tx_shift     <= '0;
            tx_parity_bit<= 1'b0;
        end else if (baud_tick) begin
            case (tx_state)
                TX_IDLE: begin
                    tx_out <= 1'b1;
                    if (!tx_empty) begin
                        // Load byte from FIFO head
                        tx_shift  <= tx_fifo_mem[tx_rd_ptr[$clog2(TX_FIFO_DEPTH)-1:0]];
                        tx_state  <= TX_START;
                        tx_bit_cnt<= '0;
                        tx_parity_bit <= LCR_q[4]; // EPS: even=0, odd=1 seed
                    end
                end

                TX_START: begin
                    tx_out   <= 1'b0;  // Start bit
                    tx_state <= TX_DATA;
                end

                TX_DATA: begin
                    tx_out        <= tx_shift[0];
                    tx_parity_bit <= tx_parity_bit ^ tx_shift[0];
                    tx_shift      <= {1'b0, tx_shift[7:1]};  // LSB first
                    if (tx_bit_cnt == tx_word_len - 1'b1) begin
                        tx_bit_cnt <= '0;
                        tx_state   <= LCR_q[3] ? TX_PARITY : TX_STOP;
                    end else begin
                        tx_bit_cnt <= tx_bit_cnt + 1'b1;
                    end
                end

                TX_PARITY: begin
                    // Stick parity (LCR[5]=1): force parity bit
                    // Even parity (LCR[4]=1): parity=0 if even count
                    tx_out   <= LCR_q[5] ? ~LCR_q[4] : tx_parity_bit;
                    tx_state <= TX_STOP;
                end

                TX_STOP: begin
                    tx_out <= 1'b1;  // Stop bit(s)
                    // LCR[2]: 0=1 stop, 1=2 stop (for 6,7,8-bit words)
                    if (LCR_q[2] & (tx_bit_cnt == '0)) begin
                        tx_bit_cnt <= tx_bit_cnt + 1'b1; // Wait extra stop
                    end else begin
                        tx_bit_cnt <= '0;
                        tx_state   <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // Break condition: LCR[6]=1 forces tx_o=0
    // Loopback: wenn MCR[4]=1, tx_o still outputs normally, loop handled externally
    assign tx_o = LCR_q[6] ? 1'b0 : tx_out;

    // LSR TX flags
    wire lsr_thre = tx_empty;  // TX FIFO empty
    wire lsr_temt = tx_empty & (tx_state == TX_IDLE);  // TX shift also idle

    // IRQ source
    assign irq_tx_empty_o = tx_empty;

    // ════════════════════════════════════════════════════════
    // RX 2-FF SYNCHRONISER
    // rx_i is asynchronous — must pass through 2-FF before any logic
    // SDC: set_false_path -from [get_ports rx_i]
    // ════════════════════════════════════════════════════════
    logic rx_ff1, rx_ff2, rx_sync;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_ff1 <= 1'b1;
            rx_ff2 <= 1'b1;
        end else begin
            rx_ff1 <= MCR_q[4] ? tx_out : rx_i;  // Loopback mux
            rx_ff2 <= rx_ff1;
        end
    end
    assign rx_sync = rx_ff2;

    // ════════════════════════════════════════════════════════
    // RX DESERIALIZER FSM  (oversampling ×16)
    // ════════════════════════════════════════════════════════
    typedef enum logic [2:0] {
        RX_IDLE   = 3'd0,
        RX_START  = 3'd1,
        RX_DATA   = 3'd2,
        RX_PARITY = 3'd3,
        RX_STOP   = 3'd4
    } rx_state_e;

    rx_state_e   rx_state;
    logic [7:0]  rx_shift;
    logic [3:0]  rx_sample_cnt;  // 0-15: count baud_tick_x16 pulses
    logic [2:0]  rx_bit_cnt;
    logic        rx_parity_bit;
    logic        rx_pe, rx_fe, rx_bi;  // Error flags for current byte
    logic        rx_done;              // Pulse: valid byte in rx_shift
    logic [7:0]  rx_byte_out;
    logic        rx_pe_out, rx_fe_out, rx_bi_out;

    wire [2:0] rx_word_len = {1'b0, LCR_q[1:0]} + 3'd5;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_state      <= RX_IDLE;
            rx_shift      <= '0;
            rx_sample_cnt <= '0;
            rx_bit_cnt    <= '0;
            rx_parity_bit <= 1'b0;
            rx_pe         <= 1'b0;
            rx_fe         <= 1'b0;
            rx_bi         <= 1'b0;
            rx_done       <= 1'b0;
            rx_byte_out   <= '0;
            rx_pe_out     <= 1'b0;
            rx_fe_out     <= 1'b0;
            rx_bi_out     <= 1'b0;
        end else begin
            rx_done <= 1'b0;  // Default: no new byte

            case (rx_state)
                // ── IDLE: watch for falling edge (start bit candidate)
                RX_IDLE: begin
                    rx_pe <= 1'b0;
                    rx_fe <= 1'b0;
                    rx_bi <= 1'b0;
                    if (~rx_sync) begin  // Falling edge detected
                        rx_state      <= RX_START;
                        rx_sample_cnt <= '0;
                        rx_bit_cnt    <= '0;
                        rx_parity_bit <= LCR_q[4]; // EPS seed
                    end
                end

                // ── START: wait 8 ticks then sample middle of start bit
                RX_START: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd7) begin
                            rx_sample_cnt <= '0;
                            // Validate start bit: must still be 0
                            if (~rx_sync)
                                rx_state <= RX_DATA;
                            else
                                rx_state <= RX_IDLE;  // Glitch, abort
                        end else begin
                            rx_sample_cnt <= rx_sample_cnt + 1'b1;
                        end
                    end
                end

                // ── DATA: sample at bit centre (every 16 ticks)
                RX_DATA: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= '0;
                            // Sample RX at bit centre (tick 15 = bit centre)
                            rx_shift      <= {rx_sync, rx_shift[7:1]};  // LSB first
                            rx_parity_bit <= rx_parity_bit ^ rx_sync;
                            if (rx_bit_cnt == rx_word_len - 1'b1) begin
                                rx_bit_cnt <= '0;
                                rx_state   <= LCR_q[3] ? RX_PARITY : RX_STOP;
                            end else begin
                                rx_bit_cnt <= rx_bit_cnt + 1'b1;
                            end
                        end else begin
                            rx_sample_cnt <= rx_sample_cnt + 1'b1;
                        end
                    end
                end

                // ── PARITY: check parity bit
                RX_PARITY: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= '0;
                            // Check parity
                            if (LCR_q[5]) begin
                                // Stick parity
                                rx_pe <= (rx_sync != ~LCR_q[4]);
                            end else begin
                                rx_pe <= (rx_sync != rx_parity_bit);
                            end
                            rx_state <= RX_STOP;
                        end else begin
                            rx_sample_cnt <= rx_sample_cnt + 1'b1;
                        end
                    end
                end

                // ── STOP: verify stop bit, push to FIFO
                RX_STOP: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= '0;
                            // Framing error: stop bit should be 1
                            rx_fe <= ~rx_sync;
                            // Break: entire frame was 0
                            rx_bi <= ~rx_sync & ~(|rx_shift);
                            // Output byte
                            rx_byte_out <= rx_shift;
                            rx_pe_out   <= rx_pe;
                            rx_fe_out   <= ~rx_sync;
                            rx_bi_out   <= ~rx_sync & ~(|rx_shift);
                            rx_done     <= 1'b1;
                            rx_state    <= RX_IDLE;
                        end else begin
                            rx_sample_cnt <= rx_sample_cnt + 1'b1;
                        end
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // ════════════════════════════════════════════════════════
    // RX FIFO
    // ════════════════════════════════════════════════════════
    // Each entry stores: {rx_bi, rx_fe, rx_pe, rx_byte} = 11 bits
    logic [10:0] rx_fifo_mem [0:RX_FIFO_DEPTH-1];
    logic [RX_PTR_W-1:0] rx_wr_ptr, rx_rd_ptr;
    logic        rx_push, rx_pop_en;
    logic        rx_full, rx_empty;
    logic [RX_PTR_W-1:0] rx_count;

    assign rx_count = rx_wr_ptr - rx_rd_ptr;
    assign rx_full  = (rx_count == RX_FIFO_DEPTH[RX_PTR_W-1:0]);
    assign rx_empty = (rx_count == '0);

    // Push: new byte from RX FSM, if not full
    assign rx_push = rx_done & ~rx_full;
    // Overrun: new byte arrives but FIFO is full
    assign irq_rx_overrun_o = rx_done & rx_full;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_wr_ptr <= '0;
        end else if (rx_push) begin
            rx_fifo_mem[rx_wr_ptr[$clog2(RX_FIFO_DEPTH)-1:0]] <= {rx_bi_out, rx_fe_out, rx_pe_out, rx_byte_out};
            rx_wr_ptr <= rx_wr_ptr + 1'b1;
        end
    end

    // Pop: APB read from RBR
    assign rx_pop_en = rd_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & ~rx_empty;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN)
            rx_rd_ptr <= '0;
        else if (rx_pop_en)
            rx_rd_ptr <= rx_rd_ptr + 1'b1;
    end

    // RX FIFO flush (FCR bit 1 = RFIFOR)
    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_wr_ptr <= '0;
            rx_rd_ptr <= '0;
        end else if (wr_en & (PADDR[2:0] == ADDR_IIR_FCR) & PWDATA[1]) begin
            rx_wr_ptr <= '0;
            rx_rd_ptr <= '0;
        end
    end

    // Current RBR head value
    wire [10:0] rx_head = rx_empty ? 11'h0 : rx_fifo_mem[rx_rd_ptr[$clog2(RX_FIFO_DEPTH)-1:0]];

    // RX FIFO status outputs
    assign rx_fifo_not_empty_o = ~rx_empty;
    assign rx_fifo_full_o      = rx_full;
    assign rx_fifo_elements_o  = rx_count;

    // ════════════════════════════════════════════════════════
    // RX TIMEOUT COUNTER
    // Assert irq_rx_timeout_o if RX FIFO has data but no new
    // byte arrives within 4 character times
    // ════════════════════════════════════════════════════════
    logic [7:0]  rx_timeout_cnt;
    logic        rx_timeout_flag;
    // Character time ≈ (word_len + 2) × 16 baud_tick_x16 pulses
    wire [7:0] char_time = (8'(rx_word_len) + 8'd2) << 4;

    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_timeout_cnt  <= '0;
            rx_timeout_flag <= 1'b0;
        end else begin
            if (rx_empty | rx_pop_en | rx_done) begin
                // Reset on empty FIFO, CPU read, or new byte
                rx_timeout_cnt  <= '0;
                rx_timeout_flag <= 1'b0;
            end else if (baud_tick_x16 & ~rx_empty) begin
                if (rx_timeout_cnt >= (char_time << 2)) begin
                    rx_timeout_flag <= 1'b1;
                end else begin
                    rx_timeout_cnt <= rx_timeout_cnt + 1'b1;
                end
            end
        end
    end

    assign irq_rx_timeout_o = rx_timeout_flag;

    // ════════════════════════════════════════════════════════
    // LSR — Line Status Register  (read-only hardware updates)
    // [0] DR    – Data Ready
    // [1] OE    – Overrun Error
    // [2] PE    – Parity Error
    // [3] FE    – Framing Error
    // [4] BI    – Break Interrupt
    // [5] THRE  – TX Holding Reg Empty
    // [6] TEMT  – Transmitter Empty
    // [7] FIFOER– Error in RX FIFO
    // ════════════════════════════════════════════════════════
    logic lsr_oe, lsr_pe, lsr_fe, lsr_bi;

    // Error bits cleared when LSR is read
    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            lsr_oe <= 1'b0;
            lsr_pe <= 1'b0;
            lsr_fe <= 1'b0;
            lsr_bi <= 1'b0;
        end else begin
            // Set on error events
            if (irq_rx_overrun_o) lsr_oe <= 1'b1;
            if (rx_push & rx_pe_out) lsr_pe <= 1'b1;
            if (rx_push & rx_fe_out) lsr_fe <= 1'b1;
            if (rx_push & rx_bi_out) lsr_bi <= 1'b1;
            // Clear on LSR read
            if (rd_en & (PADDR[2:0] == ADDR_LSR)) begin
                lsr_oe <= 1'b0;
                lsr_pe <= 1'b0;
                lsr_fe <= 1'b0;
                lsr_bi <= 1'b0;
            end
        end
    end

    // RX error flags for IRQ controller
    assign irq_rx_parity_o = lsr_pe;
    assign irq_rx_frame_o  = lsr_fe;
    assign irq_rx_break_o  = lsr_bi;

    // FIFO has an entry with error
    wire lsr_fifoer = rx_head[10] | rx_head[9] | rx_head[8]; // BI|FE|PE in head

    // Assemble LSR
    always_comb begin
        LSR_q = 8'h00;
        LSR_q[0] = ~rx_empty;       // DR
        LSR_q[1] = lsr_oe;          // OE
        LSR_q[2] = lsr_pe;          // PE
        LSR_q[3] = lsr_fe;          // FE
        LSR_q[4] = lsr_bi;          // BI
        LSR_q[5] = lsr_thre;        // THRE
        LSR_q[6] = lsr_temt;        // TEMT
        LSR_q[7] = lsr_fifoer;      // FIFOER
    end

    // IRQ: data available when FIFO ≥ trigger level
    wire [4:0] rx_trigger = (FCR_q[7:6] == 2'b00) ? 5'd1  :
                            (FCR_q[7:6] == 2'b01) ? 5'd4  :
                            (FCR_q[7:6] == 2'b10) ? 5'd8  : 5'd14;
    assign irq_rx_data_avail_o = (rx_count >= {2'b0, rx_trigger});

    // ════════════════════════════════════════════════════════
    // IIR — Interrupt Identification Register (combinational)
    // Priority: RLS(011) > RDA(010) > RTO(110) > THRE(001) > MS(000)
    // Bit [0] IPend: 0=interrupt pending, 1=no interrupt (active-low)
    // ════════════════════════════════════════════════════════
    logic line_int, rx_int_pend, rx_tout_pend, tx_int_pend, ms_int_pend;

    assign line_int    = IER_q[2] & (lsr_oe | lsr_pe | lsr_fe | lsr_bi);
    assign rx_int_pend = IER_q[0] & irq_rx_data_avail_o;
    assign rx_tout_pend= IER_q[0] & rx_timeout_flag;
    assign tx_int_pend = IER_q[1] & lsr_thre;
    assign ms_int_pend = IER_q[3] & msr_ext_i[3:0] != 4'h0; // any delta bit set

    always_comb begin
        if (line_int)
            IIR_q = 8'h06;  // RLS
        else if (rx_int_pend & ~rx_tout_pend)
            IIR_q = 8'h04;  // RDA
        else if (rx_tout_pend)
            IIR_q = 8'h0C;  // RTO
        else if (tx_int_pend)
            IIR_q = 8'h02;  // THRE
        else if (ms_int_pend)
            IIR_q = 8'h00;  // MS
        else
            IIR_q = 8'h01;  // No interrupt (IPend=1)
    end

    // ════════════════════════════════════════════════════════
    // REGISTER WRITE LOGIC
    // ════════════════════════════════════════════════════════
    always_ff @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            DLL_q <= 8'h00;
            DLM_q <= 8'h00;
            IER_q <= 8'h00;
            FCR_q <= 8'h00;
            LCR_q <= 8'h00;
            MCR_q <= 8'h00;
            SCR_q <= 8'h00;
        end else if (wr_en) begin
            case (PADDR[2:0])
                ADDR_RBR_THR_DLL: if (dlab)  DLL_q <= PWDATA[7:0];
                ADDR_IER_DLM:     if (dlab)  DLM_q <= PWDATA[7:0];
                                  else        IER_q <= PWDATA[7:0];
                ADDR_IIR_FCR:                FCR_q <= PWDATA[7:0];
                ADDR_LCR:                    LCR_q <= PWDATA[7:0];
                ADDR_MCR:                    MCR_q <= PWDATA[4:0]; // only [4:0]
                ADDR_SCR:                    SCR_q <= PWDATA[7:0];
                default: ;
            endcase
        end
    end

    // ════════════════════════════════════════════════════════
    // REGISTER READ LOGIC
    // ════════════════════════════════════════════════════════
    assign msr_read_o = rd_en & (PADDR[2:0] == ADDR_MSR);

    always_comb begin
        PRDATA = 32'h0;
        if (rd_en) begin
            case (PADDR[2:0])
                ADDR_RBR_THR_DLL:
                    PRDATA = dlab ? {24'h0, DLL_q} : {24'h0, rx_head[7:0]};
                ADDR_IER_DLM:
                    PRDATA = dlab ? {24'h0, DLM_q} : {24'h0, IER_q};
                ADDR_IIR_FCR:
                    PRDATA = {24'h0, IIR_q};
                ADDR_LCR:
                    PRDATA = {24'h0, LCR_q};
                ADDR_MCR:
                    PRDATA = {24'h0, 3'b0, MCR_q[4:0]};
                ADDR_LSR:
                    PRDATA = {24'h0, LSR_q};
                ADDR_MSR:
                    // MSR driven from uart_interface_block via msr_ext_i
                    PRDATA = {24'h0, msr_ext_i};
                ADDR_SCR:
                    PRDATA = {24'h0, SCR_q};
                default:
                    PRDATA = 32'h0;
            endcase
        end
    end

    // ════════════════════════════════════════════════════════
    // EXTENDED OUTPUT ASSIGNMENTS
    // ════════════════════════════════════════════════════════

    // MCR bits → uart_interface_block
    assign mcr_dtr_o  = MCR_q[0];
    assign mcr_rts_o  = MCR_q[1];
    assign mcr_out1_o = MCR_q[2];
    assign mcr_out2_o = MCR_q[3];
    assign mcr_lb_o   = MCR_q[4];

    // IER bits → uart_interrupt_ctrl
    assign ier_o = IER_q[3:0];

    // ════════════════════════════════════════════════════════
    // LEGACY EVENT OUTPUT  (backward compatibility)
    // event_o = OR of all enabled, pending interrupts
    // ════════════════════════════════════════════════════════
    assign event_o = ~IIR_q[0];  // IIR[0]=0 means interrupt pending

endmodule : apb_uart_sv
