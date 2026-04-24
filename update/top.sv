// ============================================================
//  apb_uart_full.sv  —  Top-level wrapper (UPDATED)
//  Kết nối đầy đủ tất cả sub-module, không còn tie-off
//
//  Sub-modules:
//    u_uart_core  : apb_uart_sv       (core + extended ports)
//    u_intf_block : uart_interface_block (modem signals)
//    u_irq_ctrl   : uart_interrupt_ctrl  (separated IRQs)
//    u_dma        : uart_dma_handshake   (DMA req/ack)
// ============================================================

module apb_uart_full
#(
    parameter int APB_ADDR_WIDTH = 12,
    parameter int TX_FIFO_DEPTH  = 16,
    parameter int RX_FIFO_DEPTH  = 16
)(
    // ── Clock & Reset ────────────────────────────────────────
    input  logic                        CLK,
    input  logic                        RSTN,

    // ── APB Slave Interface ──────────────────────────────────
    input  logic [APB_ADDR_WIDTH-1:0]   PADDR,
    input  logic [31:0]                 PWDATA,
    input  logic                        PWRITE,
    input  logic                        PSEL,
    input  logic                        PENABLE,
    output logic [31:0]                 PRDATA,
    output logic                        PREADY,
    output logic                        PSLVERR,

    // ── UART Serial I/O ──────────────────────────────────────
    input  logic                        uart_sin,
    output logic                        uart_sout,

    // ── Modem Control Outputs (active-low) ───────────────────
    output logic                        uart_rts_n,
    output logic                        uart_dtr_n,
    output logic                        uart_out1_n,
    output logic                        uart_out2_n,

    // ── Modem Status Inputs (active-low) ─────────────────────
    input  logic                        uart_cts_n,
    input  logic                        uart_dsr_n,
    input  logic                        uart_ri_n,
    input  logic                        uart_dcd_n,

    // ── Interrupt Outputs ────────────────────────────────────
    output logic                        uart_int,    // Master OR
    output logic                        tx_int,
    output logic                        rx_int,
    output logic                        rxovr_int,
    output logic                        txovr_int,

    // ── DMA TX Interface ─────────────────────────────────────
    output logic                        dma_tx_req,
    output logic                        dma_tx_single,
    input  logic                        dma_tx_ack,

    // ── DMA RX Interface ─────────────────────────────────────
    output logic                        dma_rx_req,
    output logic                        dma_rx_single,
    input  logic                        dma_rx_ack
);

    // ════════════════════════════════════════════════════════
    // INTERNAL WIRES
    // ════════════════════════════════════════════════════════

    // Core ↔ Interface block (MCR/MSR)
    logic        mcr_dtr, mcr_rts, mcr_out1, mcr_out2, mcr_lb;
    logic [7:0]  msr_from_intf;
    logic        msr_read_pulse;

    // Core → IRQ ctrl (IER + sources)
    logic [3:0]  ier_bits;
    logic        irq_rx_data_avail;
    logic        irq_rx_timeout;
    logic        irq_tx_empty;
    logic        irq_rx_overrun;
    logic        irq_rx_parity;
    logic        irq_rx_frame;
    logic        irq_rx_break;
    logic        irq_tx_overflow;

    // Core → DMA (FIFO status)
    logic        tx_fifo_not_full, tx_fifo_empty;
    logic        rx_fifo_not_empty, rx_fifo_full;
    logic [$clog2(TX_FIFO_DEPTH):0] tx_fifo_elements;
    logic [$clog2(RX_FIFO_DEPTH):0] rx_fifo_elements;

    // Interface block → Core (loopback RX path)
    logic        rx_serial_muxed;

    // IRQ ctrl internal
    logic [3:0]  iir_out;
    logic        modem_int_raw;

    // DMA config (có thể expose ra port hoặc tie như sau)
    logic [4:0]  tx_dma_threshold;
    logic [4:0]  rx_dma_threshold;
    logic        dma_tx_en, dma_rx_en;

    assign tx_dma_threshold = 5'd8;
    assign rx_dma_threshold = 5'd8;
    assign dma_tx_en        = 1'b1;
    assign dma_rx_en        = 1'b1;

    // ════════════════════════════════════════════════════════
    // INST 1: apb_uart_sv CORE — full featured
    // ════════════════════════════════════════════════════════
    apb_uart_sv #(
        .APB_ADDR_WIDTH (APB_ADDR_WIDTH),
        .TX_FIFO_DEPTH  (TX_FIFO_DEPTH),
        .RX_FIFO_DEPTH  (RX_FIFO_DEPTH)
    ) u_uart_core (
        .CLK             (CLK),
        .RSTN            (RSTN),

        // APB
        .PADDR           (PADDR),
        .PWDATA          (PWDATA),
        .PWRITE          (PWRITE),
        .PSEL            (PSEL),
        .PENABLE         (PENABLE),
        .PRDATA          (PRDATA),
        .PREADY          (PREADY),
        .PSLVERR         (PSLVERR),

        // Serial
        .rx_i            (rx_serial_muxed),  // Sau loopback mux
        .tx_o            (uart_sout),
        .event_o         (),                  // Không dùng — thay bằng uart_int

        // MCR → Interface block
        .mcr_dtr_o       (mcr_dtr),
        .mcr_rts_o       (mcr_rts),
        .mcr_out1_o      (mcr_out1),
        .mcr_out2_o      (mcr_out2),
        .mcr_lb_o        (mcr_lb),

        // MSR ← Interface block
        .msr_ext_i       (msr_from_intf),
        .msr_read_clr_i  (msr_read_pulse),

        // IER → IRQ ctrl
        .ier_o           (ier_bits),

        // TX FIFO status → DMA
        .tx_fifo_not_full_o  (tx_fifo_not_full),
        .tx_fifo_empty_o     (tx_fifo_empty),
        .tx_fifo_elements_o  (tx_fifo_elements),

        // RX FIFO status → DMA
        .rx_fifo_not_empty_o (rx_fifo_not_empty),
        .rx_fifo_full_o      (rx_fifo_full),
        .rx_fifo_elements_o  (rx_fifo_elements),

        // IRQ sources → IRQ ctrl
        .irq_rx_data_avail_o (irq_rx_data_avail),
        .irq_rx_timeout_o    (irq_rx_timeout),
        .irq_tx_empty_o      (irq_tx_empty),
        .irq_rx_overrun_o    (irq_rx_overrun),
        .irq_rx_parity_o     (irq_rx_parity),
        .irq_rx_frame_o      (irq_rx_frame),
        .irq_rx_break_o      (irq_rx_break),
        .irq_tx_overflow_o   (irq_tx_overflow),

        // MSR read pulse → Interface block
        .msr_read_o          (msr_read_pulse)
    );

    // ════════════════════════════════════════════════════════
    // INST 2: uart_interface_block — Modem signals + loopback
    // ════════════════════════════════════════════════════════
    uart_interface_block u_intf_block (
        .clk_i          (CLK),
        .rstn_i         (RSTN),

        // MCR từ core
        .mcr_dtr_i      (mcr_dtr),
        .mcr_rts_i      (mcr_rts),
        .mcr_out1_i     (mcr_out1),
        .mcr_out2_i     (mcr_out2),
        .mcr_lb_i       (mcr_lb),

        // MSR ra → core
        .msr_cts_o      (msr_from_intf[4]),
        .msr_dsr_o      (msr_from_intf[5]),
        .msr_ri_o       (msr_from_intf[6]),
        .msr_dcd_o      (msr_from_intf[7]),
        .msr_dcts_o     (msr_from_intf[0]),
        .msr_ddsr_o     (msr_from_intf[1]),
        .msr_teri_o     (msr_from_intf[2]),
        .msr_ddcd_o     (msr_from_intf[3]),
        .msr_read_i     (msr_read_pulse),

        // External modem pads
        .uart_rts_n_o   (uart_rts_n),
        .uart_dtr_n_o   (uart_dtr_n),
        .uart_out1_n_o  (uart_out1_n),
        .uart_out2_n_o  (uart_out2_n),
        .uart_cts_n_i   (uart_cts_n),
        .uart_dsr_n_i   (uart_dsr_n),
        .uart_ri_n_i    (uart_ri_n),
        .uart_dcd_n_i   (uart_dcd_n),

        // Modem IRQ source
        .modem_int_o    (modem_int_raw),

        // Loopback serial mux
        .tx_serial_i    (uart_sout),
        .rx_serial_o    ()           // Không dùng — loopback tích hợp trong core
    );

    // Loopback mux: MCR[4]=1 → tx loop back vào rx
    assign rx_serial_muxed = mcr_lb ? uart_sout : uart_sin;

    // ════════════════════════════════════════════════════════
    // INST 3: uart_interrupt_ctrl — Tách IRQ sources
    // ════════════════════════════════════════════════════════
    uart_interrupt_ctrl u_irq_ctrl (
        .clk_i           (CLK),
        .rstn_i          (RSTN),

        // IER mask từ core
        .ier_erbfi_i     (ier_bits[0]),
        .ier_etbei_i     (ier_bits[1]),
        .ier_elsi_i      (ier_bits[2]),
        .ier_emsi_i      (ier_bits[3]),

        // IRQ sources từ core
        .rx_data_avail_i (irq_rx_data_avail),
        .rx_timeout_i    (irq_rx_timeout),
        .tx_fifo_empty_i (irq_tx_empty),
        .rx_overrun_i    (irq_rx_overrun),
        .rx_parity_err_i (irq_rx_parity),
        .rx_frame_err_i  (irq_rx_frame),
        .rx_break_i      (irq_rx_break),
        .tx_overflow_i   (irq_tx_overflow),
        .modem_status_i  (modem_int_raw),

        // IIR (optional readback)
        .iir_o           (iir_out),

        // Separated outputs
        .uart_int_o      (uart_int),
        .tx_int_o        (tx_int),
        .rx_int_o        (rx_int),
        .rxovr_int_o     (rxovr_int),
        .txovr_int_o     (txovr_int),
        .modem_int_o     (),
        .line_int_o      ()
    );

    // ════════════════════════════════════════════════════════
    // INST 4: uart_dma_handshake
    // ════════════════════════════════════════════════════════
    uart_dma_handshake u_dma (
        .clk_i               (CLK),
        .rstn_i              (RSTN),

        .tx_fifo_not_full_i  (tx_fifo_not_full),
        .tx_fifo_empty_i     (tx_fifo_empty),
        .tx_fifo_elements_i  (tx_fifo_elements[$clog2(TX_FIFO_DEPTH):0]),

        .rx_fifo_not_empty_i (rx_fifo_not_empty),
        .rx_fifo_full_i      (rx_fifo_full),
        .rx_fifo_elements_i  (rx_fifo_elements[$clog2(RX_FIFO_DEPTH):0]),

        .dma_tx_req_o        (dma_tx_req),
        .dma_tx_single_o     (dma_tx_single),
        .dma_tx_ack_i        (dma_tx_ack),

        .dma_rx_req_o        (dma_rx_req),
        .dma_rx_single_o     (dma_rx_single),
        .dma_rx_ack_i        (dma_rx_ack),

        .tx_dma_threshold_i  (tx_dma_threshold),
        .rx_dma_threshold_i  (rx_dma_threshold),
        .dma_tx_enable_i     (dma_tx_en),
        .dma_rx_enable_i     (dma_rx_en),

        .dma_tx_active_o     (),
        .dma_rx_active_o     ()
    );

endmodule : apb_uart_full
