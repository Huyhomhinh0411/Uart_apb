// ============================================================
//  uart_interrupt_ctrl.sv
//  Interrupt Control Block — đầy đủ như hình PL011
//
//  Tách biệt các nguồn ngắt:
//    uart_int   = OR của tất cả (nối vào NVIC/PLIC)
//    Nội bộ:
//      tx_int   = TX FIFO đạt ngưỡng / TX empty
//      rx_int   = RX FIFO đạt ngưỡng / RX timeout
//      rxovr_int= RX Overrun Error
//      txovr_int= TX Overflow (write khi full)
//      modem_int= Modem status thay đổi
//      line_int = Line status error (PE/FE/BI)
//
//  Hỗ trợ IIR priority encoder tương thích TI 16550
// ============================================================

module uart_interrupt_ctrl (
    // ── Clock & Reset ───────────────────────────────────────
    input  logic        clk_i,
    input  logic        rstn_i,

    // ── IER register (Interrupt Enable, từ register file) ───
    input  logic        ier_erbfi_i,    // IER[0]: Enable RX Data Available
    input  logic        ier_etbei_i,    // IER[1]: Enable TX Holding Reg Empty
    input  logic        ier_elsi_i,     // IER[2]: Enable Line Status Error
    input  logic        ier_emsi_i,     // IER[3]: Enable Modem Status

    // ── Raw interrupt sources ────────────────────────────────
    input  logic        rx_data_avail_i,   // RX FIFO ≥ trigger level
    input  logic        rx_timeout_i,      // RX FIFO timeout (4 char time)
    input  logic        tx_fifo_empty_i,   // TX FIFO rỗng
    input  logic        rx_overrun_i,      // RX Overrun Error (OE)
    input  logic        rx_parity_err_i,   // RX Parity Error  (PE)
    input  logic        rx_frame_err_i,    // RX Framing Error (FE)
    input  logic        rx_break_i,        // RX Break Interrupt (BI)
    input  logic        tx_overflow_i,     // TX write-when-full
    input  logic        modem_status_i,    // Modem status change

    // ── IIR register output (Interrupt Identification) ──────
    output logic [3:0]  iir_o,          // IIR[3:0] = {IID[2:0], IPend}
    // IPend=0 means interrupt pending (active-low, TI 16550 legacy)

    // ── Separated interrupt outputs (tương đương hình PL011) ─
    output logic        uart_int_o,     // Master OR — nối vào interrupt controller
    output logic        tx_int_o,       // TX interrupt only (ETBEI)
    output logic        rx_int_o,       // RX interrupt only (ERBFI + timeout)
    output logic        rxovr_int_o,    // RX Overrun interrupt
    output logic        txovr_int_o,    // TX Overflow interrupt
    output logic        modem_int_o,    // Modem status interrupt
    output logic        line_int_o      // Line status interrupt (PE/FE/BI)
);

    // ════════════════════════════════════════════════════════
    // MASKED INTERRUPT SIGNALS
    // ════════════════════════════════════════════════════════
    logic tx_int_masked;
    logic rx_int_masked;
    logic line_int_masked;
    logic modem_int_masked;
    logic rxovr_int_masked;

    assign tx_int_masked    = ier_etbei_i & tx_fifo_empty_i;
    assign rx_int_masked    = ier_erbfi_i & (rx_data_avail_i | rx_timeout_i);
    assign line_int_masked  = ier_elsi_i  & (rx_overrun_i | rx_parity_err_i |
                                              rx_frame_err_i | rx_break_i);
    assign modem_int_masked = ier_emsi_i  & modem_status_i;
    // Overrun/overflow luôn được report (không qua IER, giống PL011)
    assign rxovr_int_masked = ier_elsi_i  & rx_overrun_i;
    assign txovr_int_masked = tx_overflow_i;  // No mask — critical error

    // ════════════════════════════════════════════════════════
    // IIR PRIORITY ENCODER — TI 16550 standard
    // Priority (high → low):
    //   011 = Receiver Line Status  (OE/PE/FE/BI)
    //   010 = Received Data Ready   (FIFO ≥ trigger)
    //   110 = RX Character Timeout  (FIFO not empty, no new data)
    //   001 = TX Holding Reg Empty  (TX FIFO empty)
    //   000 = Modem Status          (CTS/DSR/RI/DCD change)
    //   IPend[0] = 1 means NO interrupt (active-low legacy)
    // ════════════════════════════════════════════════════════
    always_comb begin
        if (line_int_masked)
            iir_o = 4'b0110;   // {IID=011, IPend=0}
        else if (rx_int_masked && !rx_timeout_i)
            iir_o = 4'b0100;   // {IID=010, IPend=0}
        else if (rx_int_masked && rx_timeout_i)
            iir_o = 4'b1100;   // {IID=110, IPend=0}
        else if (tx_int_masked)
            iir_o = 4'b0010;   // {IID=001, IPend=0}
        else if (modem_int_masked)
            iir_o = 4'b0000;   // {IID=000, IPend=0}
        else
            iir_o = 4'b0001;   // No interrupt pending (IPend=1)
    end

    // ════════════════════════════════════════════════════════
    // SEPARATED OUTPUTS (như hình PL011 / ARM PrimeCell)
    // ════════════════════════════════════════════════════════
    assign tx_int_o    = tx_int_masked;
    assign rx_int_o    = rx_int_masked;
    assign line_int_o  = line_int_masked;
    assign modem_int_o = modem_int_masked;
    assign rxovr_int_o = rxovr_int_masked;
    assign txovr_int_o = txovr_int_masked;

    // Master OR: bất kỳ interrupt nào đang active
    assign uart_int_o  = tx_int_masked | rx_int_masked |
                         line_int_masked | modem_int_masked |
                         rxovr_int_masked | txovr_int_masked;

endmodule : uart_interrupt_ctrl
