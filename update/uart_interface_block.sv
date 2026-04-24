// ============================================================
//  uart_interface_block.sv
//  UART Interface Block — Modem Control Signals
//
//  Chức năng:
//    - Quản lý 8 tín hiệu modem chuẩn TI 16550:
//        Outputs: RTS_N, DTR_N, OUT1_N, OUT2_N   (active-low)
//        Inputs : CTS_N, DSR_N, RI_N, DCD_N      (active-low)
//    - Đồng bộ hóa các input từ ngoài vào (2-FF synchroniser)
//    - Tạo delta flags (DCTS, DDSR, TERI, DDCD) cho MSR
//    - Hỗ trợ loopback mode (MCR[4]) để self-test
//    - Tạo UART interrupt khi modem status thay đổi (EMSI)
//
//  Register interface:
//    MCR [4:0] = {LB, OUT2, OUT1, RTS, DTR}
//    MSR [7:0] = {DCD, RI, DSR, CTS, DDCD, TERI, DDSR, DCTS}
// ============================================================

module uart_interface_block (
    // ── Clock & Reset ───────────────────────────────────────
    input  logic        clk_i,
    input  logic        rstn_i,

    // ── MCR register bits (từ register file) ───────────────
    input  logic        mcr_dtr_i,    // MCR[0]: Data Terminal Ready
    input  logic        mcr_rts_i,    // MCR[1]: Request To Send
    input  logic        mcr_out1_i,   // MCR[2]: Output 1 (user-defined)
    input  logic        mcr_out2_i,   // MCR[3]: Output 2 (INT enable gate)
    input  logic        mcr_lb_i,     // MCR[4]: Loopback mode enable

    // ── MSR register bits (ra register file) ───────────────
    output logic        msr_cts_o,    // MSR[4]: CTS current state
    output logic        msr_dsr_o,    // MSR[5]: DSR current state
    output logic        msr_ri_o,     // MSR[6]: RI  current state
    output logic        msr_dcd_o,    // MSR[7]: DCD current state
    output logic        msr_dcts_o,   // MSR[0]: Delta CTS  (clear-on-read)
    output logic        msr_ddsr_o,   // MSR[1]: Delta DSR  (clear-on-read)
    output logic        msr_teri_o,   // MSR[2]: Trailing Edge RI
    output logic        msr_ddcd_o,   // MSR[3]: Delta DCD  (clear-on-read)

    // ── Clear delta flags (khi APB đọc MSR) ────────────────
    input  logic        msr_read_i,   // Pulse: CPU đọc MSR → xóa delta bits

    // ── Modem OUTPUT pins (active-low, ra pad) ──────────────
    output logic        uart_rts_n_o,   // Request To Send
    output logic        uart_dtr_n_o,   // Data Terminal Ready
    output logic        uart_out1_n_o,  // Output 1
    output logic        uart_out2_n_o,  // Output 2

    // ── Modem INPUT pins (active-low, từ pad) ───────────────
    input  logic        uart_cts_n_i,   // Clear To Send
    input  logic        uart_dsr_n_i,   // Data Set Ready
    input  logic        uart_ri_n_i,    // Ring Indicator
    input  logic        uart_dcd_n_i,   // Data Carrier Detect

    // ── Interrupt output (nối vào interrupt control) ────────
    output logic        modem_int_o,    // Modem status change interrupt

    // ── Loopback serial path ─────────────────────────────────
    //    Khi LB=1, tx_data được loop lại vào rx_data nội bộ
    input  logic        tx_serial_i,    // TX serial từ TX block
    output logic        rx_serial_o     // RX serial vào RX block
);

    // ════════════════════════════════════════════════════════
    // 2-FF SYNCHRONISER cho các modem input (tất cả async)
    // ════════════════════════════════════════════════════════
    logic cts_ff1, cts_ff2;
    logic dsr_ff1, dsr_ff2;
    logic ri_ff1,  ri_ff2;
    logic dcd_ff1, dcd_ff2;

    // Không có timing constraint trên đường vào FF1 (set_false_path trong SDC)
    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            {cts_ff1, cts_ff2} <= 2'b11;  // active-low: idle = high
            {dsr_ff1, dsr_ff2} <= 2'b11;
            {ri_ff1,  ri_ff2}  <= 2'b11;
            {dcd_ff1, dcd_ff2} <= 2'b11;
        end else begin
            // Giai đoạn 1: capture từ async domain
            cts_ff1 <= uart_cts_n_i;
            dsr_ff1 <= uart_dsr_n_i;
            ri_ff1  <= uart_ri_n_i;
            dcd_ff1 <= uart_dcd_n_i;
            // Giai đoạn 2: stable output
            cts_ff2 <= cts_ff1;
            dsr_ff2 <= dsr_ff1;
            ri_ff2  <= ri_ff1;
            dcd_ff2 <= dcd_ff1;
        end
    end

    // Chuyển đổi active-low thành active-high nội bộ
    // (MSR lưu dưới dạng active-high để dễ xử lý)
    logic cts_sync, dsr_sync, ri_sync, dcd_sync;
    assign cts_sync = ~cts_ff2;
    assign dsr_sync = ~dsr_ff2;
    assign ri_sync  = ~ri_ff2;
    assign dcd_sync = ~dcd_ff2;

    // ════════════════════════════════════════════════════════
    // LOOPBACK MUX
    // Khi MCR[4]=1: modem inputs được thay bằng giá trị từ MCR outputs
    //               serial input lấy từ TX serial output
    // ════════════════════════════════════════════════════════
    logic cts_muxed, dsr_muxed, ri_muxed, dcd_muxed;

    always_comb begin
        if (mcr_lb_i) begin
            // Loopback: modem outputs loop lại thành modem inputs
            cts_muxed = mcr_rts_i;    // RTS → CTS
            dsr_muxed = mcr_dtr_i;    // DTR → DSR
            ri_muxed  = mcr_out1_i;   // OUT1 → RI
            dcd_muxed = mcr_out2_i;   // OUT2 → DCD
        end else begin
            // Normal: dùng synchronized external pins
            cts_muxed = cts_sync;
            dsr_muxed = dsr_sync;
            ri_muxed  = ri_sync;
            dcd_muxed = dcd_sync;
        end
    end

    // Serial loopback
    assign rx_serial_o = mcr_lb_i ? tx_serial_i : 1'bz; // Z = driven externally

    // ════════════════════════════════════════════════════════
    // DELTA FLAGS — phát hiện thay đổi trạng thái modem
    // DCTS, DDSR, DDCD: set khi có bất kỳ thay đổi nào (rising hoặc falling)
    // TERI: chỉ set khi RI chuyển từ HIGH xuống LOW (trailing edge)
    // Tất cả bị xóa khi CPU đọc MSR (msr_read_i pulse)
    // ════════════════════════════════════════════════════════
    logic cts_prev, dsr_prev, ri_prev, dcd_prev;
    logic dcts_r, ddsr_r, teri_r, ddcd_r;

    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            cts_prev <= 1'b0;
            dsr_prev <= 1'b0;
            ri_prev  <= 1'b0;
            dcd_prev <= 1'b0;
            dcts_r   <= 1'b0;
            ddsr_r   <= 1'b0;
            teri_r   <= 1'b0;
            ddcd_r   <= 1'b0;
        end else begin
            // Pipeline stage: register current state for edge detection
            cts_prev <= cts_muxed;
            dsr_prev <= dsr_muxed;
            ri_prev  <= ri_muxed;
            dcd_prev <= dcd_muxed;

            // Set delta flags on any change, clear on MSR read
            if (msr_read_i) begin
                dcts_r <= 1'b0;
                ddsr_r <= 1'b0;
                teri_r <= 1'b0;
                ddcd_r <= 1'b0;
            end else begin
                // DCTS: any CTS transition
                if (cts_muxed ^ cts_prev) dcts_r <= 1'b1;
                // DDSR: any DSR transition
                if (dsr_muxed ^ dsr_prev) ddsr_r <= 1'b1;
                // TERI: only HIGH-to-LOW transition of RI (trailing edge)
                if (ri_prev & ~ri_muxed)  teri_r <= 1'b1;
                // DDCD: any DCD transition
                if (dcd_muxed ^ dcd_prev) ddcd_r <= 1'b1;
            end
        end
    end

    // ════════════════════════════════════════════════════════
    // MODEM INTERRUPT
    // Assert khi có bất kỳ delta flag nào set
    // (IER[3] EMSI là mask, được xử lý bên ngoài trong interrupt logic)
    // ════════════════════════════════════════════════════════
    assign modem_int_o = dcts_r | ddsr_r | teri_r | ddcd_r;

    // ════════════════════════════════════════════════════════
    // OUTPUT PINS — active-low
    // Khi loopback mode, external pins không được drive
    // (trong thực tế cần tristating nếu dùng bidirectional pad)
    // ════════════════════════════════════════════════════════
    assign uart_rts_n_o  = mcr_lb_i ? 1'b1 : ~mcr_rts_i;
    assign uart_dtr_n_o  = mcr_lb_i ? 1'b1 : ~mcr_dtr_i;
    assign uart_out1_n_o = mcr_lb_i ? 1'b1 : ~mcr_out1_i;
    assign uart_out2_n_o = mcr_lb_i ? 1'b1 : ~mcr_out2_i;

    // ════════════════════════════════════════════════════════
    // MSR OUTPUT — current state + delta flags
    // ════════════════════════════════════════════════════════
    assign msr_cts_o  = cts_muxed;
    assign msr_dsr_o  = dsr_muxed;
    assign msr_ri_o   = ri_muxed;
    assign msr_dcd_o  = dcd_muxed;
    assign msr_dcts_o = dcts_r;
    assign msr_ddsr_o = ddsr_r;
    assign msr_teri_o = teri_r;
    assign msr_ddcd_o = ddcd_r;

endmodule : uart_interface_block
