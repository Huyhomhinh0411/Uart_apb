// ============================================================
//  uart_dma_handshake.sv
//  DMA Handshaking block cho apb_uart_sv
//
//  Chức năng:
//    - Tạo các tín hiệu request/acknowledge cho DMA controller
//    - TX path: DMA đẩy dữ liệu vào TX FIFO thay cho CPU
//    - RX path: DMA kéo dữ liệu từ RX FIFO về memory
//    - Hỗ trợ cả burst (dma_*_req) và single (dma_*_single)
//
//  Interface:
//    - Kết nối với APB UART thông qua FIFO status signals
//    - Kết nối với DMA controller thông qua req/single/ack
//
//  Synthesizable SystemVerilog, single-clock domain
// ============================================================

module uart_dma_handshake (
    // ── Clock & Reset ───────────────────────────────────────
    input  logic        clk_i,
    input  logic        rstn_i,

    // ── TX FIFO status (từ apb_uart_sv nội bộ) ─────────────
    input  logic        tx_fifo_not_full_i,   // TX FIFO chưa đầy → có thể nhận thêm
    input  logic        tx_fifo_empty_i,       // TX FIFO rỗng hoàn toàn
    input  logic [4:0]  tx_fifo_elements_i,    // Số byte hiện có trong TX FIFO

    // ── RX FIFO status (từ apb_uart_sv nội bộ) ─────────────
    input  logic        rx_fifo_not_empty_i,   // RX FIFO có dữ liệu → DMA có thể đọc
    input  logic        rx_fifo_full_i,         // RX FIFO đầy
    input  logic [4:0]  rx_fifo_elements_i,    // Số byte hiện có trong RX FIFO

    // ── DMA TX interface ────────────────────────────────────
    //   DMA controller nhìn vào đây để biết khi nào nên ghi vào UART TX
    output logic        dma_tx_req_o,     // Request burst: FIFO chưa đầy, DMA ghi nhiều byte
    output logic        dma_tx_single_o,  // Request single: FIFO sắp đầy, DMA chỉ ghi 1 byte
    input  logic        dma_tx_ack_i,     // DMA xác nhận đã ghi 1 beat

    // ── DMA RX interface ────────────────────────────────────
    //   DMA controller nhìn vào đây để biết khi nào có thể đọc từ UART RX
    output logic        dma_rx_req_o,     // Request burst: FIFO có nhiều byte
    output logic        dma_rx_single_o,  // Request single: FIFO có ≥1 byte
    input  logic        dma_rx_ack_i,     // DMA xác nhận đã đọc 1 beat

    // ── Configuration (từ thanh ghi FCR / custom reg) ───────
    input  logic [4:0]  tx_dma_threshold_i,  // Ngưỡng: TX FIFO elements < threshold → req
    input  logic [4:0]  rx_dma_threshold_i,  // Ngưỡng: RX FIFO elements ≥ threshold → req
    input  logic        dma_tx_enable_i,     // Enable DMA TX channel
    input  logic        dma_rx_enable_i,     // Enable DMA RX channel

    // ── Status output ────────────────────────────────────────
    output logic        dma_tx_active_o,   // DMA TX transaction đang diễn ra
    output logic        dma_rx_active_o    // DMA RX transaction đang diễn ra
);

    // ── Parameters ──────────────────────────────────────────
    localparam FIFO_DEPTH = 16;

    // ── Internal registers ──────────────────────────────────
    logic tx_req_r,    tx_single_r;
    logic rx_req_r,    rx_single_r;
    logic tx_active_r, rx_active_r;

    // ════════════════════════════════════════════════════════
    // TX DMA Logic
    // Điều kiện:
    //   dma_tx_req    = enable AND tx_elements < threshold (còn nhiều chỗ → burst)
    //   dma_tx_single = enable AND tx_fifo_not_full AND tx_elements >= threshold
    // ════════════════════════════════════════════════════════
    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            tx_req_r    <= 1'b0;
            tx_single_r <= 1'b0;
            tx_active_r <= 1'b0;
        end else begin
            if (dma_tx_enable_i) begin
                // Burst request: có đủ chỗ cho burst (elements < threshold)
                tx_req_r <= tx_fifo_not_full_i &&
                            (tx_fifo_elements_i < tx_dma_threshold_i);

                // Single request: gần đầy, chỉ đủ chỗ cho 1 byte
                tx_single_r <= tx_fifo_not_full_i &&
                               (tx_fifo_elements_i >= tx_dma_threshold_i);

                // Active: có request đang chờ ack
                if (tx_req_r || tx_single_r)
                    tx_active_r <= 1'b1;
                else if (dma_tx_ack_i)
                    tx_active_r <= 1'b0;
            end else begin
                tx_req_r    <= 1'b0;
                tx_single_r <= 1'b0;
                tx_active_r <= 1'b0;
            end
        end
    end

    // ════════════════════════════════════════════════════════
    // RX DMA Logic
    // Điều kiện:
    //   dma_rx_req    = enable AND rx_elements ≥ threshold (có nhiều byte → burst)
    //   dma_rx_single = enable AND rx_fifo_not_empty (có ≥1 byte → single)
    // ════════════════════════════════════════════════════════
    always_ff @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rx_req_r    <= 1'b0;
            rx_single_r <= 1'b0;
            rx_active_r <= 1'b0;
        end else begin
            if (dma_rx_enable_i) begin
                // Burst request: có đủ data để DMA đọc nhiều
                rx_req_r <= rx_fifo_not_empty_i &&
                            (rx_fifo_elements_i >= rx_dma_threshold_i);

                // Single request: chỉ có ít byte, không đủ threshold nhưng vẫn có data
                rx_single_r <= rx_fifo_not_empty_i &&
                               (rx_fifo_elements_i < rx_dma_threshold_i);

                // Active flag
                if (rx_req_r || rx_single_r)
                    rx_active_r <= 1'b1;
                else if (dma_rx_ack_i)
                    rx_active_r <= 1'b0;
            end else begin
                rx_req_r    <= 1'b0;
                rx_single_r <= 1'b0;
                rx_active_r <= 1'b0;
            end
        end
    end

    // ── Output assignments ───────────────────────────────────
    assign dma_tx_req_o    = tx_req_r;
    assign dma_tx_single_o = tx_single_r;
    assign dma_rx_req_o    = rx_req_r;
    assign dma_rx_single_o = rx_single_r;
    assign dma_tx_active_o = tx_active_r;
    assign dma_rx_active_o = rx_active_r;

endmodule : uart_dma_handshake
