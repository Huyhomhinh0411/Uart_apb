module uart_dma_handshake (
    input  wire        clk_i,
    input  wire        rstn_i,

    input  wire        tx_fifo_not_full_i,
    input  wire        tx_fifo_empty_i,
    input  wire [4:0]  tx_fifo_elements_i,

    input  wire        rx_fifo_not_empty_i,
    input  wire        rx_fifo_full_i,
    input  wire [4:0]  rx_fifo_elements_i,

    output wire        dma_tx_req_o,
    output wire        dma_tx_single_o,
    input  wire        dma_tx_ack_i,

    output wire        dma_rx_req_o,
    output wire        dma_rx_single_o,
    input  wire        dma_rx_ack_i,

    input  wire [4:0]  tx_dma_threshold_i,
    input  wire [4:0]  rx_dma_threshold_i,
    input  wire        dma_tx_enable_i,
    input  wire        dma_rx_enable_i,

    output wire        dma_tx_active_o,
    output wire        dma_rx_active_o
);

    reg tx_req_r,    tx_single_r;
    reg rx_req_r,    rx_single_r;
    reg tx_active_r, rx_active_r;

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            tx_req_r    <= 1'b0;
            tx_single_r <= 1'b0;
            tx_active_r <= 1'b0;
        end else begin
            if (dma_tx_enable_i) begin
                tx_req_r <= tx_fifo_not_full_i && (tx_fifo_elements_i < tx_dma_threshold_i);
                tx_single_r <= tx_fifo_not_full_i && (tx_fifo_elements_i >= tx_dma_threshold_i);
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

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            rx_req_r    <= 1'b0;
            rx_single_r <= 1'b0;
            rx_active_r <= 1'b0;
        end else begin
            if (dma_rx_enable_i) begin
                rx_req_r <= rx_fifo_not_empty_i && (rx_fifo_elements_i >= rx_dma_threshold_i);
                rx_single_r <= rx_fifo_not_empty_i && (rx_fifo_elements_i < rx_dma_threshold_i);
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

    assign dma_tx_req_o    = tx_req_r;
    assign dma_tx_single_o = tx_single_r;
    assign dma_rx_req_o    = rx_req_r;
    assign dma_rx_single_o = rx_single_r;
    assign dma_tx_active_o = tx_active_r;
    assign dma_rx_active_o = rx_active_r;

endmodule