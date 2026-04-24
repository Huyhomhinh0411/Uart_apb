module top #(
    parameter APB_ADDR_WIDTH = 12,
    parameter TX_FIFO_DEPTH  = 16,
    parameter RX_FIFO_DEPTH  = 16
)(
    input  wire                        CLK,
    input  wire                        RSTN,
    input  wire [APB_ADDR_WIDTH-1:0]   PADDR,
    input  wire [31:0]                 PWDATA,
    input  wire                        PWRITE,
    input  wire                        PSEL,
    input  wire                        PENABLE,
    output wire [31:0]                 PRDATA,
    output wire                        PREADY,
    output wire                        PSLVERR,

    input  wire                        uart_sin,
    output wire                        uart_sout,

    output wire                        uart_rts_n,
    output wire                        uart_dtr_n,
    output wire                        uart_out1_n,
    output wire                        uart_out2_n,

    input  wire                        uart_cts_n,
    input  wire                        uart_dsr_n,
    input  wire                        uart_ri_n,
    input  wire                        uart_dcd_n,

    output wire                        uart_int,
    output wire                        tx_int,
    output wire                        rx_int,
    output wire                        rxovr_int,
    output wire                        txovr_int,

    output wire                        dma_tx_req,
    output wire                        dma_tx_single,
    input  wire                        dma_tx_ack,
    output wire                        dma_rx_req,
    output wire                        dma_rx_single,
    input  wire                        dma_rx_ack
);

    wire        mcr_dtr, mcr_rts, mcr_out1, mcr_out2, mcr_lb;
    wire [7:0]  msr_from_intf;
    wire        msr_read_pulse;
    wire [3:0]  ier_bits;

    wire        irq_rx_data_avail, irq_rx_timeout, irq_tx_empty;
    wire        irq_rx_overrun, irq_rx_parity, irq_rx_frame;
    wire        irq_rx_break, irq_tx_overflow;

    wire        tx_fifo_not_full, tx_fifo_empty;
    wire        rx_fifo_not_empty, rx_fifo_full;
    wire [4:0]  tx_fifo_elements, rx_fifo_elements;

    wire        rx_serial_muxed;
    wire [3:0]  iir_out;
    wire        modem_int_raw;

    wire [4:0]  tx_dma_threshold = 5'd8;
    wire [4:0]  rx_dma_threshold = 5'd8;
    wire        dma_tx_en = 1'b1;
    wire        dma_rx_en = 1'b1;

    apb_uart #(
        .APB_ADDR_WIDTH (APB_ADDR_WIDTH),
        .TX_FIFO_DEPTH  (TX_FIFO_DEPTH),
        .RX_FIFO_DEPTH  (RX_FIFO_DEPTH)
    ) u_uart_core (
        .CLK             (CLK),
        .RSTN            (RSTN),
        .PADDR           (PADDR),
        .PWDATA          (PWDATA),
        .PWRITE          (PWRITE),
        .PSEL            (PSEL),
        .PENABLE         (PENABLE),
        .PRDATA          (PRDATA),
        .PREADY          (PREADY),
        .PSLVERR         (PSLVERR),
        .rx_i            (rx_serial_muxed),
        .tx_o            (uart_sout),
        .event_o         (), 
        .mcr_dtr_o       (mcr_dtr),
        .mcr_rts_o       (mcr_rts),
        .mcr_out1_o      (mcr_out1),
        .mcr_out2_o      (mcr_out2),
        .mcr_lb_o        (mcr_lb),
        .msr_ext_i       (msr_from_intf),
        .msr_read_clr_i  (msr_read_pulse),
        .ier_o           (ier_bits),
        .tx_fifo_not_full_o  (tx_fifo_not_full),
        .tx_fifo_empty_o     (tx_fifo_empty),
        .tx_fifo_elements_o  (tx_fifo_elements),
        .rx_fifo_not_empty_o (rx_fifo_not_empty),
        .rx_fifo_full_o      (rx_fifo_full),
        .rx_fifo_elements_o  (rx_fifo_elements),
        .irq_rx_data_avail_o (irq_rx_data_avail),
        .irq_rx_timeout_o    (irq_rx_timeout),
        .irq_tx_empty_o      (irq_tx_empty),
        .irq_rx_overrun_o    (irq_rx_overrun),
        .irq_rx_parity_o     (irq_rx_parity),
        .irq_rx_frame_o      (irq_rx_frame),
        .irq_rx_break_o      (irq_rx_break),
        .irq_tx_overflow_o   (irq_tx_overflow),
        .msr_read_o          (msr_read_pulse)
    );

    uart_interface_block u_intf_block (
        .clk_i          (CLK),
        .rstn_i         (RSTN),
        .mcr_dtr_i      (mcr_dtr),
        .mcr_rts_i      (mcr_rts),
        .mcr_out1_i     (mcr_out1),
        .mcr_out2_i     (mcr_out2),
        .mcr_lb_i       (mcr_lb),
        .msr_cts_o      (msr_from_intf[4]),
        .msr_dsr_o      (msr_from_intf[5]),
        .msr_ri_o       (msr_from_intf[6]),
        .msr_dcd_o      (msr_from_intf[7]),
        .msr_dcts_o     (msr_from_intf[0]),
        .msr_ddsr_o     (msr_from_intf[1]),
        .msr_teri_o     (msr_from_intf[2]),
        .msr_ddcd_o     (msr_from_intf[3]),
        .msr_read_i     (msr_read_pulse),
        .uart_rts_n_o   (uart_rts_n),
        .uart_dtr_n_o   (uart_dtr_n),
        .uart_out1_n_o  (uart_out1_n),
        .uart_out2_n_o  (uart_out2_n),
        .uart_cts_n_i   (uart_cts_n),
        .uart_dsr_n_i   (uart_dsr_n),
        .uart_ri_n_i    (uart_ri_n),
        .uart_dcd_n_i   (uart_dcd_n),
        .modem_int_o    (modem_int_raw),
        .tx_serial_i    (uart_sout),
        .rx_serial_o    ()
    );

    assign rx_serial_muxed = mcr_lb ? uart_sout : uart_sin;

    uart_interrupt_ctrl u_irq_ctrl (
        .clk_i           (CLK),
        .rstn_i          (RSTN),
        .ier_erbfi_i     (ier_bits[0]),
        .ier_etbei_i     (ier_bits[1]),
        .ier_elsi_i      (ier_bits[2]),
        .ier_emsi_i      (ier_bits[3]),
        .rx_data_avail_i (irq_rx_data_avail),
        .rx_timeout_i    (irq_rx_timeout),
        .tx_fifo_empty_i (irq_tx_empty),
        .rx_overrun_i    (irq_rx_overrun),
        .rx_parity_err_i (irq_rx_parity),
        .rx_frame_err_i  (irq_rx_frame),
        .rx_break_i      (irq_rx_break),
        .tx_overflow_i   (irq_tx_overflow),
        .modem_status_i  (modem_int_raw),
        .iir_o           (iir_out),
        .uart_int_o      (uart_int),
        .tx_int_o        (tx_int),
        .rx_int_o        (rx_int),
        .rxovr_int_o     (rxovr_int),
        .txovr_int_o     (txovr_int),
        .modem_int_o     (),
        .line_int_o      ()
    );

    uart_dma_handshake u_dma (
        .clk_i               (CLK),
        .rstn_i              (RSTN),
        .tx_fifo_not_full_i  (tx_fifo_not_full),
        .tx_fifo_empty_i     (tx_fifo_empty),
        .tx_fifo_elements_i  (tx_fifo_elements),
        .rx_fifo_not_empty_i (rx_fifo_not_empty),
        .rx_fifo_full_i      (rx_fifo_full),
        .rx_fifo_elements_i  (rx_fifo_elements),
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

endmodule