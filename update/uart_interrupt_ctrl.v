module uart_interrupt_ctrl (
    input  wire        clk_i,
    input  wire        rstn_i,

    input  wire        ier_erbfi_i,
    input  wire        ier_etbei_i,
    input  wire        ier_elsi_i,
    input  wire        ier_emsi_i,

    input  wire        rx_data_avail_i,
    input  wire        rx_timeout_i,
    input  wire        tx_fifo_empty_i,
    input  wire        rx_overrun_i,
    input  wire        rx_parity_err_i,
    input  wire        rx_frame_err_i,
    input  wire        rx_break_i,
    input  wire        tx_overflow_i,
    input  wire        modem_status_i,

    output reg  [3:0]  iir_o,

    output wire        uart_int_o,
    output wire        tx_int_o,
    output wire        rx_int_o,
    output wire        rxovr_int_o,
    output wire        txovr_int_o,
    output wire        modem_int_o,
    output wire        line_int_o
);

    wire tx_int_masked;
    wire rx_int_masked;
    wire line_int_masked;
    wire modem_int_masked;
    wire rxovr_int_masked;
    wire txovr_int_masked;

    assign tx_int_masked    = ier_etbei_i & tx_fifo_empty_i;
    assign rx_int_masked    = ier_erbfi_i & (rx_data_avail_i | rx_timeout_i);
    assign line_int_masked  = ier_elsi_i  & (rx_overrun_i | rx_parity_err_i | rx_frame_err_i | rx_break_i);
    assign modem_int_masked = ier_emsi_i  & modem_status_i;
    assign rxovr_int_masked = ier_elsi_i  & rx_overrun_i;
    assign txovr_int_masked = tx_overflow_i;

    always @* begin
        if (line_int_masked)
            iir_o = 4'b0110;
        else if (rx_int_masked && !rx_timeout_i)
            iir_o = 4'b0100;
        else if (rx_int_masked && rx_timeout_i)
            iir_o = 4'b1100;
        else if (tx_int_masked)
            iir_o = 4'b0010;
        else if (modem_int_masked)
            iir_o = 4'b0000;
        else
            iir_o = 4'b0001;
    end

    assign tx_int_o    = tx_int_masked;
    assign rx_int_o    = rx_int_masked;
    assign line_int_o  = line_int_masked;
    assign modem_int_o = modem_int_masked;
    assign rxovr_int_o = rxovr_int_masked;
    assign txovr_int_o = txovr_int_masked;

    assign uart_int_o  = tx_int_masked | rx_int_masked | line_int_masked | 
                         modem_int_masked | rxovr_int_masked | txovr_int_masked;

endmodule