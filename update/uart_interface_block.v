module uart_interface_block (
    input  wire        clk_i,
    input  wire        rstn_i,

    input  wire        mcr_dtr_i,
    input  wire        mcr_rts_i,
    input  wire        mcr_out1_i,
    input  wire        mcr_out2_i,
    input  wire        mcr_lb_i,

    output wire        msr_cts_o,
    output wire        msr_dsr_o,
    output wire        msr_ri_o,
    output wire        msr_dcd_o,
    output wire        msr_dcts_o,
    output wire        msr_ddsr_o,
    output wire        msr_teri_o,
    output wire        msr_ddcd_o,

    input  wire        msr_read_i,

    output wire        uart_rts_n_o,
    output wire        uart_dtr_n_o,
    output wire        uart_out1_n_o,
    output wire        uart_out2_n_o,

    input  wire        uart_cts_n_i,
    input  wire        uart_dsr_n_i,
    input  wire        uart_ri_n_i,
    input  wire        uart_dcd_n_i,

    output wire        modem_int_o,

    input  wire        tx_serial_i,
    output wire        rx_serial_o
);

    reg cts_ff1, cts_ff2;
    reg dsr_ff1, dsr_ff2;
    reg ri_ff1,  ri_ff2;
    reg dcd_ff1, dcd_ff2;

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            {cts_ff1, cts_ff2} <= 2'b11;
            {dsr_ff1, dsr_ff2} <= 2'b11;
            {ri_ff1,  ri_ff2}  <= 2'b11;
            {dcd_ff1, dcd_ff2} <= 2'b11;
        end else begin
            cts_ff1 <= uart_cts_n_i; dsr_ff1 <= uart_dsr_n_i;
            ri_ff1  <= uart_ri_n_i;  dcd_ff1 <= uart_dcd_n_i;
            
            cts_ff2 <= cts_ff1; dsr_ff2 <= dsr_ff1;
            ri_ff2  <= ri_ff1;  dcd_ff2 <= dcd_ff1;
        end
    end

    wire cts_sync = ~cts_ff2;
    wire dsr_sync = ~dsr_ff2;
    wire ri_sync  = ~ri_ff2;
    wire dcd_sync = ~dcd_ff2;

    reg cts_muxed, dsr_muxed, ri_muxed, dcd_muxed;

    always @* begin
        if (mcr_lb_i) begin
            cts_muxed = mcr_rts_i;
            dsr_muxed = mcr_dtr_i;
            ri_muxed  = mcr_out1_i;
            dcd_muxed = mcr_out2_i;
        end else begin
            cts_muxed = cts_sync;
            dsr_muxed = dsr_sync;
            ri_muxed  = ri_sync;
            dcd_muxed = dcd_sync;
        end
    end

    assign rx_serial_o = mcr_lb_i ? tx_serial_i : 1'bz;

    reg cts_prev, dsr_prev, ri_prev, dcd_prev;
    reg dcts_r, ddsr_r, teri_r, ddcd_r;

    always @(posedge clk_i or negedge rstn_i) begin
        if (!rstn_i) begin
            cts_prev <= 1'b0; dsr_prev <= 1'b0; ri_prev  <= 1'b0; dcd_prev <= 1'b0;
            dcts_r   <= 1'b0; ddsr_r   <= 1'b0; teri_r   <= 1'b0; ddcd_r   <= 1'b0;
        end else begin
            cts_prev <= cts_muxed; dsr_prev <= dsr_muxed;
            ri_prev  <= ri_muxed;  dcd_prev <= dcd_muxed;

            if (msr_read_i) begin
                dcts_r <= 1'b0; ddsr_r <= 1'b0; teri_r <= 1'b0; ddcd_r <= 1'b0;
            end else begin
                if (cts_muxed ^ cts_prev) dcts_r <= 1'b1;
                if (dsr_muxed ^ dsr_prev) ddsr_r <= 1'b1;
                if (ri_prev & ~ri_muxed)  teri_r <= 1'b1;
                if (dcd_muxed ^ dcd_prev) ddcd_r <= 1'b1;
            end
        end
    end

    assign modem_int_o = dcts_r | ddsr_r | teri_r | ddcd_r;

    assign uart_rts_n_o  = mcr_lb_i ? 1'b1 : ~mcr_rts_i;
    assign uart_dtr_n_o  = mcr_lb_i ? 1'b1 : ~mcr_dtr_i;
    assign uart_out1_n_o = mcr_lb_i ? 1'b1 : ~mcr_out1_i;
    assign uart_out2_n_o = mcr_lb_i ? 1'b1 : ~mcr_out2_i;

    assign msr_cts_o  = cts_muxed; assign msr_dsr_o  = dsr_muxed;
    assign msr_ri_o   = ri_muxed;  assign msr_dcd_o  = dcd_muxed;
    assign msr_dcts_o = dcts_r;    assign msr_ddsr_o = ddsr_r;
    assign msr_teri_o = teri_r;    assign msr_ddcd_o = ddcd_r;

endmodule