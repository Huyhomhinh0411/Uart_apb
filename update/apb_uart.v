module apb_uart #(
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
    output reg  [31:0]                 PRDATA,
    output wire                        PREADY,
    output wire                        PSLVERR,

    input  wire                        rx_i,
    output wire                        tx_o,
    output wire                        event_o,

    output wire                        mcr_dtr_o,
    output wire                        mcr_rts_o,
    output wire                        mcr_out1_o,
    output wire                        mcr_out2_o,
    output wire                        mcr_lb_o,

    input  wire [7:0]                  msr_ext_i,
    input  wire                        msr_read_clr_i,
    output wire [3:0]                  ier_o,

    output wire                        tx_fifo_not_full_o,
    output wire                        tx_fifo_empty_o,
    output wire [4:0]                  tx_fifo_elements_o, 

    output wire                        rx_fifo_not_empty_o,
    output wire                        rx_fifo_full_o,
    output wire [4:0]                  rx_fifo_elements_o,

    output wire                        irq_rx_data_avail_o,
    output wire                        irq_rx_timeout_o,
    output wire                        irq_tx_empty_o,
    output wire                        irq_rx_overrun_o,
    output wire                        irq_rx_parity_o,
    output wire                        irq_rx_frame_o,
    output wire                        irq_rx_break_o,
    output wire                        irq_tx_overflow_o,
    output wire                        msr_read_o
);

    localparam [2:0]
        ADDR_RBR_THR_DLL = 3'h0,
        ADDR_IER_DLM     = 3'h1,
        ADDR_IIR_FCR     = 3'h2,
        ADDR_LCR         = 3'h3,
        ADDR_MCR         = 3'h4,
        ADDR_LSR         = 3'h5,
        ADDR_MSR         = 3'h6,
        ADDR_SCR         = 3'h7;

    reg [7:0] DLL_q, DLM_q, IER_q, FCR_q, LCR_q, MCR_q, SCR_q;
    reg [7:0] IIR_q, LSR_q;

    wire dlab = LCR_q[7];
    wire wr_en = PSEL & PENABLE & PWRITE;
    wire rd_en = PSEL & PENABLE & ~PWRITE;

    assign PREADY  = 1'b1;
    assign PSLVERR = 1'b0;

    // TX FIFO
    reg [7:0] tx_fifo_mem [0:TX_FIFO_DEPTH-1];
    reg [4:0] tx_wr_ptr, tx_rd_ptr;
    wire      tx_push, tx_pop, tx_full, tx_empty;
    wire [4:0] tx_count = tx_wr_ptr - tx_rd_ptr;

    assign tx_full   = (tx_count == TX_FIFO_DEPTH[4:0]);
    assign tx_empty  = (tx_count == 5'd0);
    assign tx_push   = wr_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & ~tx_full;
    assign irq_tx_overflow_o = wr_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & tx_full;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            tx_wr_ptr <= 0;
            tx_rd_ptr <= 0;
        end else begin
            if (wr_en & (PADDR[2:0] == ADDR_IIR_FCR) & PWDATA[2]) begin // flush
                tx_wr_ptr <= 0;
                tx_rd_ptr <= 0;
            end else begin
                if (tx_push) begin
                    tx_fifo_mem[tx_wr_ptr[3:0]] <= PWDATA[7:0];
                    tx_wr_ptr <= tx_wr_ptr + 1'b1;
                end
                if (tx_pop & ~tx_empty)
                    tx_rd_ptr <= tx_rd_ptr + 1'b1;
            end
        end
    end

    assign tx_fifo_not_full_o  = ~tx_full;
    assign tx_fifo_empty_o     = tx_empty;
    assign tx_fifo_elements_o  = tx_count;

    // BRG
    wire [15:0] baud_div = {DLM_q, DLL_q};
    reg  [15:0] baud_cnt;
    reg         baud_tick;
    reg  [3:0]  baud_x16_cnt;
    reg         baud_tick_x16;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            baud_cnt <= 0; baud_tick <= 1'b0;
        end else begin
            if (baud_cnt == (baud_div - 1'b1) || baud_div == 0) begin
                baud_cnt  <= 0; baud_tick <= 1'b1;
            end else begin
                baud_cnt  <= baud_cnt + 1'b1; baud_tick <= 1'b0;
            end
        end
    end

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            baud_x16_cnt <= 0; baud_tick_x16 <= 1'b0;
        end else begin
            if (baud_tick) begin
                baud_x16_cnt <= baud_x16_cnt + 1'b1; baud_tick_x16 <= 1'b1;
            end else begin
                baud_tick_x16 <= 1'b0;
            end
        end
    end

    // TX FSM
    localparam TX_IDLE = 3'd0, TX_START = 3'd1, TX_DATA = 3'd2, TX_PARITY = 3'd3, TX_STOP = 3'd4;
    reg [2:0] tx_state;
    reg [7:0] tx_shift;
    reg [2:0] tx_bit_cnt;
    reg       tx_parity_bit;
    reg       tx_out;

    wire [2:0] tx_word_len = {1'b0, LCR_q[1:0]} + 3'd5;
    assign tx_pop = (tx_state == TX_IDLE) & ~tx_empty & baud_tick;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            tx_state <= TX_IDLE; tx_out <= 1'b1; tx_bit_cnt <= 0; tx_shift <= 0; tx_parity_bit <= 1'b0;
        end else if (baud_tick) begin
            case (tx_state)
                TX_IDLE: begin
                    tx_out <= 1'b1;
                    if (!tx_empty) begin
                        tx_shift <= tx_fifo_mem[tx_rd_ptr[3:0]];
                        tx_state <= TX_START;
                        tx_bit_cnt <= 0;
                        tx_parity_bit <= LCR_q[4];
                    end
                end
                TX_START: begin
                    tx_out <= 1'b0; tx_state <= TX_DATA;
                end
                TX_DATA: begin
                    tx_out <= tx_shift[0];
                    tx_parity_bit <= tx_parity_bit ^ tx_shift[0];
                    tx_shift <= {1'b0, tx_shift[7:1]};
                    if (tx_bit_cnt == tx_word_len - 1'b1) begin
                        tx_bit_cnt <= 0;
                        tx_state <= LCR_q[3] ? TX_PARITY : TX_STOP;
                    end else begin
                        tx_bit_cnt <= tx_bit_cnt + 1'b1;
                    end
                end
                TX_PARITY: begin
                    tx_out <= LCR_q[5] ? ~LCR_q[4] : tx_parity_bit;
                    tx_state <= TX_STOP;
                end
                TX_STOP: begin
                    tx_out <= 1'b1;
                    if (LCR_q[2] & (tx_bit_cnt == 0)) begin
                        tx_bit_cnt <= tx_bit_cnt + 1'b1;
                    end else begin
                        tx_bit_cnt <= 0;
                        tx_state <= TX_IDLE;
                    end
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign tx_o = LCR_q[6] ? 1'b0 : tx_out;
    wire lsr_thre = tx_empty;
    wire lsr_temt = tx_empty & (tx_state == TX_IDLE);
    assign irq_tx_empty_o = tx_empty;

    // RX Sync & FSM
    reg rx_ff1, rx_ff2;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_ff1 <= 1'b1; rx_ff2 <= 1'b1;
        end else begin
            rx_ff1 <= MCR_q[4] ? tx_out : rx_i;
            rx_ff2 <= rx_ff1;
        end
    end
    wire rx_sync = rx_ff2;

    localparam RX_IDLE = 3'd0, RX_START = 3'd1, RX_DATA = 3'd2, RX_PARITY = 3'd3, RX_STOP = 3'd4;
    reg [2:0] rx_state;
    reg [7:0] rx_shift;
    reg [3:0] rx_sample_cnt;
    reg [2:0] rx_bit_cnt;
    reg       rx_parity_bit;
    reg       rx_pe, rx_fe, rx_bi;
    reg       rx_done;
    reg [7:0] rx_byte_out;
    reg       rx_pe_out, rx_fe_out, rx_bi_out;
    wire [2:0] rx_word_len = {1'b0, LCR_q[1:0]} + 3'd5;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_state <= RX_IDLE; rx_shift <= 0; rx_sample_cnt <= 0; rx_bit_cnt <= 0;
            rx_parity_bit <= 1'b0; rx_pe <= 1'b0; rx_fe <= 1'b0; rx_bi <= 1'b0;
            rx_done <= 1'b0; rx_byte_out <= 0; rx_pe_out <= 1'b0; rx_fe_out <= 1'b0; rx_bi_out <= 1'b0;
        end else begin
            rx_done <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    rx_pe <= 1'b0; rx_fe <= 1'b0; rx_bi <= 1'b0;
                    if (~rx_sync) begin
                        rx_state <= RX_START; rx_sample_cnt <= 0; rx_bit_cnt <= 0; rx_parity_bit <= LCR_q[4];
                    end
                end
                RX_START: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd7) begin
                            rx_sample_cnt <= 0;
                            if (~rx_sync) rx_state <= RX_DATA;
                            else rx_state <= RX_IDLE;
                        end else begin
                            rx_sample_cnt <= rx_sample_cnt + 1'b1;
                        end
                    end
                end
                RX_DATA: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= 0;
                            rx_shift <= {rx_sync, rx_shift[7:1]};
                            rx_parity_bit <= rx_parity_bit ^ rx_sync;
                            if (rx_bit_cnt == rx_word_len - 1'b1) begin
                                rx_bit_cnt <= 0; rx_state <= LCR_q[3] ? RX_PARITY : RX_STOP;
                            end else rx_bit_cnt <= rx_bit_cnt + 1'b1;
                        end else rx_sample_cnt <= rx_sample_cnt + 1'b1;
                    end
                end
                RX_PARITY: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= 0;
                            if (LCR_q[5]) rx_pe <= (rx_sync != ~LCR_q[4]);
                            else rx_pe <= (rx_sync != rx_parity_bit);
                            rx_state <= RX_STOP;
                        end else rx_sample_cnt <= rx_sample_cnt + 1'b1;
                    end
                end
                RX_STOP: begin
                    if (baud_tick_x16) begin
                        if (rx_sample_cnt == 4'd15) begin
                            rx_sample_cnt <= 0;
                            rx_fe <= ~rx_sync;
                            rx_bi <= ~rx_sync & ~(|rx_shift);
                            rx_byte_out <= rx_shift;
                            rx_pe_out <= rx_pe; rx_fe_out <= ~rx_sync; rx_bi_out <= ~rx_sync & ~(|rx_shift);
                            rx_done <= 1'b1; rx_state <= RX_IDLE;
                        end else rx_sample_cnt <= rx_sample_cnt + 1'b1;
                    end
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // RX FIFO
    reg [10:0] rx_fifo_mem [0:RX_FIFO_DEPTH-1];
    reg [4:0]  rx_wr_ptr, rx_rd_ptr;
    wire       rx_push, rx_pop_en, rx_full, rx_empty;
    wire [4:0] rx_count = rx_wr_ptr - rx_rd_ptr;

    assign rx_full  = (rx_count == RX_FIFO_DEPTH[4:0]);
    assign rx_empty = (rx_count == 5'd0);
    assign rx_push  = rx_done & ~rx_full;
    assign irq_rx_overrun_o = rx_done & rx_full;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_wr_ptr <= 0; rx_rd_ptr <= 0;
        end else begin
            if (wr_en & (PADDR[2:0] == ADDR_IIR_FCR) & PWDATA[1]) begin
                rx_wr_ptr <= 0; rx_rd_ptr <= 0;
            end else begin
                if (rx_push) begin
                    rx_fifo_mem[rx_wr_ptr[3:0]] <= {rx_bi_out, rx_fe_out, rx_pe_out, rx_byte_out};
                    rx_wr_ptr <= rx_wr_ptr + 1'b1;
                end
                if (rx_pop_en) rx_rd_ptr <= rx_rd_ptr + 1'b1;
            end
        end
    end

    assign rx_pop_en = rd_en & (~dlab) & (PADDR[2:0] == ADDR_RBR_THR_DLL) & ~rx_empty;
    wire [10:0] rx_head = rx_empty ? 11'h0 : rx_fifo_mem[rx_rd_ptr[3:0]];

    assign rx_fifo_not_empty_o = ~rx_empty;
    assign rx_fifo_full_o      = rx_full;
    assign rx_fifo_elements_o  = rx_count;

    // RX Timeout
    reg [7:0] rx_timeout_cnt;
    reg       rx_timeout_flag;
    wire [7:0] char_time = ({5'b0, rx_word_len} + 8'd2) << 4;

    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            rx_timeout_cnt <= 0; rx_timeout_flag <= 1'b0;
        end else begin
            if (rx_empty | rx_pop_en | rx_done) begin
                rx_timeout_cnt <= 0; rx_timeout_flag <= 1'b0;
            end else if (baud_tick_x16 & ~rx_empty) begin
                if (rx_timeout_cnt >= (char_time << 2)) rx_timeout_flag <= 1'b1;
                else rx_timeout_cnt <= rx_timeout_cnt + 1'b1;
            end
        end
    end
    assign irq_rx_timeout_o = rx_timeout_flag;

    // LSR & IIR Logic
    reg lsr_oe, lsr_pe, lsr_fe, lsr_bi;
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            lsr_oe <= 1'b0; lsr_pe <= 1'b0; lsr_fe <= 1'b0; lsr_bi <= 1'b0;
        end else begin
            if (irq_rx_overrun_o) lsr_oe <= 1'b1;
            if (rx_push & rx_pe_out) lsr_pe <= 1'b1;
            if (rx_push & rx_fe_out) lsr_fe <= 1'b1;
            if (rx_push & rx_bi_out) lsr_bi <= 1'b1;
            if (rd_en & (PADDR[2:0] == ADDR_LSR)) begin
                lsr_oe <= 1'b0; lsr_pe <= 1'b0; lsr_fe <= 1'b0; lsr_bi <= 1'b0;
            end
        end
    end

    assign irq_rx_parity_o = lsr_pe;
    assign irq_rx_frame_o  = lsr_fe;
    assign irq_rx_break_o  = lsr_bi;
    wire lsr_fifoer = rx_head[10] | rx_head[9] | rx_head[8];

    always @* begin
        LSR_q = 8'h00;
        LSR_q[0] = ~rx_empty; LSR_q[1] = lsr_oe; LSR_q[2] = lsr_pe; LSR_q[3] = lsr_fe;
        LSR_q[4] = lsr_bi; LSR_q[5] = lsr_thre; LSR_q[6] = lsr_temt; LSR_q[7] = lsr_fifoer;
    end

    wire [4:0] rx_trigger = (FCR_q[7:6] == 2'b00) ? 5'd1  :
                            (FCR_q[7:6] == 2'b01) ? 5'd4  :
                            (FCR_q[7:6] == 2'b10) ? 5'd8  : 5'd14;
    assign irq_rx_data_avail_o = (rx_count >= rx_trigger);

    wire line_int, rx_int_pend, rx_tout_pend, tx_int_pend, ms_int_pend;
    assign line_int    = IER_q[2] & (lsr_oe | lsr_pe | lsr_fe | lsr_bi);
    assign rx_int_pend = IER_q[0] & irq_rx_data_avail_o;
    assign rx_tout_pend= IER_q[0] & rx_timeout_flag;
    assign tx_int_pend = IER_q[1] & lsr_thre;
    assign ms_int_pend = IER_q[3] & (msr_ext_i[3:0] != 4'h0);

    always @* begin
        if (line_int) IIR_q = 8'h06;
        else if (rx_int_pend & ~rx_tout_pend) IIR_q = 8'h04;
        else if (rx_tout_pend) IIR_q = 8'h0C;
        else if (tx_int_pend) IIR_q = 8'h02;
        else if (ms_int_pend) IIR_q = 8'h00;
        else IIR_q = 8'h01;
    end

    // Reg File Write & Read
    always @(posedge CLK or negedge RSTN) begin
        if (!RSTN) begin
            DLL_q <= 0; DLM_q <= 0; IER_q <= 0; FCR_q <= 0; LCR_q <= 0; MCR_q <= 0; SCR_q <= 0;
        end else if (wr_en) begin
            case (PADDR[2:0])
                ADDR_RBR_THR_DLL: if (dlab) DLL_q <= PWDATA[7:0];
                ADDR_IER_DLM:     if (dlab) DLM_q <= PWDATA[7:0]; else IER_q <= PWDATA[7:0];
                ADDR_IIR_FCR:     FCR_q <= PWDATA[7:0];
                ADDR_LCR:         LCR_q <= PWDATA[7:0];
                ADDR_MCR:         MCR_q <= {3'b0, PWDATA[4:0]};
                ADDR_SCR:         SCR_q <= PWDATA[7:0];
                default: ;
            endcase
        end
    end

    assign msr_read_o = rd_en & (PADDR[2:0] == ADDR_MSR);

    always @* begin
        PRDATA = 32'h0;
        if (rd_en) begin
            case (PADDR[2:0])
                ADDR_RBR_THR_DLL: PRDATA = dlab ? {24'h0, DLL_q} : {24'h0, rx_head[7:0]};
                ADDR_IER_DLM:     PRDATA = dlab ? {24'h0, DLM_q} : {24'h0, IER_q};
                ADDR_IIR_FCR:     PRDATA = {24'h0, IIR_q};
                ADDR_LCR:         PRDATA = {24'h0, LCR_q};
                ADDR_MCR:         PRDATA = {24'h0, 3'b0, MCR_q[4:0]};
                ADDR_LSR:         PRDATA = {24'h0, LSR_q};
                ADDR_MSR:         PRDATA = {24'h0, msr_ext_i};
                ADDR_SCR:         PRDATA = {24'h0, SCR_q};
                default:          PRDATA = 32'h0;
            endcase
        end
    end

    assign mcr_dtr_o  = MCR_q[0]; assign mcr_rts_o  = MCR_q[1];
    assign mcr_out1_o = MCR_q[2]; assign mcr_out2_o = MCR_q[3];
    assign mcr_lb_o   = MCR_q[4]; assign ier_o      = IER_q[3:0];
    assign event_o    = ~IIR_q[0];

endmodule