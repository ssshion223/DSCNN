//==============================================================================
// Module: matrix_conv2d_stream (top)
//==============================================================================

module matrix_conv2d_stream #(
    parameter integer DATA_W   = 8,
    parameter integer COEFF_W  = 8,
    parameter integer ACC_W    = 32,
    parameter integer COL      = 20,
    parameter integer ROW      = 20,
    parameter integer K        = 5,
    parameter integer STRIDE   = 1,
    parameter integer OUT_CH   = 3,
    parameter integer MAC_PIPELINE = 1,
    parameter         COEFF_INIT_FILE = "E:/WorkSpace/Robei/DSCNN/test/coeff_int.memh",
    parameter integer COEFF_ADDR_W = (((K * K) <= 1) ? 1 : $clog2(K * K)),
    parameter integer COEFF_CH_W   = ((OUT_CH <= 1) ? 1 : $clog2(OUT_CH))
)(
    input  wire                             clk,
    input  wire                             rst_n,

    input  wire                             in_valid,
    output wire                             in_ready,
    input  wire signed [DATA_W-1:0]         in_pixel,

    output wire                             out_valid,
    input  wire                             out_ready,

    output wire                             out_pixel_end_line, // 新增信号，表示输出像素的行结束
    output wire                             out_pixel_end_frame, // 新增信号，表示输出像素的帧结束
    output wire signed [ACC_W-1:0]          out_pixel_data
);

    localparam integer WIN_SIZE = K * K;
    localparam integer OUT_COL = COL - K + 1;
    localparam integer OUT_ROW = ROW - K + 1;
    localparam integer OUT_COL_W = (OUT_COL <= 1) ? 1 : $clog2(OUT_COL);
    localparam integer OUT_ROW_W = (OUT_ROW <= 1) ? 1 : $clog2(OUT_ROW);

    wire window_in_valid,window_out_valid;
    wire window_in_ready,window_out_ready;
    wire[K*K*DATA_W-1:0] window_bus;
    reg [K*K*DATA_W-1:0] window_bus_hold;
    wire[K*K*COEFF_W-1:0] coeff_bus;
    wire signed [ACC_W-1:0] mac_sum;
    wire mac_valid;
    wire [1:0] user_out;
    wire fifo_almost_full;
    

    reg [COEFF_CH_W-1:0] mac_ch_idx, mac_ch_idx_align;
    reg [OUT_COL_W-1:0] col_counter;
    reg [OUT_ROW_W-1:0] row_counter;
    wire col_counter_wrap , row_counter_wrap;
    wire pixel_end_frame , pixel_end_line ;
    reg cac_trigger;
    wire cac_all_ch ;
    wire ch_idx_wrap;


    assign col_counter_wrap = (col_counter == OUT_COL - 1);
    assign row_counter_wrap = (row_counter == OUT_ROW - 1);
    assign pixel_end_line = col_counter_wrap && cac_all_ch;
    assign pixel_end_frame = pixel_end_line && row_counter_wrap;
    assign ch_idx_wrap = (mac_ch_idx == OUT_CH - 1);
    assign cac_all_ch = cac_trigger && (mac_ch_idx_align == OUT_CH - 1);
    assign window_out_ready = (mac_ch_idx == 0)&& window_out_valid;
    assign window_in_valid = in_valid && !fifo_almost_full;


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_counter <= {OUT_COL_W{1'b0}};
            row_counter <= {OUT_ROW_W{1'b0}};
        end else if (cac_all_ch) begin
            if(col_counter_wrap) begin
                col_counter <= {OUT_COL_W{1'b0}};
                if(row_counter_wrap) begin
                    row_counter <= {OUT_ROW_W{1'b0}};
                end else begin
                    row_counter <= row_counter + 1'b1;
                end
            end else begin
                col_counter <= col_counter + 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        mac_ch_idx_align <= mac_ch_idx;
    end


    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            cac_trigger <= 1'b0;
        end else begin
            if(window_out_valid) begin
                cac_trigger <= 1'b1;
            end else if (cac_all_ch) begin
                cac_trigger <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            mac_ch_idx <= {COEFF_CH_W{1'b0}};
        end else begin
            if(window_out_valid || (cac_trigger && !cac_all_ch )) begin
                mac_ch_idx <= (ch_idx_wrap) ? {COEFF_CH_W{1'b0}} : mac_ch_idx + 1'b1;
            end else begin
                mac_ch_idx <= {COEFF_CH_W{1'b0}};
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            window_bus_hold <= {(K*K*DATA_W){1'b0}};
        end else begin
            if(window_out_ready) begin
                window_bus_hold <= window_bus;
            end
        end
    end


    conv_sliding_window_kxk #(
        .DATA_W(DATA_W),
        .COL(COL),
        .ROW(ROW),
        .K(K),
        .STRIDE(STRIDE)
    ) u_conv_sliding_window_kxk (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(window_in_valid), 
        .in_ready(window_in_ready),
        .in_pixel(in_pixel),
        .out_valid(window_out_valid),
        .out_ready(window_out_ready),
        .window_bus(window_bus)
    );
    conv_coeff_store #(
        .COEFF_W(COEFF_W),
        .K(K),
        .OUT_CH(OUT_CH),
        .INIT_FILE(COEFF_INIT_FILE)
    ) u_conv_coeff_store (
        .clk(clk),
        .coeff_rd_ch(mac_ch_idx),
        .coeff_bus(coeff_bus)
    );
   conv_mac_tree_kxk #(
        .DATA_W(DATA_W),
        .COEFF_W(COEFF_W),
        .ACC_W(ACC_W),
        .K(K),
        .PIPELINE(MAC_PIPELINE),
        .USER_W(2) // 传递像素开始信号
    ) u_conv_mac_tree_kxk (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(cac_trigger),
        .in_user({pixel_end_frame, pixel_end_line}), // 传递像素开始和行结束信号
        .window_bus(window_bus_hold),
        .coeff_bus(coeff_bus),
        .mac_sum(mac_sum),
        .out_valid(mac_valid),
        .out_user(user_out)
    );
    fwft_fifo_reg #(
        .WIDTH(ACC_W+2), // 扩宽以携带用户信号
        .DEPTH(16),
        .AF_LEVEL(12) // 设置 Almost Full 阈值为 12
    ) u_fwft_fifo_reg (
        .clk(clk),
        .rst_n(rst_n),
        .din({user_out, mac_sum}),
        .wr_en(mac_valid),
        .full(), 
        .almost_full(fifo_almost_full),
        .rd_en(out_ready),
        .dout({out_pixel_end_frame, out_pixel_end_line, out_pixel_data}),
        .valid(out_valid), // 直接使用 MAC 输出的 valid 信号
        .empty() // 可以选择连接 empty 信号以监控 FIFO 状态
    );

    assign in_ready = window_in_ready && !fifo_almost_full;

   
endmodule
