//==============================================================================
// 模块名：matrix_conv2d_stream_parallel
// 功能说明：
//   支持 OUT_CH 通道并行计算的滑窗卷积引擎。
//   每完成一帧输出后切换到下一组卷积系数。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_ready/in_pixel：输入特征图流握手
//   out_ready：下游消费端反压
// 输出接口：
//   out_valid/out_pixel_data_bus：并行通道卷积和输出
//   out_end_frame/out_end_all_frame：帧边界标志
// 时序特性：
//   滑窗与 MAC 全流程流水化，端到端采用 valid/ready 握手。
//==============================================================================

module matrix_conv2d_stream_parallel #(
    parameter integer DATA_W   = 8,                         // 输入像素位宽
    parameter integer COEFF_W  = 8,                         // 系数位宽
    parameter integer K_H      = 1,                         // 卷积核高度
    parameter integer K_W      = 1,                         // 卷积核宽度
    parameter integer MUL_W    = DATA_W + COEFF_W, // 乘积位宽等于数据位宽加权重位宽以防止溢出
    parameter integer SUM_W    = MUL_W + $clog2(K_H*K_W),  // 累加输出位宽
    parameter integer COL      = 20,                        // 输入列数
    parameter integer ROW      = 20,                        // 输入行数
    parameter integer STRIDE   = 1,                         // 步长
    parameter integer PAD_TOP    = K_H/2,                   // 上补零
    parameter integer PAD_BOTTOM = K_H/2,                   // 下补零
    parameter integer PAD_LEFT   = K_W/2,                   // 左补零
    parameter integer PAD_RIGHT  = K_W/2,                   // 右补零
    parameter integer OUT_CH   = 64,                        // 并行输出通道数
    parameter integer COEFF_GRP_NUM = 2,                    // 系数组总数
    parameter integer FRAME_GRP_NUM = 1,                    // 输入帧组数
    parameter integer MAC_PIPELINE = 1,                     // MAC加法树流水模式
    parameter         COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh", // 系数初始化文件路径
    // 输出侧 FIFO 可配置参数（fwft_fifo_reg）
    parameter integer OUT_FIFO_DEPTH   = 16, // 默认 FIFO 深度
    parameter integer OUT_FIFO_AF_LEVEL= 14  // 默认几乎满阈值
)(
    input  wire                                 clk,         // 时钟信号
    input  wire                                 rst_n,       // 低有效复位

    input  wire                                 in_valid,    // 输入有效
    output wire                                 in_ready,    // 输入就绪
    input  wire signed [DATA_W-1:0]             in_pixel,    // 输入像素数据
    input  wire                                 in_end_all_frame, // 输入全部组帧结束标志

    output wire                                 out_valid,   // 输出有效
    input  wire                                 out_ready,   // 下游就绪

    output wire                                 out_end_all_frame, // 全部组帧结束
    output wire                                 out_end_frame,     // 单帧结束
    output wire signed [OUT_CH*SUM_W-1:0]       out_pixel_data_bus // 并行通道卷积输出
);

    localparam integer WIN_SIZE   = K_H * K_W;
    localparam integer P_COL      = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW      = ROW + PAD_TOP  + PAD_BOTTOM;
    localparam integer OUT_COL    = ((P_COL - K_W) / STRIDE) + 1;
    localparam integer OUT_ROW    = ((P_ROW - K_H) / STRIDE) + 1;
    localparam integer OUT_COL_W  = (OUT_COL <= 1) ? 1 : $clog2(OUT_COL);
    localparam integer OUT_ROW_W  = (OUT_ROW <= 1) ? 1 : $clog2(OUT_ROW);
    localparam integer ROM_OUT_CH = OUT_CH * COEFF_GRP_NUM;
    localparam integer GRP_W      = (COEFF_GRP_NUM <= 1) ? 1 : $clog2(COEFF_GRP_NUM);
    localparam integer FRAME_GRP_W    = (FRAME_GRP_NUM <= 1) ? 1 : $clog2(FRAME_GRP_NUM);
    localparam integer CH_COEFF_BUS_W = K_H*K_W*COEFF_W;
    localparam integer GRP_COEFF_BUS_W = OUT_CH*CH_COEFF_BUS_W;

    wire window_in_valid;
    wire window_in_ready;
    wire window_out_valid;
    wire window_out_ready;
    wire signed [K_H*K_W*DATA_W-1:0] window_bus;
    wire window_out_fire;

    reg signed [K_H*K_W*DATA_W-1:0]  window_bus_hold;
    reg                          end_all_frame_hold;
    reg                          end_frame_hold;
    reg                          mac_start_d;

    reg [OUT_COL_W-1:0] out_win_col_cnt;
    reg [OUT_ROW_W-1:0] in_win_col_cnt;
    wire end_frame,end_all_frame;

    wire win_col_wrap;
    wire win_row_wrap;
    wire coeff_grp_wrap;
    wire frame_end_fire;
    wire frame_id_wrap;

    wire fifo_almost_full;

    wire signed [OUT_CH*SUM_W-1:0] mac_sum_bus;
    wire [OUT_CH-1:0]              mac_valid_bus;
    wire [OUT_CH*2-1:0]            mac_user_bus;

    wire [1:0] out_user;
    wire out_fire;
    reg  [GRP_W-1:0] coeff_grp_idx;
    reg  [FRAME_GRP_W-1:0] frame_idx;
    wire signed [GRP_COEFF_BUS_W-1:0] coeff_bus_grp;

    assign win_col_wrap = (out_win_col_cnt == OUT_COL - 1);
    assign win_row_wrap = (in_win_col_cnt == OUT_ROW - 1);
    assign coeff_grp_wrap = (coeff_grp_idx == COEFF_GRP_NUM - 1);
    assign frame_id_wrap = (frame_idx == FRAME_GRP_NUM - 1);
    assign end_frame = win_col_wrap && win_row_wrap;
    assign end_all_frame = end_frame && frame_id_wrap;
    assign frame_end_fire = window_out_fire && end_frame;

    // 只要后端 FIFO 不接近满，就继续接收窗口
    assign window_out_ready = !fifo_almost_full;
    assign window_in_valid  = in_valid && !fifo_almost_full;
    assign window_out_fire  = window_out_valid && window_out_ready;

    // 对上游输入的 backpressure
    assign in_ready = window_in_ready && !fifo_almost_full;
    assign out_fire = out_valid && out_ready;

    // Switch to next coefficient group after one full output frame.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coeff_grp_idx <= {GRP_W{1'b0}};
        end else if (frame_end_fire) begin
            if (coeff_grp_wrap) begin
                coeff_grp_idx <= {GRP_W{1'b0}};
            end else begin
                coeff_grp_idx <= coeff_grp_idx + 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_idx <= {FRAME_GRP_W{1'b0}};
        end else if (frame_end_fire) begin
            if (frame_id_wrap) begin
                frame_idx <= {FRAME_GRP_W{1'b0}};
            end else begin
                frame_idx <= frame_idx + 1'b1;
            end
        end
    end

    // 统计窗口输出坐标，用于生成当前窗口的 end_line / end_frame
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_win_col_cnt <= {OUT_COL_W{1'b0}};
            in_win_col_cnt <= {OUT_ROW_W{1'b0}};
        end else if (window_out_fire) begin
            if (win_col_wrap) begin
                out_win_col_cnt <= {OUT_COL_W{1'b0}};
                if (win_row_wrap) begin
                    in_win_col_cnt <= {OUT_ROW_W{1'b0}};
                end else begin
                    in_win_col_cnt <= in_win_col_cnt + 1'b1;
                end
            end else begin
                out_win_col_cnt <= out_win_col_cnt + 1'b1;
            end
        end
    end

    // Latch window and flags, then delay one cycle before starting MAC.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_bus_hold <= {(K_H*K_W*DATA_W){1'b0}};
            end_all_frame_hold   <= 1'b0;
            end_frame_hold  <= 1'b0;
            mac_start_d     <= 1'b0;
        end else begin
            mac_start_d <= window_out_fire;
            if (window_out_fire) begin
                window_bus_hold <= window_bus;
                end_all_frame_hold   <= end_all_frame;
                end_frame_hold  <= end_frame;
            end
        end
    end

    conv_sliding_padding #(
        .DATA_W(DATA_W),
        .COL(COL),
        .ROW(ROW),
        .K_H(K_H),
        .K_W(K_W),
        .STRIDE(STRIDE),
        .PAD_TOP(PAD_TOP),
        .PAD_BOTTOM(PAD_BOTTOM),
        .PAD_LEFT(PAD_LEFT),
        .PAD_RIGHT(PAD_RIGHT)
    ) u_conv_sliding_padding (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(window_in_valid),
        .in_ready(window_in_ready),
        .end_all_frame(in_end_all_frame && in_valid && in_ready), // 直接将上游的 end_all_frame 信号传递到滑窗模块，确保在正确的时钟周期被采样
        .in_pixel(in_pixel),
        .out_valid(window_out_valid),
        .out_ready(window_out_ready),
        .out_window_bus(window_bus)
    );

    conv_coeff_store #(
        .COEFF_W(COEFF_W),
        .K_H(K_H),
        .K_W(K_W),
        .OUT_CH(OUT_CH),
        .COEFF_GRP_NUM(COEFF_GRP_NUM),
        .INIT_FILE(COEFF_INIT_FILE)
    ) u_conv_coeff_store_grp (
        .clk(clk),
        .coeff_rd_grp(coeff_grp_idx),
        .coeff_bus(coeff_bus_grp)
    );

    genvar gch;
    generate
        for (gch = 0; gch < OUT_CH; gch = gch + 1) begin : gen_ch_parallel
            wire signed [K_H*K_W*COEFF_W-1:0] coeff_bus_ch;
            assign coeff_bus_ch = coeff_bus_grp[gch*CH_COEFF_BUS_W +: CH_COEFF_BUS_W];

            conv_mac_tree #(
                .DATA_W(DATA_W),
                .COEFF_W(COEFF_W),
                .MUL_W(MUL_W),
                .SUM_W(SUM_W),
                .K_H(K_H),
                .K_W(K_W),
                .PIPELINE(MAC_PIPELINE),
                .USER_W(2)
            ) u_conv_mac_tree (
                .clk(clk),
                .rst_n(rst_n),
                .in_valid(mac_start_d),
                .in_user({end_all_frame_hold, end_frame_hold}),
                .window_bus(window_bus_hold),
                .coeff_bus(coeff_bus_ch),
                .mac_sum(mac_sum_bus[gch*SUM_W +: SUM_W]),
                .out_valid(mac_valid_bus[gch]),
                .out_user(mac_user_bus[gch*2 +: 2])
            );
        end
    endgenerate

    // 并行通道 valid/user 理论上一致，取ch[0]
    fwft_fifo_reg #(
        .WIDTH(OUT_CH*SUM_W + 2),
        .DEPTH(OUT_FIFO_DEPTH),
        .AF_LEVEL(OUT_FIFO_AF_LEVEL)
    ) u_fwft_fifo_reg (
        .clk(clk),
        .rst_n(rst_n),
        .din({mac_user_bus[1:0], mac_sum_bus}),
        .wr_en(mac_valid_bus[0]),
        .full(),
        .almost_full(fifo_almost_full),
        .rd_en(out_ready),
        .dout({out_user, out_pixel_data_bus}),
        .valid(out_valid),
        .empty()
    );

    assign out_end_all_frame = out_user[1];
    assign out_end_frame  = out_user[0];

    //test
	// reg[SUM_W-1:0] test_sum[0:OUT_CH-1];
    // reg[DATA_W-1:0]test_input;
    // reg[COEFF_W*K_H*K_W-1:0] test_coeff[0:OUT_CH-1];
    // reg [31:0] out_cnt,in_pixel_cnt;
    // always @(posedge clk or negedge rst_n) begin
    //     if(!rst_n) begin
    //         out_cnt <= 0;
    //         in_pixel_cnt <= 0;
    //     end else  begin
    //         if(out_valid && out_ready) begin
    //             $display("beat %d: matrix_out_pixel: %d", out_cnt, $signed(out_pixel_data_bus));
    //             out_cnt <= out_cnt + 1;
    //         end
    //         if(in_valid && in_ready) begin
    //             $display("beat %d: in_pixel: %d", in_pixel_cnt, $signed(in_pixel));
    //             in_pixel_cnt <= in_pixel_cnt + 1;
    //         end
    //     end
    // end


endmodule
