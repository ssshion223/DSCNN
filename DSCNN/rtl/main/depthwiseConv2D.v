//==============================================================================
// 模块名：depthwiseConv2D
// 功能说明：
//   顶层深度可分离卷积计算单元。
//   串接流式卷积与后处理（Bias + 量化 + ReLU）两级数据通路。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_ready/in_pixel：上游像素流握手接口
//   out_ready：下游反压信号
// 输出接口：
//   out_valid/out_pixel：后处理输出像素流
//   out_end_frame/out_end_all_frame：帧边界标志
// 时序特性：
//   全流水设计，输入输出均采用 valid/ready 握手。
//==============================================================================
module depthwiseConv2D #(
    // ==========================================
    // 卷积核心参数（MAC 与感受野）
    // ==========================================
    parameter integer DATA_W        = 8,                      // 输入像素位宽
    parameter integer COEFF_W       = 8,                      // 卷积系数位宽
    parameter integer K_H           = 3,                      // 卷积核高度
    parameter integer K_W           = 3,                      // 卷积核宽度
    parameter integer MUL_W         = DATA_W + COEFF_W,       // 乘法结果位宽
    parameter integer SUM_W         = MUL_W + $clog2(K_H*K_W), // K_H*K_W=1*1时的累加位宽
    parameter integer COL           = 49,                     // 输入特征图列数
    parameter integer ROW           = 10,                     // 输入特征图行数
    parameter integer STRIDE        = 2,                      // 卷积步长
    parameter integer PAD_TOP       = (K_H-1)/2,                  // 上边界补零
    parameter integer PAD_BOTTOM    = K_H/2,                  // 下边界补零
    parameter integer PAD_LEFT      = (K_W-1)/2,                  // 左边界补零
    parameter integer PAD_RIGHT     = K_W/2,                  // 右边界补零
    parameter integer COEFF_GRP_NUM = 64,                      // 系数组数量
    parameter integer FRAME_GRP_NUM = 64,                      // 输入帧组数量
    parameter integer MAC_PIPELINE  = 1,                      // MAC树流水级模式
    parameter         COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_dw0_Fold_bias.hex", // 系数初始化文件
    
    // ==========================================
    // 后处理参数（Bias、量化、激活、缓冲）
    // ==========================================
    parameter integer OUT_WIDTH     = 8,  // 最终输出量化位宽
    parameter integer SHIFT_VAL     = 16, // 量化右移位数
    parameter integer BIAS_GROUP_SIZE    = 1, // Bias组大小
    parameter integer BIAS_GROUP_BITS    = $clog2(BIAS_GROUP_SIZE),  // Bias组号位宽
    parameter integer BIAS_CH_BITS       = 6,  // Bias通道号位宽
    parameter integer FIFO_DEPTH    = 16, // 输出缓冲 FIFO 深度
    parameter integer FIFO_AF_LEVEL = 10,  // FIFO 将满阈值
    parameter         BIAS_INIT_FILE= "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw0_Fold_bias.hex", // Bias 初始化文件
    parameter         MULT_CNT      = 1,
    parameter signed [11:0] MULT_FACTOR0  = 12'sd1246,
    parameter signed [11:0] MULT_FACTOR1  = 12'sd828,
    parameter signed [11:0] MULT_FACTOR2  = 12'sd652,
    parameter signed [11:0] MULT_FACTOR3  = 12'sd412
)(
    input  wire                               clk,             // 时钟信号
    input  wire                               rst_n,           // 低有效复位

    // ==========================================
    // 1) 上游输入接口（像素流输入卷积）
    // ==========================================
    input  wire                               in_valid,        // 输入数据有效
    output wire                               in_ready,        // 输入就绪反馈
    input  wire signed [DATA_W-1:0]           in_pixel,        // 输入像素数据
    input  wire                               in_end_all_frame,// 全部组帧结束标志（上游）

    // ==========================================
    // 2) 下游输出接口（量化/ReLU 后输出）
    // ==========================================
    output wire        [OUT_WIDTH-1:0]        out_pixel,       // 输出像素数据
    output wire                               out_valid,       // 输出数据有效
    input  wire                               out_ready,       // 下游接收就绪
    
    // 帧边界侧带标志
    output wire                               out_end_frame,   // 当前通道帧结束标志
    output wire                               out_end_all_frame// 全部组帧结束标志
);

    // ==========================================
    // 卷积与 Bias/量化/ReLU 之间的内部流接口
    // ==========================================
    wire                               conv_out_valid;
    wire                               conv_out_ready;
    wire                               conv_out_end_all_frame;
    wire                               conv_out_end_frame;
    wire signed [SUM_W-1:0]            conv_out_pixel_data_bus;

    // ==========================================
    // 阶段1：流式卷积
    // ==========================================
    matrix_conv2d_stream_parallel #(
        .DATA_W          (DATA_W),
        .COEFF_W         (COEFF_W),
        .SUM_W           (SUM_W),
        .COL             (COL),
        .ROW             (ROW),
        .K_H             (K_H),
        .K_W             (K_W),
        .STRIDE          (STRIDE),
        .PAD_TOP         (PAD_TOP),
        .PAD_BOTTOM      (PAD_BOTTOM),
        .PAD_LEFT        (PAD_LEFT),
        .PAD_RIGHT       (PAD_RIGHT),
        .OUT_CH          (1),
        .COEFF_GRP_NUM   (COEFF_GRP_NUM),
        .FRAME_GRP_NUM   (FRAME_GRP_NUM),
        .MAC_PIPELINE    (MAC_PIPELINE),
        .COEFF_INIT_FILE (COEFF_INIT_FILE),
        .OUT_FIFO_DEPTH  (FIFO_DEPTH),
        .OUT_FIFO_AF_LEVEL(FIFO_AF_LEVEL)
    ) u_conv2d (
        .clk                 (clk),
        .rst_n               (rst_n),
        
        .in_valid            (in_valid),
        .in_ready            (in_ready),
        .in_pixel            (in_pixel),
        .in_end_all_frame    (in_end_all_frame),
        
        // 接入内部中间连线
        .out_valid           (conv_out_valid),
        .out_ready           (conv_out_ready),
        .out_end_all_frame   (conv_out_end_all_frame),
        .out_end_frame       (conv_out_end_frame),
        .out_pixel_data_bus  (conv_out_pixel_data_bus)
    );

    // ==========================================
    // 阶段2：Bias + 量化 + ReLU
    // ==========================================
    bias_process_wrapper #(
        .IN_WIDTH      (SUM_W), // 来自卷积输出的位宽
        .OUT_WIDTH     (OUT_WIDTH),
        .SHIFT_VAL     (SHIFT_VAL),
        .GROUP_BITS    (BIAS_GROUP_BITS),
        .GROUP_SIZE    (BIAS_GROUP_SIZE),
        .CH_BITS       (BIAS_CH_BITS),
        .FIFO_DEPTH    (FIFO_DEPTH),
        .FIFO_AF_LEVEL (FIFO_AF_LEVEL),
        .BIAS_INIT_FILE(BIAS_INIT_FILE),
        .MULT_CNT      (MULT_CNT),
        .MULT_FACTOR0  (MULT_FACTOR0),
        .MULT_FACTOR1  (MULT_FACTOR1),
        .MULT_FACTOR2  (MULT_FACTOR2),
        .MULT_FACTOR3  (MULT_FACTOR3)
    ) u_bias_quant_relu (
        .clk              (clk),
        .rst_n            (rst_n),

        // 来自卷积阶段的输入流
        .in_pixel_data_bus (conv_out_pixel_data_bus),
        .in_valid          (conv_out_valid),
        .in_end_frame      (conv_out_end_frame),
        .in_end_all_frame  (conv_out_end_all_frame),
        .in_ready          (conv_out_ready),

        // 输出到下游模块的结果流
        .out_pixel_data     (out_pixel),
        .out_valid          (out_valid),
        .out_end_frame      (out_end_frame),
        .out_end_all_frame  (out_end_all_frame),
        .out_ready          (out_ready)
    );

    // ==========================================
    reg [31:0] dw_in_cnt,dw_out_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dw_in_cnt <= 32'b0;
            dw_out_cnt <= 32'b0;
        end else  begin
            // if(in_valid&&in_ready) begin
			// 	$display("beat %d: DW_in_pixel: %d", dw_in_cnt, $signed(in_pixel));
			// 	dw_in_cnt <= dw_in_cnt + 1;
			// end
            // if (conv_out_valid && conv_out_ready) begin
            //     $display("beat %d: DW_out_pixel: %d", dw_out_cnt, $signed(conv_out_pixel_data_bus));
            // dw_out_cnt <= dw_out_cnt + 1;
            // end
        end
    end


endmodule