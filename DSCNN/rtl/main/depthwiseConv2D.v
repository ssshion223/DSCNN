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
    parameter integer K_H           = 1,                      // 卷积核高度
    parameter integer K_W           = 1,                      // 卷积核宽度
    parameter integer MUL_W         = DATA_W + COEFF_W,       // 乘法结果位宽
    parameter integer SUM_W         = MUL_W + $clog2(K_H*K_W), // K_H*K_W=1*1时的累加位宽
    parameter integer COL           = 20,                     // 输入特征图列数
    parameter integer ROW           = 20,                     // 输入特征图行数
    parameter integer STRIDE        = 1,                      // 卷积步长
    parameter integer PAD_TOP       = K_H/2,                  // 上边界补零
    parameter integer PAD_BOTTOM    = K_H/2,                  // 下边界补零
    parameter integer PAD_LEFT      = K_W/2,                  // 左边界补零
    parameter integer PAD_RIGHT     = K_W/2,                  // 右边界补零
    parameter integer COEFF_GRP_NUM = 2,                      // 系数组数量
    parameter integer MAC_PIPELINE  = 1,                      // MAC树流水级模式
    parameter         COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/rtl/dw/DS-CNN_dw0_Fold_bias.hex", // 系数初始化文件
    
    // ==========================================
    // 后处理参数（Bias、量化、激活、缓冲）
    // ==========================================
    parameter integer OUT_WIDTH     = 8,  // 最终输出量化位宽
    parameter integer SHIFT_VAL     = 24, // 量化右移位数
    parameter integer GROUP_BITS    = 4,  // Bias组号位宽
    parameter integer GROUP_SIZE    = 16, // Bias组大小
    parameter integer CH_BITS       = 6,  // Bias通道号位宽
    parameter integer FIFO_DEPTH    = 16, // 输出缓冲 FIFO 深度
    parameter integer FIFO_AF_LEVEL = 14  // FIFO 将满阈值
)(
    input  wire                               clk,             // 时钟信号
    input  wire                               rst_n,           // 低有效复位

    // ==========================================
    // 1) 上游输入接口（像素流输入卷积）
    // ==========================================
    input  wire                               in_valid,        // 输入数据有效
    output wire                               in_ready,        // 输入就绪反馈
    input  wire signed [DATA_W-1:0]           in_pixel,        // 输入像素数据

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
        .MAC_PIPELINE    (MAC_PIPELINE),
        .COEFF_INIT_FILE (COEFF_INIT_FILE)
    ) u_conv2d (
        .clk                 (clk),
        .rst_n               (rst_n),
        
        .in_valid            (in_valid),
        .in_ready            (in_ready),
        .in_pixel            (in_pixel),
        
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
        .GROUP_BITS    (GROUP_BITS),
        .GROUP_SIZE    (GROUP_SIZE),
        .CH_BITS       (CH_BITS),
        .FIFO_DEPTH    (FIFO_DEPTH),
        .FIFO_AF_LEVEL (FIFO_AF_LEVEL)
    ) u_bias_quant_relu (
        .clk              (clk),
        .rst_n            (rst_n),

        // 来自卷积阶段的输入流
        .s_pixel_data_bus (conv_out_pixel_data_bus),
        .s_valid          (conv_out_valid),
        .s_end_frame      (conv_out_end_frame),
        .s_end_all_frame  (conv_out_end_all_frame),
        .s_ready          (conv_out_ready),

        // 输出到下游模块的结果流
        .m_pixel_data     (out_pixel),
        .m_valid          (out_valid),
        .m_end_frame      (out_end_frame),
        .m_end_all_frame  (out_end_all_frame),
        .m_ready          (out_ready)
    );

endmodule