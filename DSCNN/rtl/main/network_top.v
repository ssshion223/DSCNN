`timescale 1ns / 1ps

//==============================================================================
// 模块名：network_top
// 功能说明：
//   端到端神经网络顶层模块。
//   [前端]：depthwiseConv2D (1x49x10 -> 64x25x5 降采样与通道扩展)
//   [后端]：dw_pw_pingpong  (处理后续的多轮 DW+PW 乒乓流水线)
//   注意:前端输入需要重复输入通道次数的像素数据以适配后端的 64 通道输入要求。(该设计中为1x49x10重复64次)
//==============================================================================
module network_top #(
    // ==========================================
    // 阶段 1：降采样层 (Downsample Layer - DS) 
    // 默认配置为 10x4 卷积核，步长 2
    // ==========================================
    parameter integer DS_DATA_W        = 8,
    parameter integer DS_COEFF_W       = 8,
    parameter integer DS_K_H           = 10,
    parameter integer DS_K_W           = 4,
    parameter integer DS_COL           = 10,  // 原始输入宽
    parameter integer DS_ROW           = 49,  // 原始输入高
    parameter integer DS_STRIDE        = 2,   // 核心降采样步长
    parameter integer DS_PAD_TOP       = 4,
    parameter integer DS_PAD_BOTTOM    = 5,
    parameter integer DS_PAD_LEFT      = 1,
    parameter integer DS_PAD_RIGHT     = 2,
    parameter integer DS_COEFF_GRP_NUM = 64,  // 输出 64 个通道(理论可以并行,但是总线压力大且数据流不一致,故单通道输出)
    parameter integer DS_OUT_WIDTH     = 8,   // 降采样量化后的输出位宽
    parameter         DS_COEFF_FILE    = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_dw0.memh",
    parameter         DS_BIAS_FILE     = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw0_Fold_bias.hex",
    parameter         DS_MULT_CNT      = 1,
    parameter         DS_MULT_FACTOR0  = 12'sd915,

    // ==========================================
    // 阶段 2：乒乓流水线 (Ping-Pong Module - PP)
    // 默认处理 25x5 特征图
    // ==========================================
    parameter integer PP_ROUNDS        = 3,   // 乒乓轮数(不包括第一层)
    parameter integer PP_PIXEL_DEPTH   = 125, // 25 x 5 降采样后的面积
    parameter integer PP_IN_FRAME_SIZE = 64,  // 承接前级的 64 个通道
    parameter integer PP_DW_K_H        = 3,
    parameter integer PP_DW_K_W        = 3,
    parameter integer PP_OUT_PIXEL_W   = 8,   // 最终网络输出位宽
    parameter         PP_DW_COEFF_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh",
    parameter         PP_PW_COEFF_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_pw.memh",
    parameter         PP_DW_BIAS_FILE  = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw_pingpong_bias.hex",
    parameter         PP_PW_BIAS_FILE  = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_pw_pingpong_bias.hex"
)(
    input  wire                               clk,
    input  wire                               rst_n,

    // ==========================================
    // 顶层网络控制与状态
    // ==========================================
    input  wire                               start,   // 脉冲启动信号，唤醒后端乒乓状态机
    output wire                               busy,    // 网络忙碌状态

    // ==========================================
    // 外部原始输入流
    // ==========================================
    input  wire                               in_valid,
    output wire                               in_ready,
    input  wire signed [DS_DATA_W-1:0]        in_pixel,

    // ==========================================
    // 最终网络输出流
    // ==========================================
    output wire                               out_valid,
    input  wire                               out_ready,
    output wire        [PP_OUT_PIXEL_W-1:0]   out_data,
    output wire                               out_end_frame,
    output wire                               out_end_all_frame
);

    // ==========================================
    // 状态机控制
    // ==========================================
    localparam  IN_PIXEL_MAX = DS_COL * DS_ROW * DS_COEFF_GRP_NUM; // 490
    localparam  IN_PIXEL_W = $clog2(IN_PIXEL_MAX); // 9 位地址宽度足够寻址 490 个输入像素
    wire in_fire = in_valid && in_ready; // 输入数据有效且被接受的时钟周期
    wire out_fire = out_valid && out_ready; // 输出数据有效且被接受的时钟周期
    reg start_flag , input_processing; // 内部处理状态标志
    reg [IN_PIXEL_W-1:0] in_pixel_cnt; // 输入像素计数器
    wire in_pixel_cnt_wrap = (in_pixel_cnt == IN_PIXEL_MAX - 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_flag <= 1'b0;
        end else if (start && !busy) begin
            start_flag <= 1'b1;
        end else begin
            start_flag <= 1'b0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_pixel_cnt <= {IN_PIXEL_W{1'b0}};
            input_processing <= 1'b0;
        end else if (start_flag) begin
            in_pixel_cnt <= {IN_PIXEL_W{1'b0}}; 
            input_processing <= 1'b1; // 启动后进入处理状态
        end else if (in_fire) begin
            if(in_pixel_cnt_wrap) begin
                in_pixel_cnt <= {IN_PIXEL_W{1'b0}}; // 输入像素计数器回绕
                input_processing <= 1'b0; // 继续保持处理状态
            end else begin
                in_pixel_cnt <= in_pixel_cnt + 1'b1; // 正常计数
            end
        end
    end   

    // ==========================================
    // 内部总线：连接 DS(降采样层) 与 PP(乒乓层)
    // ==========================================
    wire                      ds_in_valid;
    wire                      ds_in_ready;
    wire signed [DS_DATA_W-1:0] ds_in_pixel;
    wire                      ds_in_end_all_frame;
    wire                      ds_out_valid;
    wire                      ds_out_ready;
    wire [DS_OUT_WIDTH-1:0]   ds_out_pixel;
    wire                      ds_out_end_frame;
    wire                      ds_out_end_all_frame;

    assign in_ready = input_processing ? ds_in_ready : 1'b0; // 处理状态时准备接受输入
    assign ds_in_valid = input_processing ? in_valid : 1'b0; // 处理状态时传递输入有效信号
    assign ds_in_pixel = in_pixel; // 输入像素直接传递给降采样层
    assign ds_in_end_all_frame = in_pixel_cnt_wrap;
    // ==========================================
    // 第一级：降采样 Depthwise 卷积引擎
    // ==========================================
    depthwiseConv2D #(
        .DATA_W          (DS_DATA_W),
        .COEFF_W         (DS_COEFF_W),
        .K_H             (DS_K_H),
        .K_W             (DS_K_W),
        .COL             (DS_COL),
        .ROW             (DS_ROW),
        .STRIDE          (DS_STRIDE),
        .PAD_TOP         (DS_PAD_TOP),
        .PAD_BOTTOM      (DS_PAD_BOTTOM),
        .PAD_LEFT        (DS_PAD_LEFT),
        .PAD_RIGHT       (DS_PAD_RIGHT),
        .COEFF_GRP_NUM   (DS_COEFF_GRP_NUM),
        .FRAME_GRP_NUM   (DS_COEFF_GRP_NUM), 
        .OUT_WIDTH       (DS_OUT_WIDTH),
        .COEFF_INIT_FILE (DS_COEFF_FILE),
        .BIAS_INIT_FILE  (DS_BIAS_FILE),
        .MULT_CNT        (DS_MULT_CNT),
        .MULT_FACTOR0    (DS_MULT_FACTOR0)
        // (如内部有其他固定的量化因子 MULT_FACTOR 等参数，可在此补充映射)
    ) u_layer0_downsample (
        .clk                 (clk),
        .rst_n               (rst_n),
        
        // 接收外部输入
        .in_valid            (ds_in_valid),
        .in_ready            (ds_in_ready),
        .in_pixel            (ds_in_pixel),
        .in_end_all_frame    (ds_in_end_all_frame),
        
        // 吐出给后级乒乓
        .out_pixel           (ds_out_pixel),
        .out_valid           (ds_out_valid),
        .out_ready           (ds_out_ready),
        .out_end_frame       (ds_out_end_frame),
        .out_end_all_frame   (ds_out_end_all_frame)
    );

    // ==========================================
    // 第二级：多轮乒乓引擎 (DW + PW)
    // ==========================================
    dw_pw_pingpong #(
        .PINGPONG_ROUNDS     (PP_ROUNDS),
        .PIXEL_DEPTH         (PP_PIXEL_DEPTH),
        .IN_FRAME_SIZE       (PP_IN_FRAME_SIZE),
        .IN_DATA_W           (DS_OUT_WIDTH), // 完美承接上一级的量化位宽
        .DW_K_H              (PP_DW_K_H),
        .DW_K_W              (PP_DW_K_W),
        .OUT_PIXEL_WIDTH     (PP_OUT_PIXEL_W),
        .DW_COEFF_INIT_FILE  (PP_DW_COEFF_FILE),
        .PW_COEFF_INIT_FILE  (PP_PW_COEFF_FILE),
        .DW_BIAS_INIT_FILE   (PP_DW_BIAS_FILE),
        .PW_BIAS_INIT_FILE   (PP_PW_BIAS_FILE)
        // (注：视情况可映射更多 DW_COL=5, DW_ROW=25 等参数)
    ) u_layer1_to_N_pingpong (
        .clk                 (clk),
        .rst_n               (rst_n),

        // 顶层状态机控制
        .start               (start),
        .busy                (busy),

        // 接收前级的中间流
        .in_valid            (ds_out_valid),
        .in_ready            (ds_out_ready),
        .in_pixel            (ds_out_pixel),
        .in_end_all_frame    (ds_out_end_all_frame),

        // 顶层输出网络结果
        .out_valid           (out_valid),
        .out_ready           (out_ready),
        .out_data            (out_data),
        .end_frame           (out_end_frame),
        .end_all_frame       (out_end_all_frame)
    );

    //test
    reg[31:0] dowmsample_out_count;
    reg[31:0] in_pixel_cnt_test;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dowmsample_out_count <= 0;
            in_pixel_cnt_test <= 0;
        end else begin
            // if(ds_out_valid&&ds_out_ready) begin
            //     $display("beat %d: DownPixel=%h, EndFrame=%b, EndAllFrame=%b", dowmsample_out_count, ds_out_pixel, ds_out_end_frame, ds_out_end_all_frame);
            //     dowmsample_out_count <= dowmsample_out_count + 1;
            // end
            // if(in_valid&&in_ready) begin
            //     $display("beat %d: InPixel=%h, EndAllFrame=%b", in_pixel_cnt_test, in_pixel, in_end_all_frame);
            //     in_pixel_cnt_test <= in_pixel_cnt_test + 1;
            // end
        end
    end


endmodule