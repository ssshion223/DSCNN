//==============================================================================
// 模块名：bias_process_wrapper
// 功能说明：
//   封装 Bias 读取、Bias+量化+ReLU 处理以及输出缓冲。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   s_*：上游数据流（数据 + 帧标志）
//   m_ready：下游反压
// 输出接口：
//   m_*：后处理后的输出流
//   s_ready：基于输出 FIFO 占用的上游回压
// 时序特性：
//   流水线含 1 级对齐，并使用 FWFT FIFO 解耦上下游。
//==============================================================================
module bias_process_wrapper #(
    parameter IN_WIDTH      = 32, // 上游输入数据位宽
    parameter BIAS_WIDTH    = 32, // Bias 数据位宽
    parameter OUT_WIDTH     = 8,  // 输出数据位宽
    parameter SHIFT_VAL     = 24, // 量化右移位数
    parameter GROUP_BITS    = 4,  // 组编号位宽
    parameter GROUP_SIZE    = 1<<GROUP_BITS, // 组总数
    parameter CH_BITS       = 4,  // 通道编号位宽
    parameter FIFO_DEPTH    = 16, // 输出 FIFO 深度
    parameter FIFO_AF_LEVEL = 14, // FIFO 将满阈值
    parameter BIAS_INIT_FILE= "D:/vivado/exp/DSCNN/rtl/dw/DS-CNN_dw0_Fold_bias.hex" // Bias 初始化文件
)(
    input  wire        clk,   // 时钟信号
    input  wire        rst_n, // 低有效复位

    // ==========================================
    // 上游输入接口 (自适应位宽)
    // ==========================================
    input  wire signed [IN_WIDTH-1:0] s_pixel_data_bus, // 上游输入数据
    input  wire                       s_valid,         // 上游输入有效
    input  wire                       s_end_frame,     // 当前帧结束标志
    input  wire                       s_end_all_frame, // 全部组帧结束标志
    output wire                       s_ready,         // 上游回压就绪

    // ==========================================
    // 下游输出接口 (自适应位宽)
    // ==========================================
    output wire        [OUT_WIDTH-1:0] m_pixel_data,    // 下游输出数据
    output wire                        m_valid,         // 下游输出有效
    output wire                        m_end_frame,     // 当前帧结束标志
    output wire                        m_end_all_frame, // 全部组帧结束标志
    input  wire                        m_ready          // 下游接收就绪
);

    wire fifo_almost_full;
    wire fifo_full;
    assign s_ready = ~fifo_almost_full;
    wire handshake = s_valid && s_ready;

    // 1. 组号与通道号逻辑
    reg [GROUP_BITS-1:0] curr_group;
    reg [CH_BITS-1:0]    curr_channel;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            curr_group   <= {GROUP_BITS{1'b0}};
            curr_channel <= {CH_BITS{1'b0}};
        end else if (handshake) begin
            if (s_end_all_frame) begin
                if(curr_group == GROUP_SIZE - 1) begin
                    curr_group <= {GROUP_BITS{1'b0}};
                end else begin
                    curr_group <= curr_group + 1'b1;
                end
                curr_channel <= {CH_BITS{1'b0}};
            end else if (s_end_frame) begin
                curr_channel <= curr_channel + 1'b1;
            end
        end
    end

    // 2. 例化参数化的 Bias Memory
    wire signed [BIAS_WIDTH-1:0] current_bias;
    bias_memory #(
        .DATA_WIDTH (BIAS_WIDTH),  // 与 Bias 数据位宽对齐
        .GROUP_BITS (GROUP_BITS),
        .CH_BITS    (CH_BITS),
        .BIAS_INIT_FILE (BIAS_INIT_FILE)
    ) u_bias_mem (
        .clk        (clk),
        .read_en    (handshake),
        .group_id   (curr_group),
        .channel_id (curr_channel),
        .bias_out   (current_bias)
    );

    // 3. 数据流打一拍缓冲
    reg signed [IN_WIDTH-1:0] pipe_data;
    reg                       pipe_valid;
    reg                       pipe_end_frame;
    reg                       pipe_end_all_frame;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_valid         <= 1'b0;
            pipe_data          <= 0;
            pipe_end_frame     <= 1'b0;
            pipe_end_all_frame <= 1'b0;
        end else begin
            pipe_valid <= handshake; 
            if (handshake) begin
                pipe_data          <= s_pixel_data_bus;
                pipe_end_frame     <= s_end_frame;
                pipe_end_all_frame <= s_end_all_frame;
            end
        end
    end

    // 4. 例化参数化的 ALU 模块
    wire [OUT_WIDTH-1:0] alu_result;
    wire alu_valid;
    bias_quant_relu #(
        .IN_WIDTH  (IN_WIDTH),
        .BIAS_WIDTH (BIAS_WIDTH),
        .OUT_WIDTH (OUT_WIDTH),
        .SHIFT_VAL (SHIFT_VAL)
    ) u_alu (
        .clk      (clk),
        .rst_n    (rst_n),
        .wr_valid (pipe_valid),
        .data_in  (pipe_data),
        .bias     (current_bias),
        .data_out (alu_result),
        .out_valid (alu_valid)
    );

    // ==========================================
    // 5. 输出 FIFO (位宽自动计算)
    // ==========================================
    // 利用 localparam 自动计算 FIFO 需要存储的总位宽：数据位宽 + 2个标志位
    localparam FIFO_WIDTH = OUT_WIDTH + 2; 

    wire [FIFO_WIDTH-1:0] fifo_din = {pipe_end_all_frame, pipe_end_frame, alu_result};
    wire [FIFO_WIDTH-1:0] fifo_dout;

    fwft_fifo_reg #(
        .WIDTH    (FIFO_WIDTH),  // 动态赋值
        .DEPTH    (FIFO_DEPTH),
        .AF_LEVEL (FIFO_AF_LEVEL)
    ) u_output_buffer (
        .clk         (clk),
        .rst_n       (rst_n),
        .din         (fifo_din),
        .wr_en       (alu_valid), 
        .full        (fifo_full),
        .almost_full (fifo_almost_full),
        .rd_en       (m_ready),
        .dout        (fifo_dout),
        .valid       (m_valid),
        .empty       () 
    );

    // ==========================================
    // 6. 解包自动映射输出
    // ==========================================
    assign m_pixel_data    = fifo_dout[OUT_WIDTH-1 : 0];
    assign m_end_frame     = fifo_dout[OUT_WIDTH];     // 第 OUT_WIDTH 位
    assign m_end_all_frame = fifo_dout[OUT_WIDTH+1];   // 第 OUT_WIDTH+1 位

endmodule