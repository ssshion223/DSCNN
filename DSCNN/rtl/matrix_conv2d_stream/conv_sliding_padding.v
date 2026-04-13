//==============================================================================
// 模块名：conv_sliding_padding
// 功能说明：
//   将输入像素流构造成 K_HxK_W 滑动窗口，支持上下左右独立补零与步长配置。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_ready/in_pixel：源像素流握手与数据
//   out_ready：下游窗口流接收就绪
// 输出接口：
//   out_valid/window_bus：打包后的 K_H*K_W 窗口数据
// 时序特性：
//   当 pipe_fire 条件满足时推进一个采样。
//   当在有效区时，等待 in_valid（且下层已经准备好接收数据）；
//   当在 padding 区时，自动注入 0 并流转（此时in_ready 为0）。
//==============================================================================
module conv_sliding_padding #(
    parameter integer DATA_W = 8,           // 输入像素位宽
    parameter integer COL    = 64,          // 输入列数
    parameter integer ROW    = 64,          // 输入行数
    parameter integer K_H    = 3,           // 卷积核高度
    parameter integer K_W    = 3,           // 卷积核宽度
    parameter integer STRIDE = 1,           // 窗口步长
    parameter integer PAD_TOP    = K_H/2,   // 上补零
    parameter integer PAD_BOTTOM = K_H/2,   // 下补零
    parameter integer PAD_LEFT   = K_W/2,   // 左补零
    parameter integer PAD_RIGHT  = K_W/2    // 右补零
)(
    input  wire                         clk,          // 时钟信号
    input  wire                         rst_n,        // 低有效复位
    
    input  wire                         in_valid,     // 输入像素有效
    output wire                         in_ready,     // 输入像素就绪
    input  wire signed [DATA_W-1:0]     in_pixel,      // 输入像素
    
    output reg                          out_valid,    // 窗口输出有效
    input  wire                         out_ready,    // 窗口输出就绪
    output wire signed [K_H*K_W*DATA_W-1:0] window_bus // 输出窗口打包总线
);

    localparam integer WIN_SIZE      = K_H * K_W;
    
    // [修改] 计算包含上下左右独立 Padding 后的虚拟总宽和总高
    localparam integer P_COL         = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW         = ROW + PAD_TOP  + PAD_BOTTOM;
    
    localparam integer P_COL_W       = (P_COL <= 1) ? 1 : $clog2(P_COL);
    localparam integer P_ROW_W       = (P_ROW <= 1) ? 1 : $clog2(P_ROW);
    localparam integer STRIDE_W      = (STRIDE <= 1) ? 1 : $clog2(STRIDE);
    
    localparam integer LINEBUF_BANKS = (K_H <= 1) ? 1 : (K_H - 1);
    localparam integer LINEBUF_BUS_W = DATA_W * LINEBUF_BANKS;

    reg signed [DATA_W-1:0] window [0:WIN_SIZE-1];
    
    // [修改] 坐标计数器现在基于扩充后的 P_COL 和 P_ROW
    reg [P_COL_W-1:0] pad_col_idx;
    reg [P_ROW_W-1:0] pad_row_idx;

    wire signed [LINEBUF_BUS_W-1:0] linebuf_q_bus;
    wire signed [LINEBUF_BUS_W-1:0] linebuf_din_bus;

    wire pad_col_wrap;
    wire pad_row_wrap;
    
    wire is_active;     // 当前坐标是否在有效像素区域
    wire pipe_ready;    // 下游是否准备好接收（或当前不需要输出）
    wire pipe_fire;     // 内部流水线是否可以流动
    wire signed [DATA_W-1:0] pipe_din; // 送入流水线的实际数据

    integer r;
    integer c;
    genvar gw;
    genvar gb;

    assign pad_col_wrap = (pad_col_idx == P_COL - 1);
    assign pad_row_wrap = (pad_row_idx == P_ROW - 1);

    // [核心逻辑] 判断当前扫描的坐标是否落在真实图像区域内
    wire is_active_col = (pad_col_idx >= PAD_LEFT) && (pad_col_idx < COL + PAD_LEFT);
    wire is_active_row = (pad_row_idx >= PAD_TOP)  && (pad_row_idx < ROW + PAD_TOP);
    assign is_active = is_active_col && is_active_row;



    // 下游就绪逻辑
    assign pipe_ready = !out_valid || out_ready;
    
    // 如果在有效区，需等待 valid；如果在 padding 区，自行流转（自动注入0）
    assign pipe_fire  = pipe_ready && (is_active ? in_valid : 1'b1);

    // 只有在有效区且流水线可流动时，才对上游表示 Ready
    assign in_ready = is_active && pipe_ready;
    
    // 注入数据：有效区取外部像素，Padding 区强制填 0
    assign pipe_din = is_active ? in_pixel : {DATA_W{1'b0}};

    generate
        for (gw = 0; gw < WIN_SIZE; gw = gw + 1) begin : gen_window_pack
            assign window_bus[gw*DATA_W +: DATA_W] = window[gw];
        end
    endgenerate

    generate
        if (K_H > 1) begin : gen_linebuf
            for (gb = 0; gb < (K_H-1); gb = gb + 1) begin : gen_bank
                localparam integer LB_POS = (K_H - 2 - gb);

                if (gb == 0) begin : gen_din0
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] = pipe_din; // 使用内部带 padding 的数据
                end else begin : gen_dinx
                    localparam integer PREV_POS = (K_H - 1 - gb);
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] =
                        linebuf_q_bus[PREV_POS*DATA_W +: DATA_W];
                end

                linebuf_bank_ram #(
                    .DATA_W (DATA_W),
                    .DEPTH  (P_COL),   // [修改] 深度调整为带 padding 的列宽
                    .ADDR_W (P_COL_W)
                ) u_linebuf_bank_ram (
                    .clk  (clk),
                    .we   (pipe_fire), // 使用内部 pipeline 触发信号
                    .addr (pad_col_idx),
                    .din  (linebuf_din_bus[LB_POS*DATA_W +: DATA_W]),
                    .dout (linebuf_q_bus[LB_POS*DATA_W +: DATA_W])
                );
            end
        end else begin : gen_no_linebuf
            assign linebuf_q_bus   = {LINEBUF_BUS_W{1'b0}};
            assign linebuf_din_bus = {LINEBUF_BUS_W{1'b0}};
        end
    endgenerate

    reg [STRIDE_W-1:0] stride_col_cnt;
    reg [STRIDE_W-1:0] stride_row_cnt;

    // 坐标计数更新
    always @(posedge clk) begin
        if (!rst_n) begin
            pad_col_idx <= {P_COL_W{1'b0}};
            pad_row_idx <= {P_ROW_W{1'b0}};
        end else if (pipe_fire) begin
            if (pad_col_wrap) begin
                pad_col_idx <= {P_COL_W{1'b0}};
                pad_row_idx <= (pad_row_wrap) ? {P_ROW_W{1'b0}} : (pad_row_idx + 1'b1);
            end else begin
                pad_col_idx <= pad_col_idx + 1'b1;
            end
        end
    end

    // Stride 计数更新
    always @(posedge clk) begin
        if (!rst_n) begin
            stride_col_cnt <= {STRIDE_W{1'b0}};
            stride_row_cnt <= {STRIDE_W{1'b0}};
        end else if (pipe_fire) begin
            if (pad_col_wrap) begin
                stride_col_cnt <= {STRIDE_W{1'b0}};
                if (pad_row_wrap) begin
                    stride_row_cnt <= {STRIDE_W{1'b0}};
                end else if (pad_row_idx >= K_H-1) begin
                    stride_row_cnt <= (stride_row_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_row_cnt + 1;
                end
            end else if (pad_col_idx >= K_W-1) begin
                stride_col_cnt <= (stride_col_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_col_cnt + 1;
            end
        end
    end

    // 窗口移位寄存器更新
    always @(posedge clk) begin
        if (pipe_fire) begin
            for (r = 0; r < K_H; r = r + 1) begin
                for (c = 0; c < K_W-1; c = c + 1) begin
                    window[r*K_W + c] <= window[r*K_W + c + 1];
                end
            end
            for (r = 0; r < K_H - 1; r = r + 1) begin
                window[r*K_W + (K_W-1)] <= linebuf_q_bus[r*DATA_W +: DATA_W];
            end
            window[(K_H-1)*K_W + (K_W-1)] <= pipe_din; // 使用带 padding 的数据
        end
    end

    wire is_output_col = (pad_col_idx >= (K_W-1)) && (stride_col_cnt == 0);
    wire is_output_row = (pad_row_idx >= (K_H-1)) && (stride_row_cnt == 0);
    // Output Valid 逻辑
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
        end else if (out_valid && !out_ready) begin
            out_valid <= 1'b1;
        end else if (pipe_fire) begin
            out_valid <= is_output_col && is_output_row; // 只有在满足输出条件时才有效
        end else begin
            out_valid <= 1'b0;
        end
    end

endmodule