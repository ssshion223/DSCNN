//==============================================================================
// 模块名：conv_sliding_padding (终极优化版：无阻塞流水 + 零算术逻辑掩码)
// 功能说明：
//   将输入像素流构造成 K_H x K_W 滑动窗口，支持上下左右独立逻辑补零与步长配置。
//   1. 完美无阻塞流水线，输入端在图像接收期间 in_ready 恒为 1。
//   2. 基于 Valid Shift Register 实现完美的帧级气泡追踪与外部 Flush。
//   3. 极简 Padding MUX 架构，0 减法器，0 加法器，将逻辑复杂度降至最低。
//==============================================================================
module conv_sliding_padding #(
    parameter integer DATA_W     = 8,       // 输入像素位宽
    parameter integer COL        = 64,      // 输入原始图像列数
    parameter integer ROW        = 64,      // 输入原始图像行数
    parameter integer K_H        = 3,       // 卷积核高度
    parameter integer K_W        = 3,       // 卷积核宽度
    parameter integer STRIDE     = 1,       // 窗口步长 (目前 X/Y 步长保持一致，可按需拆分)
    
    // 默认提供 SAME Padding 计算，支持外部传参覆盖
    parameter integer PAD_TOP    = (K_H - 1) / 2,
    parameter integer PAD_BOTTOM = K_H / 2,
    parameter integer PAD_LEFT   = (K_W - 1) / 2,
    parameter integer PAD_RIGHT  = K_W / 2
)(
    input  wire                         clk,
    input  wire                         rst_n,
    
    // --- 握手与输入接口 ---
    input  wire                         in_valid,
    output wire                         in_ready,
    input  wire signed [DATA_W-1:0]     in_pixel,
    input  wire                         end_all_frame, // 触发排空流水线尾部数据
    
    // --- 握手与输出接口 ---
    output wire                          out_valid,
    input  wire                         out_ready,
    output wire signed [K_H*K_W*DATA_W-1:0] out_window_bus
);

    // 内部常量计算
    localparam integer WIN_SIZE      = K_H * K_W;
    localparam integer COL_W         = (COL <= 1) ? 1 : $clog2(COL);
    localparam integer ROW_W         = (ROW <= 1) ? 1 : $clog2(ROW);
    localparam integer STRIDE_W      = (STRIDE <= 1) ? 1 : $clog2(STRIDE);
    
    localparam integer LINEBUF_BANKS = (K_H <= 1) ? 1 : (K_H - 1);
    localparam integer LINEBUF_BUS_W = DATA_W * LINEBUF_BANKS;


    // 全填充
    localparam integer FULL_PAD_LEFT   = (K_W - 1) / 2;
    localparam integer FULL_PAD_RIGHT  = K_W / 2;
    localparam integer FULL_PAD_TOP    = (K_H - 1) / 2;
    localparam integer FULL_PAD_BOTTOM = K_H / 2;

    localparam integer OUT_WIN_LEFT = FULL_PAD_LEFT >= PAD_LEFT ? (FULL_PAD_LEFT - PAD_LEFT) : 0;
    localparam integer OUT_WIN_TOP  = FULL_PAD_TOP >= PAD_TOP ? (FULL_PAD_TOP - PAD_TOP) : 0;
    localparam integer OUT_WIN_RIGHT = (FULL_PAD_RIGHT >= PAD_RIGHT) ? (COL - 1 - (FULL_PAD_RIGHT - PAD_RIGHT)) : (COL - 1);
    localparam integer OUT_WIN_BOTTOM = (FULL_PAD_BOTTOM >= PAD_BOTTOM) ? (ROW - 1 - (FULL_PAD_BOTTOM - PAD_BOTTOM)) : (ROW - 1);

    // 物理延迟：最新输入像素走到窗口中心（即满足PAD_BOTTOM和PAD_RIGHT偏移）所需的移位周期
    localparam integer DELAY_CYCLES  = (FULL_PAD_RIGHT) + (FULL_PAD_BOTTOM) * COL + 1 ;

    // 二维展开的窗口寄存器阵列 (展开为一维方便循环)
    reg signed [DATA_W-1:0] window [0:WIN_SIZE-1];
    wire signed [K_H*K_W*DATA_W-1:0] window_bus;
    reg signed [K_H*K_W*DATA_W-1:0] window_bus_reg;

    wire fifo_almost_full; // FIFO 将满标志，用于上游流控


    //---------------------------------------------------------
    // 1. 流水线核心控制逻辑 (Valid 追踪与防死锁 Flush)
    //---------------------------------------------------------
    wire pipe_ready = !fifo_almost_full; // 流水线就绪条件：下游 FIFO 没有将满
    wire center_valid; // 窗口中心是否存在真实像素
    wire pipe_busy;    // 流水线中是否还有未吐出的真实像素
    reg is_flushing;
    wire pipe_data_valid = in_valid || is_flushing; // 当前输入数据是否有效（包括正常输入和Flush注入的气泡）
    // 流水线流动条件：下游就绪，且 (有新输入 或者 正在强制排空)
    wire pipe_fire = pipe_ready && pipe_data_valid;
    // 对上游 ready：下游就绪，且没在进行排空操作
    assign in_ready = pipe_ready && !is_flushing;
    // 写入流水线的数据（Flush 注入零气泡）
    wire signed [DATA_W-1:0] pipe_din = (is_flushing) ? {DATA_W{1'b0}} : in_pixel;
    generate
        if (DELAY_CYCLES > 1) begin : gen_valid_tracker
            // 精确追踪有效数据的物理位置
            reg [DELAY_CYCLES-1:0] valid_sr;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) 
                    valid_sr <= {DELAY_CYCLES{1'b0}};
                else if (pipe_fire) 
                    valid_sr <= {valid_sr[DELAY_CYCLES-2:0], !is_flushing}; 
            end
            assign center_valid = valid_sr[DELAY_CYCLES-1];
            assign pipe_busy    = (valid_sr != {DELAY_CYCLES{1'b0}});
        end else begin : gen_one_delay
            reg valid_sr;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) 
                    valid_sr <= 1'b0;
                else if (pipe_fire) 
                    valid_sr <= !is_flushing;
            end
            assign center_valid = valid_sr;
            assign pipe_busy    = valid_sr;
        end 
    endgenerate

    // Flush 状态机 (防死锁设计)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            is_flushing <= 1'b0;
        end else begin
            // 外部给结束信号，且流水线还没排空时，进入Flush推力模式
            if (end_all_frame && (pipe_busy || in_valid)) begin
                is_flushing <= 1'b1;
            end else if (!pipe_busy) begin
                // 流水线干净了，自动退出Flush
                is_flushing <= 1'b0;
            end
        end
    end


    //---------------------------------------------------------
    // 2. Line Buffer 实例化 (深度恒定为物理图宽 COL)
    //---------------------------------------------------------
    wire signed [LINEBUF_BUS_W-1:0] linebuf_q_bus;
    wire signed [LINEBUF_BUS_W-1:0] linebuf_din_bus;
    
    // 循环读写地址计数器
    reg [COL_W-1:0] lb_addr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            lb_addr <= {COL_W{1'b0}};
        else if (pipe_fire) 
            lb_addr <= (lb_addr == COL - 1) ? {COL_W{1'b0}} : lb_addr + 1'b1;
    end

    genvar gb;
    generate
        if (K_H > 1) begin : gen_linebuf
            for (gb = 0; gb < (K_H-1); gb = gb + 1) begin : gen_bank
                localparam integer LB_POS = (K_H - 2 - gb);
                
                if (gb == 0) begin : gen_din0
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] = pipe_din;
                end else begin : gen_dinx
                    localparam integer PREV_POS = (K_H - 1 - gb);
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] = linebuf_q_bus[PREV_POS*DATA_W +: DATA_W];
                end

                linebuf_bank_ram #(
                    .DATA_W (DATA_W),
                    .DEPTH  (COL),     
                    .ADDR_W (COL_W)
                ) u_linebuf_bank_ram (
                    .clk  (clk),
                    .we   (pipe_fire),
                    .addr (lb_addr),
                    .din  (linebuf_din_bus[LB_POS*DATA_W +: DATA_W]),
                    .dout (linebuf_q_bus[LB_POS*DATA_W +: DATA_W])
                );
            end
        end else begin : gen_no_linebuf
            assign linebuf_q_bus   = {LINEBUF_BUS_W{1'b0}};
            assign linebuf_din_bus = {LINEBUF_BUS_W{1'b0}};
        end
    endgenerate

    //---------------------------------------------------------
    // 3. 移位寄存器阵列更新 (K_H x K_W Window)
    //---------------------------------------------------------
    integer r_idx, c_idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize the window to zero
            for (r_idx = 0; r_idx < K_H; r_idx = r_idx + 1) begin
                for (c_idx = 0; c_idx < K_W; c_idx = c_idx + 1) begin
                    window[r_idx*K_W + c_idx] <= {DATA_W{1'b0}};
                end
            end
        end else if (pipe_fire) begin
            // 每一行内部向左移位
            for (r_idx = 0; r_idx < K_H; r_idx = r_idx + 1) begin
                for (c_idx = 0; c_idx < K_W-1; c_idx = c_idx + 1) begin
                    window[r_idx*K_W + c_idx] <= window[r_idx*K_W + c_idx + 1];
                end
            end
            // 每一行的最右侧接收上一行的缓存输出 (除了最底层接收最新输入)
            for (r_idx = 0; r_idx < K_H - 1; r_idx = r_idx + 1) begin
                window[r_idx*K_W + (K_W-1)] <= linebuf_q_bus[r_idx*DATA_W +: DATA_W];
            end
            window[(K_H-1)*K_W + (K_W-1)] <= pipe_din;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_bus_reg <= {K_H*K_W*DATA_W{1'b0}};
        end else if (pipe_fire) begin
            window_bus_reg <= window_bus;
        end
    end

    //---------------------------------------------------------
    // 4. 中心坐标追踪与零算法逻辑掩码 
    //---------------------------------------------------------
    reg [COL_W-1:0] center_col;
    reg [ROW_W-1:0] center_row;
    wire center_col_wrap = (center_col == COL - 1);
    wire center_row_wrap = (center_row == ROW - 1);
    wire center_fire = center_valid && pipe_fire;
    // 更新物理图像坐标系下的中心点
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            center_col <= {COL_W{1'b0}};
            center_row <= {ROW_W{1'b0}};
        end else if (center_fire) begin
            if (center_col_wrap) begin
                center_col <= {COL_W{1'b0}};
                if (center_row_wrap) begin
                    center_row <= {ROW_W{1'b0}};
                end else begin
                    center_row <= center_row + 1'b1;
                end
            end else begin
                center_col <= center_col + 1'b1;
            end
        end
    end

    wire row_valid [0:K_H-1];
    wire col_valid [0:K_W-1];
    
    genvar r, c;
    generate
        for (r = 0; r < K_H; r = r + 1) begin : gen_row_mask
            localparam signed [31:0] MIN_Y = FULL_PAD_TOP - r;
            localparam signed [31:0] MAX_Y = ROW + FULL_PAD_TOP - r;
            assign row_valid[r] = ($signed({1'b0, center_row}) >= MIN_Y) && 
                                  ($signed({1'b0, center_row}) < MAX_Y);
        end
        for (c = 0; c < K_W; c = c + 1) begin : gen_col_mask
            localparam signed [31:0] MIN_X = FULL_PAD_LEFT - c;
            localparam signed [31:0] MAX_X = COL + FULL_PAD_LEFT - c;
            assign col_valid[c] = ($signed({1'b0, center_col}) >= MIN_X) && 
                                  ($signed({1'b0, center_col}) < MAX_X);
        end
        for (r = 0; r < K_H; r = r + 1) begin : gen_pad_r
            for (c = 0; c < K_W; c = c + 1) begin : gen_pad_c
                wire pixel_valid = row_valid[r] && col_valid[c];
                wire signed [DATA_W-1:0] masked_data = pixel_valid ? window[r*K_W + c] : {DATA_W{1'b0}};
                assign window_bus[(r*K_W + c)*DATA_W +: DATA_W] = masked_data;
            end
        end
    endgenerate

    //---------------------------------------------------------
    // 5. 步长与有效信号控制
    //---------------------------------------------------------
    wire is_output_stride;
    wire out_col_valid = (center_col >= OUT_WIN_LEFT) && (center_col <= OUT_WIN_RIGHT);
    wire out_row_valid = (center_row >= OUT_WIN_TOP) && (center_row <= OUT_WIN_BOTTOM);
    generate
        if(STRIDE > 1) begin
            reg [STRIDE_W-1:0] stride_col_cnt;
            reg [STRIDE_W-1:0] stride_row_cnt;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    stride_col_cnt <= {STRIDE_W{1'b0}};
                    stride_row_cnt <= {STRIDE_W{1'b0}};
                end else if (center_fire) begin
                    if (center_col_wrap) begin
                        stride_col_cnt <= {STRIDE_W{1'b0}}; // 换行重置列步长
                        if (center_row_wrap) begin
                            stride_row_cnt <= {STRIDE_W{1'b0}}; // 换帧重置行步长
                        end else if(out_row_valid) begin
                            // 行步长仅在换行时累加
                            stride_row_cnt <= (stride_row_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_row_cnt + 1'b1;
                        end
                    end else if(out_col_valid) begin
                        // 列步长按像素累加
                        stride_col_cnt <= (stride_col_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_col_cnt + 1'b1;
                    end
                end
            end
            reg is_output_stride_reg;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    is_output_stride_reg <= 1'b0;
                end else begin
                    is_output_stride_reg <= (stride_col_cnt == {STRIDE_W{1'b0}}) && (stride_row_cnt == {STRIDE_W{1'b0}});
                end
            end
            assign is_output_stride = is_output_stride_reg;
        end else begin
            assign is_output_stride = 1'b1; // 步长为1时始终输出
        end
    endgenerate
    
    reg out_fire;
    generate
        if(FULL_PAD_LEFT == PAD_LEFT && FULL_PAD_RIGHT == PAD_RIGHT && FULL_PAD_TOP == PAD_TOP && FULL_PAD_BOTTOM == PAD_BOTTOM) begin
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    out_fire <= 1'b0;
                end else begin
                    out_fire <= center_fire;
                end
            end
        end else begin
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    out_fire <= 1'b0;
                end else begin
                    out_fire <= center_fire && out_col_valid && out_row_valid;
                end
            end
        end
    endgenerate 

    wire fifo_in_valid = out_fire && is_output_stride ;

    fwft_fifo_reg #(
        .WIDTH(K_H*K_W*DATA_W),
        .DEPTH(4), // 深度不需要太大，主要是为了缓冲输出时序
        .AF_LEVEL(3)
    ) u_output_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .din(window_bus_reg),
        .wr_en(fifo_in_valid),
        .full(), // 不关心满标志，因为上游 always 块会自然停掉 valid
        .almost_full(fifo_almost_full), // 不关心 almost_full，因为上游 always 块会自然停掉 valid
        .rd_en(out_ready && out_valid), // 仅在下游准备好且当前有效时确认读出
        .dout(out_window_bus),
        .valid(out_valid) 
    );

endmodule

