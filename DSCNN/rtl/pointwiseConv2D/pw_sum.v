`timescale 1ns / 1ps

//==============================================================================
// 模块名：pw_sum
//==============================================================================

module pw_sum #(
    parameter MULT_WIDTH    = 16,  // 前级输入的乘积结果位宽
    parameter CHANNELS      = 64,  // 总并行输出通道数
    parameter SEGMENTS      = 4,   // 分块数 (例如 4块，每块处理16个通道)
    parameter PIXEL_DEPTH = 125,  // 空间像素数 (25x5)
    parameter RAM_DATA_W    = 22*(CHANNELS/SEGMENTS),  // RAM 中存储的部分和累加位宽
    parameter RAM_ADDR_W = $clog2(PIXEL_DEPTH * SEGMENTS), // RAM 地址位宽
    parameter IN_FRAME_SIZE= 64
)(
    input  wire clk,
    input  wire rst_n,

    // 1. 与前级交互的输入接口
    input  wire                                         in_valid, // 前级数据有效标志
    output wire                                         in_ready, // 本模块就绪，可接收新数据
    input  wire [CHANNELS*MULT_WIDTH-1:0]               mult_in,  // 前级算好的 64 通道乘积结果

    // 2. 与窄带 Psum RAM 交互的接口
    output wire                                         ram_re,
    output wire [RAM_ADDR_W-1:0]                        ram_raddr,
    input  wire [RAM_DATA_W-1:0]                        ram_rdata,

    output wire                                         ram_we,
    output wire [RAM_ADDR_W-1:0]                        ram_waddr,
    output wire [RAM_DATA_W-1:0]                        ram_wdata,

    output wire                                         end_frame, // 当前帧结束标志
    output wire                                         end_all_frame // 全部帧结束标志
);
    localparam SEG_CHANS = CHANNELS / SEGMENTS;
    localparam RAM_ONE_DATA_W = (SEG_CHANS == 0) ? 1 : (RAM_DATA_W / SEG_CHANS); // 每个 segment 内每通道对应的 RAM 数据位宽
    localparam SEG_BITS  = (SEGMENTS > 1) ? $clog2(SEGMENTS) : 1; // 分段计数器位宽
    localparam PIXEL_W = (PIXEL_DEPTH > 1) ? $clog2(PIXEL_DEPTH) : 1; // 像素计数器位宽
    localparam FRAME_W = (IN_FRAME_SIZE > 1) ? $clog2(IN_FRAME_SIZE) : 1; // 帧计数器位宽
    //=========================================================
    // 1. 分段控制状态机
    //=========================================================
    reg [SEG_BITS-1:0]  seg_cnt; // 当前正在处理的段号 (0 ~ SEGMENTS-1)
    reg [PIXEL_W-1:0]   pixel_cnt; // 当前正在处理的像素索引 (0 ~ PIXEL_DEPTH-1)
    reg                 processing;
    reg [FRAME_W-1:0]   frame_cnt;
    reg [CHANNELS*MULT_WIDTH-1:0] mult_in_reg; // 锁存输入乘积数据

    wire in_fire = in_valid && in_ready;
    wire seg_cnt_wrap = (seg_cnt == SEGMENTS-1);
    assign in_ready = !processing || (seg_cnt_wrap);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seg_cnt <= {SEG_BITS{1'b0}};
        end else if (in_fire) begin
            seg_cnt <= {SEG_BITS{1'b0}}; // 每次新输入时段计数器重置
        end else if (processing) begin
            seg_cnt <= (seg_cnt_wrap) ? {SEG_BITS{1'b0}} : (seg_cnt + 1'b1); // 每段处理完后递增
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            processing <= 1'b0;
            mult_in_reg <= {(CHANNELS*MULT_WIDTH){1'b0}};
        end else if (in_fire) begin
            processing <= 1'b1; // 开始处理新输入
            mult_in_reg <= mult_in; // 锁存输入乘积数据
        end else if (processing && seg_cnt_wrap) begin
            processing <= 1'b0; // 全部帧处理完毕，回到空闲状态
        end
    end


    wire pixel_wrap = (pixel_cnt == PIXEL_DEPTH-1);
    wire frame_wrap = (frame_cnt == IN_FRAME_SIZE-1);
    reg end_frame_reg, end_all_frame_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_cnt <= {PIXEL_W{1'b0}};
            frame_cnt <= {FRAME_W{1'b0}};
            end_frame_reg <= 1'b0;
            end_all_frame_reg <= 1'b0;
        end else if (processing && seg_cnt_wrap) begin
            if (pixel_wrap) begin
                pixel_cnt <= {PIXEL_W{1'b0}};
                end_frame_reg <= 1'b1; // 当前帧结束
                if (frame_wrap) begin
                    frame_cnt <= {FRAME_W{1'b0}};
                    end_all_frame_reg <= 1'b1; // 全部帧结束
                end else begin
                    frame_cnt <= frame_cnt + 1'b1;
                    end_all_frame_reg <= 1'b0;
                end
            end else begin
                pixel_cnt <= pixel_cnt + 1'b1;
                end_frame_reg <= 1'b0;
                end_all_frame_reg <= 1'b0;
            end
        end else begin
            end_frame_reg <= 1'b0;
            end_all_frame_reg <= 1'b0;
        end
    end


    //=========================================================
    // 2. 生成 RAM 读地址并读取切片数据
    //=========================================================
    reg [RAM_ADDR_W-1:0]         ram_raddr_reg;
    wire first_frame =(frame_cnt == {FRAME_W{1'b0}});
    wire [RAM_DATA_W-1:0]        ram_data = (first_frame) ? {RAM_DATA_W{1'b0}} : ram_rdata; // 第一帧时 RAM 输出全0

    wire rd_ram_en = in_fire || (processing && !seg_cnt_wrap); // 读使能：当有新输入或正在处理且未完成当前段时
    wire raddr_wrap = (ram_raddr_reg == PIXEL_DEPTH*SEGMENTS-1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_raddr_reg <= {RAM_ADDR_W{1'b0}};
        end else if (rd_ram_en) begin
            ram_raddr_reg <= (raddr_wrap) ? {RAM_ADDR_W{1'b0}} : (ram_raddr_reg + 1'b1); // 每段处理完后地址递增
        end
    end

    //=========================================================
    // 3. 分段累加逻辑：将输入乘积与 RAM 中的部分和相加
    //=========================================================
    wire [SEG_CHANS*RAM_ONE_DATA_W-1:0]  add_result; 
    wire [SEG_CHANS*MULT_WIDTH-1:0] mult_in_reg_slice;

    assign mult_in_reg_slice = mult_in_reg[(seg_cnt*SEG_CHANS*MULT_WIDTH) +: (SEG_CHANS*MULT_WIDTH)]; // 当前段的乘积输入切片
    genvar i;
    generate
        for (i = 0; i < SEG_CHANS; i = i + 1) begin : ADD_LOOP
            wire [MULT_WIDTH-1:0] mult_slice = mult_in_reg_slice[(i+1)*MULT_WIDTH-1 -: MULT_WIDTH];
            wire [RAM_ONE_DATA_W-1:0] psum_slice = ram_data[(i+1)*RAM_ONE_DATA_W-1 -: RAM_ONE_DATA_W];
            assign add_result[(i+1)*RAM_ONE_DATA_W-1 -: RAM_ONE_DATA_W] = $signed(mult_slice) + $signed(psum_slice); // 累加乘积和部分和
        end
    endgenerate
    
    //=========================================================
    // 4. 写回 RAM 的地址和数据准备
    //=========================================================
    reg [RAM_ADDR_W-1:0]  ram_waddr_reg;

    wire waddr_wrap = (ram_waddr_reg == PIXEL_DEPTH*SEGMENTS-1);
    wire wr_ram_en = processing ;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_waddr_reg <= {RAM_ADDR_W{1'b0}};
        end else if (processing) begin
            ram_waddr_reg <= (waddr_wrap) ? {RAM_ADDR_W{1'b0}} : (ram_waddr_reg + 1'b1); // 每段处理完后地址递增
        end
    end

    //=========================================================
    // 5. 端口信号最终赋值
    //========================================================= 


    assign ram_re = rd_ram_en; // 读使能：当有新输入或正在处理且未完成当前段时
    assign ram_raddr = ram_raddr_reg; // 读地址：当前段的像素索引
    assign ram_we = wr_ram_en; // 写使能：当正在处理时
    assign ram_waddr = ram_waddr_reg; // 写地址：当前段
    assign ram_wdata = add_result; // 写数据：累加结果

    assign end_frame = end_frame_reg; // 当前帧结束：当像素和段都处理完时
    assign end_all_frame = end_all_frame_reg; // 全部帧结束：当所有帧都处理完时


    //=========================================================
    // 6. 监视信号（仅用于仿真调试）
    //=========================================================
    reg [31:0] ram_read_cnt, ram_write_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_read_cnt <= 0;
            ram_write_cnt <= 0;
        end else begin
            // if (rd_ram_en) begin
            //     $display("beat %d: RAM Read - Addr: %d, Data: %h",ram_read_cnt, ram_raddr_reg, ram_data);
            //     ram_read_cnt <= ram_read_cnt + 1;
            // end
            // if (wr_ram_en) begin
            //     $display("beat %d: RAM Write - Addr: %d, Data: %h",ram_write_cnt, ram_waddr_reg, add_result);
            //     ram_write_cnt <= ram_write_cnt + 1;
            // end
        end
    end
    //=========================================================

endmodule