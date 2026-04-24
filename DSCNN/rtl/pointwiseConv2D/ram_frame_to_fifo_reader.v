`timescale 1ns / 1ps


module ram_frame_to_fifo_reader #(
    parameter integer PIXELS_PER_FRAME  = 125,
    parameter integer SEGMENTS          = 4,
    parameter integer CHANNELS          = 64,
    parameter integer RAM_DATA_W        = 22 * (CHANNELS/SEGMENTS),
    parameter integer RAM_ADDR_W        = $clog2(PIXELS_PER_FRAME * SEGMENTS),
    parameter integer FIFO_DEPTH        = 16,
    parameter integer FIFO_AF_LEVEL     = 10,
    parameter integer OUT_W = RAM_DATA_W / (CHANNELS/SEGMENTS)

)(
    input  wire                         clk,
    input  wire                         rst_n,

    // 启动控制
    input  wire                         start,
    output wire                         busy,

    // RAM 读口（同步读，rdata 延后 1 拍）
    output wire                         ram_re,
    output wire [RAM_ADDR_W-1:0]        ram_raddr,
    input  wire [RAM_DATA_W-1:0]        ram_rdata,

    // FIFO 流输出
    output wire                         out_valid,
    input  wire                         out_ready,
    output wire [OUT_W-1:0]             out_data,

    output wire                         end_frame,
    output wire                         end_all_frame
);

    localparam integer RAM_DEPTH   = PIXELS_PER_FRAME * SEGMENTS;
    localparam integer SEG_CHANS    = CHANNELS / SEGMENTS;
    localparam integer CH_W = (CHANNELS <= 1) ? 1 : $clog2(CHANNELS);
    localparam integer SEG_W = (SEGMENTS <= 1) ? 1 : $clog2(SEGMENTS);
    localparam integer CHN_W = (SEG_CHANS <= 1) ? 1 : $clog2(SEG_CHANS);
    localparam integer PIXEL_W = $clog2(PIXELS_PER_FRAME);

    reg                         busy_reg;
    reg                         processing;

    reg                         ram_re_reg;
    reg                         ram_data_valid_reg;
    reg [RAM_ADDR_W-1:0]        ram_raddr_reg;
    reg [SEG_W-1:0]             ram_seg_cnt;

    reg [CH_W-1:0]              out_ch_cnt; // 用于跟踪当前 segment 内的 channel 计数
    reg [CH_W-1:0]              ram_ch_cnt;


    wire start_fire = start && !busy_reg; // 仅在非忙状态下响应 start 信号
    wire raddr_wrap = (ram_raddr_reg == RAM_DEPTH - 1);
    wire ram_seg_cnt_wrap = (ram_seg_cnt == SEGMENTS - 1);
    wire ram_ch_cnt_wrap = (ram_ch_cnt == CHANNELS - 1);
    wire done = (ram_ch_cnt_wrap && raddr_wrap); // 当 channel 计数器和地址计数器都回绕时表示全部读完
    reg raddr_wrap_flag; 
    reg start_flag;

    always @(posedge clk) begin
        if (start_fire) begin
            start_flag <= 1'b1;
        end else begin
            start_flag <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            processing <= 1'b0;
        end else if(start_flag) begin
            processing <= 1'b1; // 启动时置processing状态
        end else if(done && ram_re_reg) begin
            processing <= 1'b0; // 全部读完后清除processing状态
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy_reg <= 1'b0;
        end else if (start_fire) begin
            busy_reg <= 1'b1; // 启动时置忙状态
        end else if (out_valid && out_ready && end_all_frame) begin
            busy_reg <= 1'b0; // 全部读完后清除忙状态
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_raddr_reg <= {RAM_ADDR_W{1'b0}};
        end else if(start_fire) begin
            ram_raddr_reg <= {RAM_ADDR_W{1'b0}}; // 启动时地址重置
        end else if(ram_re_reg) begin
            ram_raddr_reg <= (raddr_wrap) ? {RAM_ADDR_W{1'b0}} : ram_raddr_reg + 1'b1; // 每发出一个读请求地址加1
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_ch_cnt <= {CH_W{1'b0}};
        end else if(start_flag) begin
            ram_ch_cnt <= {CH_W{1'b0}};
        end else if(raddr_wrap && ram_re_reg) begin
            ram_ch_cnt <= (ram_ch_cnt_wrap) ? {CH_W{1'b0}} : ram_ch_cnt + 1'b1;
        end
    end

    wire can_read = processing && !fifo_almost_full;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_re_reg <= 1'b0;
        end else if(start_fire) begin
            ram_re_reg <= 1'b0;
        end else if (done) begin
            ram_re_reg <= 1'b0;
        end else begin
            ram_re_reg <= can_read;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_data_valid_reg <= 1'b0;
        end else if(start_flag) begin
            ram_data_valid_reg <= 1'b0; // 启动时清除数据有效标志
        end else begin
            ram_data_valid_reg <= ram_re_reg; // 读请求发出后一个周期数据有效
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ram_seg_cnt <= {SEG_W{1'b0}};
        end else if (start_flag) begin
            ram_seg_cnt <= {SEG_W{1'b0}}; // 启动时 segment 计数器重置
        end else if (ram_data_valid_reg) begin
            ram_seg_cnt <= (ram_seg_cnt_wrap) ? {SEG_W{1'b0}} : ram_seg_cnt + 1'b1; // 每读完一个 segment 切换到下一个 segment
        end
    end


    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            raddr_wrap_flag <= 1'b0;
        end else begin
            raddr_wrap_flag <= raddr_wrap; // 将地址回绕信号延迟一个周期，以便在数据有效时使用
        end
    end

    wire out_ch_wrap = (out_ch_cnt == CHANNELS - 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_ch_cnt <= {CH_W{1'b0}};
        end else if (start_flag) begin
            out_ch_cnt <= {CH_W{1'b0}};
        end else if (ram_data_valid_reg && raddr_wrap_flag ) begin
            out_ch_cnt <= (out_ch_wrap) ? {CH_W{1'b0}} : out_ch_cnt + 1'b1; // 每读完一个 channel 切换到下一个 channel
        end
    end

    wire [CHN_W-1:0] seg_ch_cnt = out_ch_cnt[CHN_W-1:0]; // 当前 segment 内的 channel 索引
    wire [SEG_W-1:0] seg_cnt;
    generate
        if(SEGMENTS > 1) 
            assign seg_cnt = out_ch_cnt[CH_W-1:CHN_W]; // 当前 segment 索引
        else
            assign seg_cnt = ram_seg_cnt; // 如果有多个 segment，直接使用 segment 计数器        
    endgenerate


    wire [OUT_W-1:0] ram_data_sel = ram_rdata[seg_ch_cnt * OUT_W +: OUT_W]; // 从读出的数据中选择当前 segment 的部分
    wire seg_aligned = (ram_seg_cnt == seg_cnt); // 仅当读到的 segment 与当前 segment 计数器对齐时才认为数据有效
    reg [OUT_W-1:0] fifo_din_reg;
    reg fifo_wr_en_reg;
    reg [PIXEL_W-1:0] out_pixel_cnt;
    reg end_frame_align, end_all_frame_align;
    wire out_pixel_cnt_wrap = (out_pixel_cnt == PIXELS_PER_FRAME - 1);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fifo_din_reg <= {OUT_W{1'b0}};
            fifo_wr_en_reg <= 1'b0;
            out_pixel_cnt <= {PIXEL_W{1'b0}};
            end_frame_align <= 1'b0;
            end_all_frame_align <= 1'b0;
        end else if (ram_data_valid_reg && seg_aligned) begin
            fifo_din_reg <= ram_data_sel;
            fifo_wr_en_reg <= 1'b1;
            out_pixel_cnt <= (out_pixel_cnt_wrap) ? {PIXEL_W{1'b0}} : out_pixel_cnt + 1'b1; // 每读完一个像素计数器加1
            end_frame_align <= out_pixel_cnt_wrap; // 当像素计数器回绕时表示当前帧结束
            end_all_frame_align <= out_pixel_cnt_wrap && out_ch_wrap;
        end else begin
            fifo_wr_en_reg <= 1'b0;
            end_frame_align <= 1'b0;
            end_all_frame_align <= 1'b0;
        end
    end

    wire                        fifo_valid;
    wire [OUT_W + 2 -1:0]       fifo_dout;

    assign busy = busy_reg;

    assign ram_re = ram_re_reg;
    assign ram_raddr = ram_raddr_reg;

    fwft_fifo_reg #(
        .WIDTH(OUT_W+2), // 数据位宽 + 2 个标志位(end_frame 和 end_all_frame)
        .DEPTH(FIFO_DEPTH),
        .AF_LEVEL(FIFO_AF_LEVEL)
    ) u_out_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .din({end_all_frame_align, end_frame_align, fifo_din_reg}),
        .wr_en(fifo_wr_en_reg),
        .full(),
        .almost_full(fifo_almost_full),
        .rd_en(out_ready),
        .dout(fifo_dout),
        .valid(fifo_valid),
        .empty()
    );
    assign out_valid = fifo_valid;
    assign out_data = fifo_dout[OUT_W-1:0];
    assign end_frame = fifo_dout[OUT_W];
    assign end_all_frame = fifo_dout[OUT_W + 1];

    //test===========================================
    // reg [OUT_W-1:0] rd_data_reg_test[0:SEG_CHANS-1];
    // integer test_i;
    // always @(*) begin
    //     for (test_i = 0; test_i < SEG_CHANS; test_i = test_i + 1) begin
    //         rd_data_reg_test[test_i] = ram_rdata[test_i * OUT_W +: OUT_W];
    //     end
    // end
    // reg [31:0] ram_read_cnt;
    // reg [OUT_W-1:0] ram_read_pixel_data_test;
    // always @(posedge clk or negedge rst_n) begin
    //     if (!rst_n) begin
    //         ram_read_cnt <= 0;
    //         ram_read_pixel_data_test <= {OUT_W{1'b0}};
    //     end else begin
    //         if(out_ready && out_valid) begin
    //             $display("beat %d: FIFO_Out_Data: %h, End_Frame: %b, End_All_Frame: %b", ram_read_cnt, out_data, fifo_dout[OUT_W], fifo_dout[OUT_W + 1]);
    //             ram_read_cnt <= ram_read_cnt + 1;
    //         end
    //     end
    // end

endmodule
