`timescale 1ns/1ps

// GAP 单模块 testbench：
// 只负责给 DUT 喂输入数据，不做自动判错。
// 为了便于看波形，额外给出“当前正在发送哪个通道/像素”
// 以及“当前输出理论上应当对应哪个通道、什么平均值”这类辅助信号。
module tb_global_avg_pool_stream;

    localparam integer DATA_W   = 8;
    localparam integer ROW_LEN  = 25;
    localparam integer ROW_NUM  = 5;
    localparam integer CHANNELS = 64;
    localparam integer SUM_W    = 16;

    reg               clk;
    reg               rst_n;
    reg               in_valid;
    wire              in_ready;
    reg  [DATA_W-1:0] in_data;
    wire              out_valid;
    reg               out_ready;
    wire [DATA_W-1:0] out_data;
    wire              out_channel_last;
    wire              out_frame_last;

    integer           ch_idx;
    integer           pix_idx;

    reg  [15:0]       tb_drive_channel;   // 当前正在发送的通道编号
    reg  [15:0]       tb_drive_pixel;     // 当前正在发送的通道内像素编号
    reg  [DATA_W-1:0] tb_drive_value;     // 当前发送到 DUT 的数据值
    reg  [15:0]       tb_out_channel;     // 当前输出对应的通道编号
    reg               tb_out_fire;        // 当前拍是否真正输出了一个平均值
    wire [DATA_W-1:0] tb_expect_out_data; // 当前输出理论平均值，便于直接对波形

    function [7:0] channel_value;
        input integer ch;
        begin
            case (ch % 4)
                0: channel_value = 8'h00;
                1: channel_value = 8'h07;
                2: channel_value = 8'hF8;
                default: channel_value = 8'hFD;
            endcase
        end
    endfunction

    assign tb_expect_out_data = channel_value(tb_out_channel);

    task send_sample;
        input [7:0] sample_data;
        begin
            @(posedge clk);
            while (in_ready == 1'b0) begin
                @(posedge clk);
            end
            in_valid <= 1'b1;
            in_data  <= sample_data;
            @(posedge clk);
            in_valid <= 1'b0;
            in_data  <= 8'h00;
        end
    endtask

    global_avg_pool_stream #(
        .DATA_W(DATA_W),
        .ROW_LEN(ROW_LEN),
        .ROW_NUM(ROW_NUM),
        .CHANNELS(CHANNELS),
        .SUM_W(SUM_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_data(in_data),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .out_channel_last(out_channel_last),
        .out_frame_last(out_frame_last)
    );

    always #5 clk = ~clk;  // 100MHz 时钟

    always @(posedge clk or negedge rst_n) begin  // 输出侧观察辅助寄存器
        if (!rst_n) begin
            tb_out_channel <= 16'd0;
            tb_out_fire    <= 1'b0;
        end else begin
            tb_out_fire <= out_valid && out_ready;
            if (out_valid && out_ready) begin
                if (out_frame_last) begin
                    tb_out_channel <= 16'd0;
                end else begin
                    tb_out_channel <= tb_out_channel + 16'd1;
                end
            end
        end
    end

    initial begin
        clk             = 1'b0;
        rst_n           = 1'b0;
        in_valid        = 1'b0;
        in_data         = 8'h00;
        out_ready       = 1'b1;   // 始终接受 DUT 输出，方便连续观察
        tb_drive_channel = 16'd0;
        tb_drive_pixel   = 16'd0;
        tb_drive_value   = 8'h00;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        for (ch_idx = 0; ch_idx < CHANNELS; ch_idx = ch_idx + 1) begin
            for (pix_idx = 0; pix_idx < (ROW_LEN*ROW_NUM); pix_idx = pix_idx + 1) begin
                tb_drive_channel = ch_idx;
                tb_drive_pixel   = pix_idx;
                tb_drive_value   = channel_value(ch_idx);
                send_sample(tb_drive_value);
            end
        end

        wait (out_valid && out_ready && out_frame_last);  // 等最后一个通道平均值输出
        repeat (20) @(posedge clk);                       // 留出收尾波形
        $finish;
    end

endmodule
