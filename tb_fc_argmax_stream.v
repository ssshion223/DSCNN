`timescale 1ns/1ps

// FC + argmax 单模块 testbench：
// 连续送入 64 个输入，检查输出类别和最大分数是否符合 memh 中的示例参数。
module tb_fc_argmax_stream;

    localparam integer DATA_W    = 8;
    localparam integer IN_CH     = 64;
    localparam integer OUT_CLASS = 12;
    localparam integer ACC_W     = 40;

    reg               clk;
    reg               rst_n;
    reg               in_valid;
    wire              in_ready;
    reg  [DATA_W-1:0] in_data;
    wire              out_valid;
    wire [7:0]        out_class;
    wire [ACC_W-1:0]  out_score;

    integer           in_idx;
    integer           error_count;

    task send_input;
        input [7:0] sample_data;
        begin
            @(posedge clk);                  // 开始发送一个输入拍
            while (in_ready == 1'b0) begin
                @(posedge clk);              // 等待 DUT 进入可接收状态
            end
            in_valid <= 1'b1;
            in_data  <= sample_data;
            @(posedge clk);                  // valid 保持一拍
            in_valid <= 1'b0;
            in_data  <= 8'h00;
        end
    endtask

    fc_argmax_stream #(
        .DATA_W(DATA_W),
        .IN_CH(IN_CH),
        .OUT_CLASS(OUT_CLASS),
        .ACC_W(ACC_W),
        .WEIGHT_INIT_FILE("fc_weight.memh"),
        .BIAS_INIT_FILE("fc_bias.memh")
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_data(in_data),
        .out_valid(out_valid),
        .out_class(out_class),
        .out_score(out_score)
    );

    always #5 clk = ~clk;  // 100MHz 时钟

    initial begin  // 主激励与结果检查
        clk         = 1'b0;
        rst_n       = 1'b0;
        in_valid    = 1'b0;
        in_data     = 8'h00;
        error_count = 0;

        repeat (4) @(posedge clk);  // 复位阶段
        rst_n = 1'b1;

        for (in_idx = 0; in_idx < IN_CH; in_idx = in_idx + 1) begin
            send_input(8'h01);      // 模拟 GAP 连续输出 64 个 1
        end

        wait (out_valid == 1'b1);   // 等待最终分类结果
        @(posedge clk);


        $finish;
    end

endmodule
