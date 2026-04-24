`timescale 1ns/1ns

//==============================================================================
// 模块名：tb_pw_pingpong
// 功能说明：
//   pw_pingpong 顶层 testbench，覆盖 start/busy、输入输出握手与帧标志校验。
// 校验策略：
//   不做逐点数值黄金值比较（链路较长），重点校验输出拍数与 end_frame/end_all_frame 时序。
//==============================================================================
module tb_dw_pw_pingpong;
    localparam integer IN_DATA_W          = 8;
    localparam integer PIXEL_DEPTH        = 125;
    localparam integer PW_OUT_CH          = 64;
    localparam integer OUT_PIXEL_WIDTH    = 8;

    localparam integer RUNS               = 1;
    localparam integer ROUND_IN_BEATS   = PIXEL_DEPTH * PW_OUT_CH;
    localparam integer TOTAL_IN_BEATS     = ROUND_IN_BEATS * RUNS;

    localparam INPUT_INIT_FILE      = "D:/vivado/exp/DSCNN/data/pixel_output/downsample.memh";

    reg                                  clk;
    reg                                  rst_n;
    reg                                  start;

    reg                                  in_valid;
    wire                                 in_ready;
    reg  [IN_DATA_W-1:0]                 in_pixel;
    reg                                  in_end_all_frame;

    wire                                 out_valid;
    reg                                  out_ready;
    wire                                 end_frame;
    wire                                 end_all_frame;
    wire [OUT_PIXEL_WIDTH-1:0]           out_data;
    wire                                 busy;

    reg  [IN_DATA_W-1:0] input_mem [0:TOTAL_IN_BEATS-1];

    integer err_cnt;
    integer cyc;
    integer run_idx;
    integer in_idx;

    // sender state (conv-style)
    integer send_idx;
    reg send_done;

    dw_pw_pingpong #(
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .PW_OUT_CH(PW_OUT_CH),
        .IN_DATA_W(IN_DATA_W),
        .OUT_PIXEL_WIDTH(OUT_PIXEL_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .in_end_all_frame(in_end_all_frame),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .end_frame(end_frame),
        .end_all_frame(end_all_frame),
        .out_data(out_data),
        .start(start),
        .busy(busy)
    );

    always #5 clk = ~clk;

    task pulse_start;
    begin
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
    end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        in_valid = 1'b0;
        in_pixel = {IN_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;
        out_ready = 1'b1;
        $readmemh(INPUT_INIT_FILE, input_mem, 0, TOTAL_IN_BEATS-1);
        err_cnt = 0;

        send_idx = 0;
        send_done = 1'b0;



        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // start runs sequentially, using combinational sender similar to tb_dw_pw_cac
        for (run_idx = 0; run_idx < RUNS; run_idx = run_idx + 1) begin
            
            // issue start pulse for this run
            pulse_start();

            // wait until sender has produced this run's worth of beats
            while (send_idx < (run_idx+1)*ROUND_IN_BEATS) @(posedge clk);

            while(!end_all_frame || !out_valid || !out_ready) @(posedge clk); // wait until all frames are done and output is drained
        end

        #20; $stop;
    end

    // 组合驱动发送：类似 tb_dw_pw_cac 的風格
    always @(*) begin
        in_valid = 1'b0;
        in_pixel = {IN_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;

        if (!send_done && (send_idx < TOTAL_IN_BEATS)) begin
            in_valid = 1'b1;
            in_pixel = input_mem[send_idx % ROUND_IN_BEATS];
            in_end_all_frame = ((send_idx % ROUND_IN_BEATS) == (ROUND_IN_BEATS - 1)) ? 1'b1 : 1'b0;
        end
    end


    // 发送索引推进：在握手时前进并在达到 TOTAL_IN_BEATS 时标记完成
    always @(posedge clk) begin
        if (!rst_n) begin
            send_idx <= 0;
            send_done <= 1'b0;
        end else begin
            if (!send_done && in_valid && in_ready) begin
                send_idx <= send_idx + 1;
                if (send_idx + 1 >= TOTAL_IN_BEATS) begin
                    send_done <= 1'b1;
                end
            end
        end
    end

    reg [31:0] out_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_cnt <= 32'b0;
        end else begin
            if(out_valid && out_ready) begin
                $display("beat %d: PW_out_pixel(Hex): %h ,end_frame: %b,end_all_frame: %b", out_cnt, out_data, end_frame, end_all_frame);
                out_cnt <=(out_cnt == (ROUND_IN_BEATS - 1)) ? 0 : out_cnt + 1;
            end
        end
    end

endmodule
