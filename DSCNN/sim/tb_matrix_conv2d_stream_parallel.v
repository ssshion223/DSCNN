`timescale 1ns/1ns

//==============================================================================
// 模块名：tb_matrix_conv2d_stream_parallel
// 功能说明：
//   matrix_conv2d_stream_parallel 卷积核心 testbench。
// 时钟与复位：
//   等效 100MHz 时钟（10ns 周期），低有效复位时序。
// 激励输入：
//   从文件加载像素流和系数组，并可选随机 valid/ready 压测。
// 结果校验：
//   与软件生成黄金值比较卷积和与帧标志。
// 时序特性：
//   流握手驱动，带循环超时保护。
//==============================================================================
module tb_matrix_conv2d_stream_parallel;
    // 说明：testbench 顶层无模块输入/输出端口，均通过内部信号驱动 DUT。
    // ==================== 基础参数 ====================
    localparam integer DATA_W  = 8;  // 输入像素位宽
    localparam integer COEFF_W = 8;  // 卷积系数位宽
    localparam integer COL     = 10; // 输入列数
    localparam integer ROW     = 49; // 输入行数
    localparam integer K_H     = 10; // 卷积核高度
    localparam integer K_W     = 4;  // 卷积核宽度
    localparam integer MUL_W   = DATA_W + COEFF_W;
    localparam integer SUM_W   = MUL_W + $clog2(K_H*K_W);
    localparam integer STRIDE  = 2;
    localparam integer PAD_TOP    = 4;
    localparam integer PAD_BOTTOM = 5;
    localparam integer PAD_LEFT   = 1;
    localparam integer PAD_RIGHT  = 1;
    localparam integer OUT_CH  = 1;
    localparam integer COEFF_GRP_NUM = 64;
    localparam integer MAC_PIPELINE = 1;
    localparam COEFF_INIT_FILE = "D:\\vivado\\exp\\SmartCar\\rtl\\dw\\DS-CNN_dw0.memh";
    localparam INPUT_INIT_FILE = "D:/vivado/exp/SmartCar/sim/pixel_input/input_pixels.memh";

    // ==================== 随机流控参数 ====================
    localparam integer IN_VALID_RND_PROB  = 70;
    localparam integer OUT_READY_RND_PROB = 70;
    localparam integer ENABLE_RAND_TEST   = 1;

    // ==================== 测试控制参数 ====================
    localparam integer FRAMES_PER_ROUND = 64;
    localparam integer LOOP_ROUNDS      = 1;
    localparam integer WAIT_CYC_MAX     = 12000;

    localparam integer WIN_SIZE   = K_H * K_W;
    localparam integer ROM_OUT_CH = OUT_CH * COEFF_GRP_NUM;
    localparam integer GROUP_SIZE = OUT_CH * WIN_SIZE;
    localparam integer GRP_COEFF_BUS_W = GROUP_SIZE * COEFF_W;
    localparam integer P_COL      = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW      = ROW + PAD_TOP + PAD_BOTTOM;
    localparam integer OUT_W      = ((P_COL - K_W) / STRIDE) + 1;
    localparam integer OUT_H      = ((P_ROW - K_H) / STRIDE) + 1;
    localparam integer OUT_SIZE   = OUT_W * OUT_H;
    localparam integer EXP_BEATS_PER_ROUND = FRAMES_PER_ROUND * OUT_SIZE;

    reg clk;
    reg rst_n;
    reg in_valid;
    wire in_ready;
    reg signed [DATA_W-1:0] in_pixel;

    wire out_valid;
    reg out_ready;
    wire out_end_all_frame;
    wire out_end_frame;
    wire signed [OUT_CH*SUM_W-1:0] out_pixel_data_bus;

    reg signed [DATA_W-1:0] img_mem [0:ROW*COL-1];
    reg signed [DATA_W-1:0] padded_mem [0:P_ROW*P_COL-1];
    reg [GRP_COEFF_BUS_W-1:0] ker_grp_mem [0:COEFF_GRP_NUM-1];
    reg signed [COEFF_W-1:0] ker_mem [0:ROM_OUT_CH*WIN_SIZE-1];
    reg signed [SUM_W-1:0] exp_mem [0:EXP_BEATS_PER_ROUND*OUT_CH-1];

    integer i;
    integer r;
    integer c;
    integer ky;
    integer kx;
    integer f;
    integer ch;
    integer grp;
    integer sample_idx;
    integer frame_idx;
    integer out_idx;
    integer err_cnt;
    integer cyc;
    integer in_idx;
    integer oy;
    integer ox;
    integer acc;
    integer exp_frame;
    integer round_idx;
    integer round_fail_cnt;
    integer err_before_round;
    integer rand_val;
    integer in_valid_count;
    integer out_ready_count;
    integer in_stall_count;
    integer out_stall_count;

    reg signed [SUM_W-1:0] got_ch;
    reg signed [SUM_W-1:0] exp_ch;

    matrix_conv2d_stream_parallel #(
        .DATA_W(DATA_W),
        .COEFF_W(COEFF_W),
        .MUL_W(MUL_W),
        .SUM_W(SUM_W),
        .COL(COL),
        .ROW(ROW),
        .K_H(K_H),
        .K_W(K_W),
        .STRIDE(STRIDE),
        .PAD_TOP(PAD_TOP),
        .PAD_BOTTOM(PAD_BOTTOM),
        .PAD_LEFT(PAD_LEFT),
        .PAD_RIGHT(PAD_RIGHT),
        .OUT_CH(OUT_CH),
        .COEFF_GRP_NUM(COEFF_GRP_NUM),
        .MAC_PIPELINE(MAC_PIPELINE),
        .COEFF_INIT_FILE(COEFF_INIT_FILE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_end_all_frame(out_end_all_frame),
        .out_end_frame(out_end_frame),
        .out_pixel_data_bus(out_pixel_data_bus)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        in_valid = 1'b0;
        in_pixel = {DATA_W{1'b0}};
        out_ready = 1'b1;
        out_idx = 0;
        err_cnt = 0;
        round_fail_cnt = 0;
        in_valid_count = 0;
        out_ready_count = 0;
        in_stall_count = 0;
        out_stall_count = 0;

        $display("\n===============================================");
        $display("[TB] matrix_conv2d_stream_parallel Test Start");
        $display("[TB] K_H=%0d, K_W=%0d, PAD_TOP=%0d, PAD_BOTTOM=%0d, PAD_LEFT=%0d, PAD_RIGHT=%0d", K_H, K_W, PAD_TOP, PAD_BOTTOM, PAD_LEFT, PAD_RIGHT);
        $display("[TB] OUT_CH=%0d, COEFF_GRP_NUM=%0d", OUT_CH, COEFF_GRP_NUM);
        $display("[TB] OUT_W=%0d, OUT_H=%0d, beats/frame=%0d", OUT_W, OUT_H, OUT_SIZE);
        $display("[TB] FRAMES_PER_ROUND=%0d, LOOP_ROUNDS=%0d", FRAMES_PER_ROUND, LOOP_ROUNDS);
        $display("===============================================\n");

        if (ENABLE_RAND_TEST) begin
            $display("[TB][RANDOM] Random stress test ENABLED");
            $display("[TB][RANDOM] IN_VALID_RND_PROB=%0d%%", IN_VALID_RND_PROB);
            $display("[TB][RANDOM] OUT_READY_RND_PROB=%0d%%", OUT_READY_RND_PROB);
            $display("");
        end

        init_input();
        build_padded();
        load_coeff();
        print_input_matrix();
        print_kernels();
        build_expected();

        for (round_idx = 0; round_idx < LOOP_ROUNDS; round_idx = round_idx + 1) begin
            $display("\n[TB][ROUND] ===== round=%0d/%0d =====", round_idx + 1, LOOP_ROUNDS);

            rst_n = 1'b0;
            in_valid = 1'b0;
            in_pixel = {DATA_W{1'b0}};
            out_ready = 1'b1;
            out_idx = 0;
            repeat (6) @(posedge clk);
            @(negedge clk);
            rst_n = 1'b1;
            repeat (2) @(posedge clk);

            err_before_round = err_cnt;
            for (f = 0; f < FRAMES_PER_ROUND; f = f + 1) begin
                feed_one_frame();
                repeat (2) @(posedge clk);
            end

            cyc = 0;
            while ((out_idx < EXP_BEATS_PER_ROUND) && (cyc < WAIT_CYC_MAX)) begin
                @(negedge clk);
                if (ENABLE_RAND_TEST) begin
                    rand_val = ($random & 32'h7fffffff) % 100;
                    if (rand_val < OUT_READY_RND_PROB) begin
                        out_ready = 1'b1;
                        out_ready_count = out_ready_count + 1;
                    end else begin
                        out_ready = 1'b0;
                        out_stall_count = out_stall_count + 1;
                    end
                end
                cyc = cyc + 1;
            end

            if (out_idx != EXP_BEATS_PER_ROUND) begin
                $display("[TB][FAIL] round=%0d output beat mismatch: got=%0d exp=%0d cyc=%0d", round_idx + 1, out_idx, EXP_BEATS_PER_ROUND, cyc);
                err_cnt = err_cnt + 1;
                round_fail_cnt = round_fail_cnt + 1;
            end else if (err_cnt != err_before_round) begin
                $display("[TB][FAIL] round=%0d data/flag mismatch exists", round_idx + 1);
                round_fail_cnt = round_fail_cnt + 1;
            end else begin
                $display("[TB][PASS] round=%0d all matched", round_idx + 1);
            end
            repeat (20) @(posedge clk);
        end

        if (err_cnt == 0) begin
            $display("[TB][PASS] parallel module test PASSED");
        end else begin
            $display("[TB][FAIL] parallel module test FAILED, err_cnt=%0d round_fail_cnt=%0d/%0d", err_cnt, round_fail_cnt, LOOP_ROUNDS);
        end

        if (ENABLE_RAND_TEST) begin
            $display("\n========================================");
            $display("[TB][RANDOM] Test Statistics");
            $display("========================================");
            $display("[TB][RANDOM] in_valid=1 cycles: %0d", in_valid_count);
            $display("[TB][RANDOM] in_valid=0 cycles: %0d", in_stall_count);
            $display("[TB][RANDOM] out_ready=1 cycles: %0d", out_ready_count);
            $display("[TB][RANDOM] out_ready=0 cycles: %0d", out_stall_count);
            $display("========================================\n");
        end

        #20;
        $stop;
    end

    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            if (out_idx >= EXP_BEATS_PER_ROUND) begin
                $display("[TB][FAIL] extra beat out_idx=%0d", out_idx);
                err_cnt = err_cnt + 1;
            end else begin
                sample_idx = out_idx % OUT_SIZE;
                frame_idx = out_idx / OUT_SIZE;

                for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
                    got_ch = out_pixel_data_bus[ch*SUM_W +: SUM_W];
                    exp_ch = exp_mem[out_idx*OUT_CH + ch];
                    if (got_ch !== exp_ch) begin
                        $display("[TB][FAIL] beat=%0d ch=%0d got=%0d exp=%0d", out_idx, ch, got_ch, exp_ch);
                        err_cnt = err_cnt + 1;
                    end else if (out_idx < 10 || out_idx >= (EXP_BEATS_PER_ROUND - 10)) begin
                        $display("[TB][OK] beat=%0d ch=%0d got=%0d", out_idx, ch, got_ch);
                    end
                end

                exp_frame = (sample_idx == OUT_SIZE - 1);

                if (out_end_frame !== exp_frame[0]) begin
                    $display("[TB][FAIL] beat=%0d end_frame got=%0d exp=%0d", out_idx, out_end_frame, exp_frame);
                    err_cnt = err_cnt + 1;
                end

                if (out_end_all_frame !== (exp_frame && ((frame_idx % COEFF_GRP_NUM) == (COEFF_GRP_NUM - 1)))) begin
                    $display("[TB][FAIL] beat=%0d end_all_frame got=%0d exp=%0d", out_idx, out_end_all_frame, exp_frame && ((frame_idx % COEFF_GRP_NUM) == (COEFF_GRP_NUM - 1)));
                    err_cnt = err_cnt + 1;
                end

                if (out_idx < 10 || out_idx >= (EXP_BEATS_PER_ROUND - 10)) begin
                    $display("[TB][OK] beat=%0d end_frame=%0b end_all_frame=%0b", out_idx, out_end_frame, out_end_all_frame);
                end
            end
            out_idx = out_idx + 1;
        end
    end

    task init_input;
    begin
        for (i = 0; i < ROW*COL; i = i + 1) begin
            img_mem[i] = {DATA_W{1'b0}};
        end
        $readmemh(INPUT_INIT_FILE, img_mem, 0, ROW*COL-1);
        for (i = 0; i < ROW*COL; i = i + 1) begin
            img_mem[i] = $signed(img_mem[i]);
        end
    end
    endtask

    task build_padded;
    begin
        for (r = 0; r < P_ROW; r = r + 1) begin
            for (c = 0; c < P_COL; c = c + 1) begin
                padded_mem[r*P_COL + c] = {DATA_W{1'b0}};
            end
        end
        for (r = 0; r < ROW; r = r + 1) begin
            for (c = 0; c < COL; c = c + 1) begin
                padded_mem[(r + PAD_TOP)*P_COL + (c + PAD_LEFT)] = img_mem[r*COL + c];
            end
        end
    end
    endtask

    task print_input_matrix;
    begin
        $display("[TB] Input image (%0dx%0d):", ROW, COL);
        for (r = 0; r < ROW; r = r + 1) begin
            $write("[TB]   Row %0d: ", r);
            for (c = 0; c < COL; c = c + 1) begin
                $write("%4d ", img_mem[r*COL + c]);
            end
            $display("");
        end
        $display("");

        $display("[TB] Padded image (%0dx%0d):", P_ROW, P_COL);
        for (r = 0; r < P_ROW; r = r + 1) begin
            $write("[TB]   Row %0d: ", r);
            for (c = 0; c < P_COL; c = c + 1) begin
                $write("%4d ", padded_mem[r*P_COL + c]);
            end
            $display("");
        end
        $display("");
    end
    endtask

    task print_kernels;
    integer base_ch;
    begin
        for (grp = 0; grp < COEFF_GRP_NUM; grp = grp + 1) begin
            $display("[TB] Kernel Group %0d:", grp);
            for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
                base_ch = grp * OUT_CH + ch;
                $display("[TB]   ch=%0d (rom_ch=%0d):", ch, base_ch);
                for (ky = 0; ky < K_H; ky = ky + 1) begin
                    $write("[TB]     ");
                    for (kx = 0; kx < K_W; kx = kx + 1) begin
                        $write("%4d ", ker_mem[base_ch*WIN_SIZE + ky*K_W + kx]);
                    end
                    $display("");
                end
            end
            $display("");
        end
    end
    endtask

    task load_coeff;
    integer g;
    integer ch_idx;
    integer k_idx;
    begin
        // memh format: one packed group per line (OUT_CH*K_H*K_W*COEFF_W)
        $readmemh(COEFF_INIT_FILE, ker_grp_mem, 0, COEFF_GRP_NUM-1);

        // Unpack into flat ker_mem for expected-value and print logic reuse
        for (g = 0; g < COEFF_GRP_NUM; g = g + 1) begin
            for (ch_idx = 0; ch_idx < OUT_CH; ch_idx = ch_idx + 1) begin
                for (k_idx = 0; k_idx < WIN_SIZE; k_idx = k_idx + 1) begin
                    ker_mem[(g*OUT_CH + ch_idx)*WIN_SIZE + k_idx] =
                        ker_grp_mem[g][((ch_idx*WIN_SIZE + k_idx)+1)*COEFF_W-1 -: COEFF_W];
                end
            end
        end
    end
    endtask

    task build_expected;
    integer ker_ch;
    begin
        out_idx = 0;
        for (f = 0; f < FRAMES_PER_ROUND; f = f + 1) begin
            grp = f % COEFF_GRP_NUM;
            for (oy = 0; oy < OUT_H; oy = oy + 1) begin
                for (ox = 0; ox < OUT_W; ox = ox + 1) begin
                    for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
                        ker_ch = grp * OUT_CH + ch;
                        acc = 0;
                        for (ky = 0; ky < K_H; ky = ky + 1) begin
                            for (kx = 0; kx < K_W; kx = kx + 1) begin
                                acc = acc + padded_mem[(oy*STRIDE + ky)*P_COL + (ox*STRIDE + kx)] *
                                           ker_mem[ker_ch*WIN_SIZE + ky*K_W + kx];
                            end
                        end
                        exp_mem[out_idx*OUT_CH + ch] = acc;
                    end
                    out_idx = out_idx + 1;
                end
            end
        end
        out_idx = 0;
    end
    endtask

    task feed_one_frame;
    begin
        in_idx = 0;
        while (in_idx < ROW*COL) begin
            @(negedge clk);

            // 出口反压在喂数阶段也随机，波形上更容易观察到明显翻转
            if (ENABLE_RAND_TEST) begin
                rand_val = ($random & 32'h7fffffff) % 100;
                if (rand_val < OUT_READY_RND_PROB) begin
                    out_ready = 1'b1;
                    out_ready_count = out_ready_count + 1;
                end else begin
                    out_ready = 1'b0;
                    out_stall_count = out_stall_count + 1;
                end
            end else begin
                out_ready = 1'b1;
            end

            if (ENABLE_RAND_TEST) begin
                rand_val = ($random & 32'h7fffffff) % 100;
                if (rand_val < IN_VALID_RND_PROB) begin
                    in_valid = 1'b1;
                    in_valid_count = in_valid_count + 1;
                end else begin
                    in_valid = 1'b0;
                    in_stall_count = in_stall_count + 1;
                end
            end else begin
                in_valid = 1'b1;
                in_valid_count = in_valid_count + 1;
            end

            if (in_ready && in_valid) begin
                in_pixel = img_mem[in_idx];
                in_idx = in_idx + 1;
            end
        end

        @(negedge clk);
        in_valid = 1'b0;
        in_pixel = {DATA_W{1'b0}};
    end
    endtask

endmodule
