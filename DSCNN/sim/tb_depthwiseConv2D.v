`timescale 1ns/1ns

//==============================================================================
// 模块名：tb_depthwiseConv2D
// 功能说明：
//   顶层 depthwiseConv2D 的 testbench，覆盖 Bias/量化/ReLU 全路径。
// 时钟与复位：
//   等效 100MHz 时钟（10ns 周期），低有效复位时序。
// 激励输入：
//   从存储文件读取像素流，并可选随机 valid/ready 压测。
// 结果校验：
//   与软件生成黄金值比较输出像素和帧标志。
// 时序特性：
//   端到端流握手，使用 WAIT_CYC_MAX 做超时保护。
//==============================================================================
module tb_depthwiseConv2D;
    // 说明：testbench 顶层无模块输入/输出端口，均通过内部信号驱动 DUT。
    localparam integer DATA_W        = 8;  // 输入像素位宽
    localparam integer COEFF_W       = 8;  // 卷积系数位宽
    localparam integer K_H           = 10; // 卷积核高度
    localparam integer K_W           = 4;  // 卷积核宽度
    localparam integer MUL_W         = DATA_W + COEFF_W;
    localparam integer SUM_W         = MUL_W + $clog2(K_H*K_W); // K_H*K_W=1*1时的累加位宽
    localparam integer RESULT_W      = 32; // 最终结果位宽，足够大以防溢出
    localparam integer COL           = 10;
    localparam integer ROW           = 49;
    localparam integer STRIDE        = 2;
    localparam integer PAD_TOP       = 4;
    localparam integer PAD_BOTTOM    = 5;
    localparam integer PAD_LEFT      = 1;
    localparam integer PAD_RIGHT     = 1;
    localparam integer COEFF_GRP_NUM = 64;
    localparam integer MAC_PIPELINE  = 1;
 
    localparam integer OUT_WIDTH     = 8;
    localparam integer SHIFT_VAL     = 16;
    localparam integer GROUP_BITS    = 1;
    localparam integer GROUP_SIZE    = 1;
    localparam integer CH_BITS       = 6;
    localparam integer FIFO_DEPTH    = 16;
    localparam integer FIFO_AF_LEVEL = 14;

    localparam integer IN_VALID_RND_PROB  = 70;
    localparam integer OUT_READY_RND_PROB = 70;
    localparam integer ENABLE_RAND_TEST   = 1;

    localparam integer FRAMES_PER_ROUND = 1;
    localparam integer LOOP_ROUNDS      = 64;
    localparam integer WAIT_CYC_MAX     = 4000;

    localparam COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/rtl/dw/DS-CNN_dw0.memh";
    localparam BIAS_INIT_FILE  = "D:/vivado/exp/DSCNN/rtl/dw/DS-CNN_dw0_Fold_bias.hex";
    localparam INPUT_INIT_FILE = "D:/vivado/exp/DSCNN/rtl/pixel_input/input_pixels.memh";

    localparam integer WIN_SIZE      = K_H * K_W;
    localparam integer P_COL         = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW         = ROW + PAD_TOP + PAD_BOTTOM;
    localparam integer OUT_W         = ((P_COL - K_W) / STRIDE) + 1;
    localparam integer OUT_H         = ((P_ROW - K_H) / STRIDE) + 1;
    localparam integer OUT_SIZE      = OUT_W * OUT_H;
    localparam integer EXP_BEATS_PER_ROUND = FRAMES_PER_ROUND * OUT_SIZE;
    localparam integer TOTAL_FRAMES  = FRAMES_PER_ROUND * LOOP_ROUNDS;
    localparam integer EXP_BEATS_TOTAL = TOTAL_FRAMES * OUT_SIZE;
    localparam integer BIAS_DEPTH    = (1 << (GROUP_BITS + CH_BITS));
    localparam integer CH_MASK       = (1 << CH_BITS) - 1;

    reg clk;
    reg rst_n;
    reg in_valid;
    wire in_ready;
    reg signed [DATA_W-1:0] in_pixel;

    wire [OUT_WIDTH-1:0] out_pixel;
    wire out_valid;
    reg out_ready;
    wire out_end_frame;
    wire out_end_all_frame;

    reg signed [DATA_W-1:0] img_mem [0:ROW*COL-1];
    reg signed [DATA_W-1:0] padded_mem [0:P_ROW*P_COL-1];

    reg [WIN_SIZE*COEFF_W-1:0] ker_grp_mem [0:COEFF_GRP_NUM-1];
    reg signed [COEFF_W-1:0] ker_mem [0:COEFF_GRP_NUM*WIN_SIZE-1];

    reg signed [RESULT_W-1:0] bias_mem [0:BIAS_DEPTH-1];

    reg [OUT_WIDTH-1:0] exp_pix_mem [0:EXP_BEATS_TOTAL-1];

    integer i;
    integer r;
    integer c;
    integer ky;
    integer kx;
    integer f;
    integer oy;
    integer ox;
    integer in_idx;
    integer out_idx;
    integer err_cnt;
    integer cyc;
    integer conv_grp;
    integer bias_grp;
    integer bias_ch;
    integer bias_addr;
    integer frame_bias;
    integer acc;
    integer sum_post;
    integer q_val;
    reg signed [63:0] mul_post;
    reg signed [63:0] q_shift;
    integer exp_end_frame;
    integer exp_end_all;
    integer sample_idx;
    integer frame_idx;
    integer round_idx;
    integer round_fail_cnt;
    integer err_before_round;
    integer round_target_out;
    integer rand_val;
    integer in_valid_count;
    integer out_ready_count;
    integer in_stall_count;
    integer out_stall_count;

    reg [OUT_WIDTH-1:0] got_pix;
    reg [OUT_WIDTH-1:0] exp_pix;

    localparam integer QUANT_MULT  = 915;
    localparam integer OUT_MAX_VAL = (1 << (OUT_WIDTH-1)) - 1;
    localparam integer PRINT_BEAT_START = 3999-125;
    localparam integer PRINT_BEAT_END   = 3999;

    depthwiseConv2D #(
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
        .COEFF_GRP_NUM(COEFF_GRP_NUM),
        .MAC_PIPELINE(MAC_PIPELINE),
        .COEFF_INIT_FILE(COEFF_INIT_FILE),
        .OUT_WIDTH(OUT_WIDTH),
        .SHIFT_VAL(SHIFT_VAL),
        .GROUP_BITS(GROUP_BITS),
        .GROUP_SIZE(GROUP_SIZE),
        .CH_BITS(CH_BITS),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_AF_LEVEL(FIFO_AF_LEVEL)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .out_pixel(out_pixel),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_end_frame(out_end_frame),
        .out_end_all_frame(out_end_all_frame)
    );

    // Override internal bias file for deterministic verification.
    defparam dut.u_bias_quant_relu.BIAS_INIT_FILE = BIAS_INIT_FILE;

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
        $display("[TB] depthwiseConv2D Test Start");
        $display("[TB] COL=%0d ROW=%0d K=%0dx%0d STRIDE=%0d", COL, ROW, K_H, K_W, STRIDE);
        $display("[TB] PAD_TOP=%0d PAD_BOTTOM=%0d PAD_LEFT=%0d PAD_RIGHT=%0d", PAD_TOP, PAD_BOTTOM, PAD_LEFT, PAD_RIGHT);
        $display("[TB] OUT_W=%0d OUT_H=%0d beats/frame=%0d", OUT_W, OUT_H, OUT_SIZE);
        $display("[TB] FRAMES_PER_ROUND=%0d, LOOP_ROUNDS=%0d", FRAMES_PER_ROUND, LOOP_ROUNDS);
        $display("[TB] SHIFT_VAL=%0d OUT_WIDTH=%0d", SHIFT_VAL, OUT_WIDTH);
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
        load_bias();
        print_input_matrix();
        print_kernels();
        print_bias_table();
        build_expected();
        print_expected_preview();

        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        for (round_idx = 0; round_idx < LOOP_ROUNDS; round_idx = round_idx + 1) begin
            $display("\n[TB][ROUND] ===== round=%0d/%0d =====", round_idx + 1, LOOP_ROUNDS);

            err_before_round = err_cnt;
            round_target_out = (round_idx + 1) * EXP_BEATS_PER_ROUND;
            for (f = 0; f < FRAMES_PER_ROUND; f = f + 1) begin
                feed_one_frame();
                repeat (2) @(posedge clk);
            end

            cyc = 0;
            while ((out_idx < round_target_out) && (cyc < WAIT_CYC_MAX)) begin
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

            if (out_idx != round_target_out) begin
                $display("[TB][FAIL] round=%0d output beat mismatch: got=%0d exp=%0d cyc=%0d",
                         round_idx + 1, out_idx, round_target_out, cyc);
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
            $display("[TB][FAIL] parallel module test FAILED, err_cnt=%0d round_fail_cnt=%0d/%0d",
                     err_cnt, round_fail_cnt, LOOP_ROUNDS);
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
            if (out_idx >= EXP_BEATS_TOTAL) begin
                $display("[TB][FAIL] extra beat out_idx=%0d", out_idx);
                err_cnt = err_cnt + 1;
            end else begin
                got_pix = out_pixel;
                exp_pix = exp_pix_mem[out_idx];
                if (got_pix !== exp_pix) begin
                    $display("[TB][FAIL] beat=%0d ch=0 got=%0d exp=%0d", out_idx, got_pix, exp_pix);
                    err_cnt = err_cnt + 1;
                end else if (out_idx >= PRINT_BEAT_START && out_idx <= PRINT_BEAT_END) begin
                    $display("[TB][OK] beat=%0d ch=0 got=%0d", out_idx, got_pix);
                end

                sample_idx = out_idx % OUT_SIZE;
                frame_idx = out_idx / OUT_SIZE;
                exp_end_frame = (sample_idx == OUT_SIZE - 1);
                exp_end_all = exp_end_frame && ((frame_idx % COEFF_GRP_NUM) == (COEFF_GRP_NUM - 1));

                if (out_end_frame !== exp_end_frame[0]) begin
                    $display("[TB][FAIL] beat=%0d end_frame got=%0d exp=%0d", out_idx, out_end_frame, exp_end_frame);
                    err_cnt = err_cnt + 1;
                end
                if (out_end_all_frame !== exp_end_all[0]) begin
                    $display("[TB][FAIL] beat=%0d end_all_frame got=%0d exp=%0d", out_idx, out_end_all_frame, exp_end_all);
                    err_cnt = err_cnt + 1;
                end

                if (out_idx >= PRINT_BEAT_START && out_idx <= PRINT_BEAT_END) begin
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

    task load_coeff;
    integer g;
    integer k;
    begin
        $readmemh(COEFF_INIT_FILE, ker_grp_mem, 0, COEFF_GRP_NUM-1);
        for (g = 0; g < COEFF_GRP_NUM; g = g + 1) begin
            for (k = 0; k < WIN_SIZE; k = k + 1) begin
                ker_mem[g*WIN_SIZE + k] = ker_grp_mem[g][(k+1)*COEFF_W-1 -: COEFF_W];
            end
        end
    end
    endtask

    task load_bias;
    begin
        $readmemh(BIAS_INIT_FILE, bias_mem, 0, BIAS_DEPTH-1);
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
    integer g;
    begin
        for (g = 0; g < COEFF_GRP_NUM; g = g + 1) begin
            $display("[TB] Kernel Group %0d:", g);
            for (ky = 0; ky < K_H; ky = ky + 1) begin
                $write("[TB]     ");
                for (kx = 0; kx < K_W; kx = kx + 1) begin
                    $write("%4d ", ker_mem[g*WIN_SIZE + ky*K_W + kx]);
                end
                $display("");
            end
            $display("");
        end
    end
    endtask

    task print_bias_table;
    begin
        $display("[TB] Bias table (addr: value):");
        for (i = 0; i < BIAS_DEPTH; i = i + 1) begin
            $display("[TB]   bias[%0d] = %0d", i, bias_mem[i]);
        end
        $display("");
    end
    endtask

    task build_expected;
    begin
        out_idx = 0;
        bias_grp = 0;
        bias_ch  = 0;

        for (f = 0; f < TOTAL_FRAMES; f = f + 1) begin
            conv_grp = f % COEFF_GRP_NUM;
            bias_addr = (bias_grp << CH_BITS) + bias_ch;
            frame_bias = bias_mem[bias_addr];

            for (oy = 0; oy < OUT_H; oy = oy + 1) begin
                for (ox = 0; ox < OUT_W; ox = ox + 1) begin
                    acc = 0;
                    for (ky = 0; ky < K_H; ky = ky + 1) begin
                        for (kx = 0; kx < K_W; kx = kx + 1) begin
                            acc = acc
                                + padded_mem[(oy*STRIDE + ky)*P_COL + (ox*STRIDE + kx)]
                                * ker_mem[conv_grp*WIN_SIZE + ky*K_W + kx];
                        end
                    end

                    sum_post = acc + frame_bias;
                    mul_post = $signed(sum_post) * QUANT_MULT;

                    if (mul_post < 0) begin
                        exp_pix_mem[out_idx] = {OUT_WIDTH{1'b0}};
                    end else begin
                        q_shift = mul_post >>> SHIFT_VAL;
                        q_val = q_shift;
                        if (q_shift > OUT_MAX_VAL) begin
                            exp_pix_mem[out_idx] = OUT_MAX_VAL[OUT_WIDTH-1:0];
                        end else begin
                            exp_pix_mem[out_idx] = q_val[OUT_WIDTH-1:0];
                        end
                    end

                    out_idx = out_idx + 1;
                end
            end

            // Match bias_process_wrapper group/channel update policy.
            if (conv_grp == COEFF_GRP_NUM - 1) begin
                if (bias_grp == GROUP_SIZE - 1) begin
                    bias_grp = 0;
                end else begin
                    bias_grp = bias_grp + 1;
                end
                bias_ch = 0;
            end else begin
                bias_ch = (bias_ch + 1) & CH_MASK;
            end
        end

        out_idx = 0;
    end
    endtask

    task print_expected_preview;
    integer idx;
    integer preview_end_frame;
    integer preview_end_all;
    begin
        $display("[TB] Expected output preview, total beats=%0d:", EXP_BEATS_TOTAL);
        for (idx = 0; idx < EXP_BEATS_TOTAL; idx = idx + 1) begin
            if (idx >= PRINT_BEAT_START && idx <= PRINT_BEAT_END) begin
                preview_end_frame = ((idx % OUT_SIZE) == (OUT_SIZE - 1));
                preview_end_all = preview_end_frame && (((idx / OUT_SIZE) % COEFF_GRP_NUM) == (COEFF_GRP_NUM - 1));
                $display("[TB]   exp beat=%0d pixel=%0d end_frame=%0b end_all_frame=%0b",
                         idx, exp_pix_mem[idx], preview_end_frame[0], preview_end_all[0]);
            end else if (idx == PRINT_BEAT_START) begin
                $display("[TB]   ... middle beats omitted ...");
            end
        end
        $display("");
    end
    endtask

    task feed_one_frame;
    begin
        in_idx = 0;
        while (in_idx < ROW*COL) begin
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

            if (in_ready) begin
                if (in_valid) begin
                    in_pixel = img_mem[in_idx];
                    in_idx = in_idx + 1;
                end
            end
        end

        @(negedge clk);
        in_valid = 1'b0;
        in_pixel = {DATA_W{1'b0}};
    end
    endtask

endmodule
