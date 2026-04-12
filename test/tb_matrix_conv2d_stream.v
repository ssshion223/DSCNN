`timescale 1ns/1ns

module tb_matrix_conv2d_stream();
    // ==================== 基础参数 ====================
    localparam integer DATA_W  = 8;
    localparam integer COEFF_W = 8;
    localparam integer ACC_W   = 32;
    localparam integer COL     = 20;
    localparam integer ROW     = 20;
    localparam integer K       = 5;
    localparam integer STRIDE  = 1;
    localparam integer OUT_CH  = 3;
    localparam        COEFF_INIT_FILE = "..\\tb_matrix_conv2d_stream\\coeff_init.memh";
    
    // ==================== 随机流控参数 ====================
    localparam integer RANDOM_SEED        = 12345;     // 随机种子
    localparam integer IN_VALID_RND_PROB  = 70;        // in_valid=1 (0-100)
    localparam integer OUT_READY_RND_PROB = 50;        // out_ready=1 (0-100)
    localparam integer ENABLE_RAND_TEST   = 0;         // 0: 禁用随机, 1: 启用随机

    localparam integer WIN_SIZE = K * K;
    localparam integer COEFF_TOTAL = OUT_CH * WIN_SIZE;
    localparam integer IN_SIZE  = COL * ROW;
    localparam integer OUT_W    = ((COL - K) / STRIDE) + 1;
    localparam integer OUT_H    = ((ROW - K) / STRIDE) + 1;
    localparam integer OUT_SIZE = OUT_W * OUT_H;
    localparam integer LOOP_ROUNDS = 20;
    localparam integer WAIT_CYC_MAX = 4000;

    reg clk;
    reg rst_n;

    reg in_valid;
    wire in_ready;
    reg signed [DATA_W-1:0] in_pixel;

    wire out_valid;
    reg out_ready;
    wire out_pixel_end_line;
    wire out_pixel_end_frame;
    wire signed [ACC_W-1:0] out_pixel_data;

    reg signed [DATA_W-1:0]  img_mem [0:IN_SIZE-1];
    reg signed [COEFF_W-1:0] ker_mem [0:COEFF_TOTAL-1];
    reg signed [ACC_W-1:0]   exp_mem [0:OUT_CH*OUT_SIZE-1];

    integer i;
    integer ch;
    integer ox;
    integer oy;
    integer kx;
    integer ky;
    integer in_idx;
    integer out_idx;
    integer exp_total_out;
    integer exp_ch_idx;
    integer exp_sample_idx;
    integer err_cnt;
    integer cyc;
    integer acc;
    integer mismatch;
    integer exp_end_line_flag;
    integer exp_end_frame_flag;
    integer rand_val;
    integer in_valid_count;    // 统计有效输入
    integer out_ready_count;   // 统计就绪输出
    integer in_stall_count;    // 统计输入停滞
    integer out_stall_count;   // 统计输出停滞
    integer round_idx;
    integer round_fail_cnt;
    integer err_before_round;
    reg signed [ACC_W-1:0] got_ch;
    reg signed [ACC_W-1:0] exp_ch;

    matrix_conv2d_stream #(
        .DATA_W(DATA_W),
        .COEFF_W(COEFF_W),
        .ACC_W(ACC_W),
        .COL(COL),
        .ROW(ROW),
        .K(K),
        .STRIDE(STRIDE),
        .OUT_CH(OUT_CH),
        .COEFF_INIT_FILE(COEFF_INIT_FILE),
        .MAC_PIPELINE(1)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_pixel_end_line(out_pixel_end_line),
        .out_pixel_end_frame(out_pixel_end_frame),
        .out_pixel_data(out_pixel_data)
    );


    always #10 clk = ~clk;

    initial begin
        clk          = 1'b0;
        rst_n        = 1'b0;
        in_valid     = 1'b0;
        in_pixel     = {DATA_W{1'b0}};
        out_ready    = 1'b1;
        out_idx      = 0;
        exp_total_out = OUT_SIZE * OUT_CH;
        err_cnt      = 0;
        round_fail_cnt = 0;
        in_valid_count  = 0;
        out_ready_count = 0;
        in_stall_count  = 0;
        out_stall_count = 0;
        
        // 初始化随机数生成器
        if (ENABLE_RAND_TEST) begin
            $display("\n");
            $display("========================================");
            $display("[TB][RANDOM] Random stress test ENABLED");
            $display("[TB][RANDOM] IN_VALID_RND_PROB=%0d%%", IN_VALID_RND_PROB);
            $display("[TB][RANDOM] OUT_READY_RND_PROB=%0d%%", OUT_READY_RND_PROB);
            $display("========================================\n");
        end else begin
            $display("[TB] Standard test (random disabled)\n");
        end

        init_data();
        print_input_matrix();
        print_kernels();
        // dump_kernels_to_file("tb_kernels_loaded.memh");  
        calc_expected();

        for (round_idx = 0; round_idx < LOOP_ROUNDS; round_idx = round_idx + 1) begin
            $display("\n[TB][ROUND] ===== round=%0d/%0d =====", round_idx + 1, LOOP_ROUNDS);

            rst_n        = 1'b0;
            in_valid     = 1'b0;
            in_pixel     = {DATA_W{1'b0}};
            out_ready    = 1'b1;
            out_idx      = 0;

            repeat (6) @(posedge clk);
            @(negedge clk); 
            rst_n = 1'b1;

            err_before_round = err_cnt;
            feed_frame();

            cyc = 0;
            while ((out_idx < exp_total_out) && (cyc < WAIT_CYC_MAX)) begin
                @(negedge clk);

                // 随机设置 out_ready
                if (ENABLE_RAND_TEST) begin
                    rand_val = $random % 100;
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

            if (out_idx != exp_total_out) begin
                $display("[TB][FAIL] round=%0d output count mismatch: got=%0d exp=%0d cyc=%0d", round_idx + 1, out_idx, exp_total_out, cyc);
                err_cnt = err_cnt + 1;
                round_fail_cnt = round_fail_cnt + 1;
            end else if (err_cnt != err_before_round) begin
                $display("[TB][FAIL] round=%0d data/flag mismatch exists", round_idx + 1);
                round_fail_cnt = round_fail_cnt + 1;
            end else begin
                $display("[TB][PASS] round=%0d output count/data all matched", round_idx + 1);
            end

            repeat (20) @(posedge clk);
        end

        if (err_cnt == 0) begin
            $display("[TB][PASS] matrix_conv2d_stream check passed. rounds=%0d outputs_per_round=%0d", LOOP_ROUNDS, OUT_SIZE * OUT_CH);
        end else begin
            $display("[TB][FAIL] matrix_conv2d_stream check failed. err_cnt=%0d round_fail_cnt=%0d/%0d", err_cnt, round_fail_cnt, LOOP_ROUNDS);
        end
        
        // 打印随机测试统计信息
        if (ENABLE_RAND_TEST) begin
            $display("\n");
            $display("========================================");
            $display("[TB][RANDOM] Test Statistics");
            $display("========================================");
            $display("[TB][RANDOM] Total input clock cycles: %0d", cyc);
            $display("[TB][RANDOM] in_valid=1 cycles: %0d (%.1f%%)", 
                     in_valid_count, (in_valid_count * 100.0) / (in_valid_count + in_stall_count));
            $display("[TB][RANDOM] in_valid=0 cycles: %0d (%.1f%%)", 
                     in_stall_count, (in_stall_count * 100.0) / (in_valid_count + in_stall_count));
            $display("[TB][RANDOM] out_ready=1 cycles: %0d (%.1f%%)", 
                     out_ready_count, (out_ready_count * 100.0) / (out_ready_count + out_stall_count));
            $display("[TB][RANDOM] out_ready=0 cycles: %0d (%.1f%%)", 
                     out_stall_count, (out_stall_count * 100.0) / (out_ready_count + out_stall_count));
            $display("========================================\n");
        end

        #50;
        $stop;
    end

    always @(posedge clk) begin
        #1;
        if (rst_n && out_valid && out_ready) begin
            if (out_idx >= exp_total_out) begin
                $display("[TB][FAIL] Unexpected extra output: %0d", out_pixel_data);
                err_cnt = err_cnt + 1;
            end else begin
                mismatch = 0;
                exp_ch_idx     = out_idx % OUT_CH;
                exp_sample_idx = out_idx / OUT_CH;

                exp_end_line_flag = ((exp_sample_idx % OUT_W) == (OUT_W - 1) && (exp_ch_idx == (OUT_CH - 1))) ? 1 : 0;
                exp_end_frame_flag = ((exp_sample_idx == (OUT_SIZE - 1)) && (exp_ch_idx == (OUT_CH - 1))) ? 1 : 0;

                got_ch = out_pixel_data;
                exp_ch = exp_mem[exp_ch_idx*OUT_SIZE + exp_sample_idx];
                if (got_ch !== exp_ch) begin
                    $display("[TB][FAIL] out_idx=%0d sample=%0d ch=%0d got=%0d exp=%0d", out_idx, exp_sample_idx, exp_ch_idx, got_ch, exp_ch);
                    err_cnt = err_cnt + 1;
                    mismatch = 1;
                end else begin
                    $display("[TB][OK] out_idx=%0d sample=%0d ch=%0d got=%0d", out_idx, exp_sample_idx, exp_ch_idx, got_ch);
                end

                if (out_pixel_end_line !== exp_end_line_flag[0]) begin
                    $display("[TB][FAIL] out_idx=%0d out_pixel_end_line mismatch: got=%0d exp=%0d", out_idx, out_pixel_end_line, exp_end_line_flag);
                    err_cnt = err_cnt + 1;
                    mismatch = 1;
                end

                if (out_pixel_end_frame !== exp_end_frame_flag[0]) begin
                    $display("[TB][FAIL] out_idx=%0d out_pixel_end_frame mismatch: got=%0d exp=%0d", out_idx, out_pixel_end_frame, exp_end_frame_flag);
                    err_cnt = err_cnt + 1;
                    mismatch = 1;
                end

                if (!mismatch) begin
                    $display("[TB][OK] aligned out_idx=%0d", out_idx);
                end
            end
            out_idx = out_idx + 1;
        end
    end

    task init_data;
    begin
        // ==================== 初始化输入图像 ====================
        for (i = 0; i < IN_SIZE; i = i + 1) begin
            img_mem[i] = i + 1;
        end

        // ==================== 从文件加载卷积核 ====================
        $display("[TB] Loading kernels from %s ...", COEFF_INIT_FILE);
        $readmemh(COEFF_INIT_FILE, ker_mem, 0, COEFF_TOTAL - 1);
        $display("[TB] Kernel loading complete. Loaded %0d coefficients.", COEFF_TOTAL);
    end
    endtask

    task print_input_matrix;
    begin
        $display("[TB] Input matrix (%0dx%0d):", ROW, COL);
        for (oy = 0; oy < ROW; oy = oy + 1) begin
            for (ox = 0; ox < COL; ox = ox + 1) begin
                $write("%5d", img_mem[oy*COL + ox]);
            end
            $write("\n");
        end
    end
    endtask

    task print_kernels;
    begin
        for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
            $display("[TB] Kernel ch=%0d (%0dx%0d):", ch, K, K);
            for (ky = 0; ky < K; ky = ky + 1) begin
                for (kx = 0; kx < K; kx = kx + 1) begin
                    $write("%5d", ker_mem[ch*WIN_SIZE + ky*K + kx]);
                end
                $write("\n");
            end
        end
    end
    endtask

    // ==================== 验证卷积核 ====================
    task dump_kernels_to_file;
    integer fd, idx;
    begin
        fd = $fopen("tb_kernels_loaded.memh", "w");
        if (fd) begin
            $fwrite(fd, "// TB内加载的卷积(来自coeff_init.memh)\n");
            $fwrite(fd, "// 建议? coeff_init.memh 进行比对验证\n\n");
            for (idx = 0; idx < COEFF_TOTAL; idx = idx + 1) begin
                $fwrite(fd, "%02X%s", ker_mem[idx], ((idx + 1) % 5 == 0) ? "\n" : " ");
            end
            $fclose(fd);
            $display("[TB] Kernels dumped to tb_kernels_loaded.memh");
        end else begin
            $display("[TB] Error: Cannot open file tb_kernels_loaded.memh");
        end
    end
    endtask

    task calc_expected;
    begin
        out_idx = 0;
        for (oy = 0; oy < OUT_H; oy = oy + 1) begin
            for (ox = 0; ox < OUT_W; ox = ox + 1) begin
                for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
                    acc = 0;
                    for (ky = 0; ky < K; ky = ky + 1) begin
                        for (kx = 0; kx < K; kx = kx + 1) begin
                            acc = acc + img_mem[(oy*STRIDE + ky)*COL + (ox*STRIDE + kx)]
                                      * ker_mem[ch*WIN_SIZE + ky*K + kx];
                        end
                    end
                    exp_mem[ch*OUT_SIZE + out_idx] = acc;
                end
                out_idx = out_idx + 1;
            end
        end
        out_idx = 0;
    end
    endtask

    task feed_frame;
    begin
        in_idx = 0;
        in_pixel <= img_mem[0];
        while (in_idx < IN_SIZE) begin
            @(negedge clk);
            
            // 随机 in_valid
            if (ENABLE_RAND_TEST) begin
                rand_val = $random % 100;
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
            
            //  in_ready  in_valid 时推进数
            if (in_ready && in_valid) begin     
                in_pixel <= img_mem[in_idx];
                in_idx = in_idx + 1;
            end
        end
            @(negedge clk); // 
            in_valid = 1'b0;
            in_pixel <= {DATA_W{1'b0}};
    end
    endtask

endmodule
