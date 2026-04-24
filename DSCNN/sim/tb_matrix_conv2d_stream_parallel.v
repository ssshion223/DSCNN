`timescale 1ns/1ns

//==============================================================================
// 模块名：tb_matrix_conv2d_stream_parallel
// 功能说明：
//   支持 64 帧连续输入的卷积核心自校验测试平台。
//   - 自动加载 64 帧图像数据并进行全量校验。
//   - 仅打印首位两帧的矩阵内容以保持控制台整洁。
//==============================================================================

module tb_matrix_conv2d_stream_parallel;
    // ==================== 基础参数 ====================
    localparam integer DATA_W  = 8;                         // 输入像素位宽 [cite: 64]
    localparam integer COEFF_W = 8;                         // 卷积系数位宽 [cite: 65]
    localparam integer COL     = 5;                         // 输入列数 [cite: 66]
    localparam integer ROW     = 25;                        // 输入行数 [cite: 67]
    localparam integer K_H     = 1;                         // 卷积核高度 [cite: 68]
    localparam integer K_W     = 1;                         // 卷积核宽度 [cite: 69]
    localparam integer STRIDE  = 1;                         // 步长 [cite: 70]
    
    localparam integer MUL_W   = DATA_W + COEFF_W;          // 乘积位宽 [cite: 70]
    localparam integer SUM_W   = MUL_W + $clog2(K_H*K_W);   // 累加位宽 [cite: 70]
    
    localparam integer PAD_TOP    = 0; 
    localparam integer PAD_BOTTOM = 0; 
    localparam integer PAD_LEFT   = 0; 
    localparam integer PAD_RIGHT  = 0; 
    
    localparam integer OUT_CH  = 64;                        // 并行输出通道数 [cite: 73]
    localparam integer COEFF_GRP_NUM = 64;                  // 卷积核组数 [cite: 73]
    localparam integer MAC_PIPELINE  = 1;                   // 加法树流水线级数 [cite: 73]
    
    // ==================== 数据文件路径 ====================
    localparam COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pw1.memh"; 
    localparam INPUT_INIT_FILE = "D:\\vivado\\exp\\DSCNN\\data\\pixel_output\\layer1_dw.memh"; 

    // ==================== 测试控制参数 ====================
    localparam integer FRAMES_PER_ROUND = 64;               // 总帧数：64 [cite: 76]
    localparam integer LOOP_ROUNDS      = 1;
    localparam integer WAIT_CYC_MAX     = 50000;            // 增加超时等待上限
    localparam integer ENABLE_RAND_TEST = 1;                // 开启随机反压测试 [cite: 76]
    localparam integer TARGET_PRINT_FRAME = 1;              // 设置想要打印观测的输入帧索引 (0 ~ FRAMES_PER_ROUND-1)
    
    // ==================== 内部计算常量 ====================
    localparam integer WIN_SIZE      = K_H * K_W;
    localparam integer ROM_OUT_CH    = OUT_CH * COEFF_GRP_NUM;
    localparam integer GROUP_SIZE    = OUT_CH * WIN_SIZE;
    localparam integer GRP_COEFF_BUS_W = GROUP_SIZE * COEFF_W;
    localparam integer P_COL         = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW         = ROW + PAD_TOP + PAD_BOTTOM; 
    localparam integer OUT_W         = ((P_COL - K_W) / STRIDE) + 1; 
    localparam integer OUT_H         = ((P_ROW - K_H) / STRIDE) + 1; 
    localparam integer OUT_SIZE      = OUT_W * OUT_H;
    localparam integer EXP_BEATS_TOTAL = FRAMES_PER_ROUND * OUT_SIZE; // 总预期拍数

    // ==================== 信号定义 ====================
    reg clk;
    reg rst_n;
    reg in_valid;
    wire in_ready;
    reg signed [DATA_W-1:0] in_pixel;
    reg in_end_all_frame;

    wire out_valid;
    reg out_ready;
    wire out_end_all_frame;
    wire out_end_frame;
    wire signed [OUT_CH*SUM_W-1:0] out_pixel_data_bus;

    // ==================== 存储器定义 (覆盖64帧) ====================
    reg signed [DATA_W-1:0] img_mem [0:FRAMES_PER_ROUND*ROW*COL-1];
    reg signed [DATA_W-1:0] padded_mem [0:FRAMES_PER_ROUND*P_ROW*P_COL-1];
    reg [GRP_COEFF_BUS_W-1:0] ker_grp_mem [0:COEFF_GRP_NUM-1];
    reg signed [COEFF_W-1:0] ker_mem [0:ROM_OUT_CH*WIN_SIZE-1];
    reg signed [SUM_W-1:0] exp_mem [0:EXP_BEATS_TOTAL*OUT_CH-1];

    integer i, r, c, ky, kx, f, ch, grp, out_idx, err_cnt, cyc;
    reg signed [SUM_W-1:0] got_ch, exp_ch;

    // ==================== 实例化 DUT ====================
    matrix_conv2d_stream_parallel #(
        .DATA_W(DATA_W), .COEFF_W(COEFF_W), .COL(COL), .ROW(ROW),
        .K_H(K_H), .K_W(K_W), .STRIDE(STRIDE), .OUT_CH(OUT_CH),
        .COEFF_GRP_NUM(COEFF_GRP_NUM), .FRAME_GRP_NUM(FRAMES_PER_ROUND),
        .COEFF_INIT_FILE(COEFF_INIT_FILE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_ready(in_ready), .in_pixel(in_pixel),
        .in_end_all_frame(in_end_all_frame),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_end_all_frame(out_end_all_frame), .out_end_frame(out_end_frame),
        .out_pixel_data_bus(out_pixel_data_bus)
    );

    // 时钟生成
    always #5 clk = ~clk; 

    // ==================== 测试主流程 ====================
    initial begin
        // 初始化信号
        clk = 1'b0; rst_n = 1'b0; in_valid = 1'b0; out_ready = 1'b1;
        err_cnt = 0; out_idx = 0;
        
        $display("[TB] Start: 64 Frames Simulation");
        
        init_input();       // 加载 64 帧数据 [cite: 131]
        build_padded();     // 补零处理 [cite: 135]
        load_coeff();       // 加载权重 [cite: 154]
        print_input_matrix(); // 打印首末帧 [cite: 138]
        build_expected();   // 计算黄金模型 [cite: 158]

        rst_n = 1'b0; repeat(10) @(posedge clk); rst_n = 1'b1;

        // 喂入 64 帧
        for (f = 0; f < FRAMES_PER_ROUND; f = f + 1) begin
            feed_one_frame(f, f == FRAMES_PER_ROUND - 1);
            repeat(5) @(posedge clk);
        end

        // 等待所有输出完成
        cyc = 0;
        while ((out_idx < EXP_BEATS_TOTAL) && (cyc < WAIT_CYC_MAX)) begin
            @(negedge clk);
            if (ENABLE_RAND_TEST) out_ready = ($random % 100 < 70);
            cyc = cyc + 1;
        end

        // 结果判定
        if (err_cnt == 0 && out_idx == EXP_BEATS_TOTAL)
            $display("\n[TB] ALL PASSED: Checked %0d frames.", FRAMES_PER_ROUND);
        else
            $display("\n[TB] FAILED: err_cnt=%0d, out_idx=%0d/%0d", err_cnt, out_idx, EXP_BEATS_TOTAL);

        #100; $stop;
    end

    // ==================== 自动比对逻辑 ====================
    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            if (out_idx < EXP_BEATS_TOTAL) begin
                for (ch = 0; ch < OUT_CH; ch = ch + 1) begin
                    got_ch = out_pixel_data_bus[ch*SUM_W +: SUM_W];
                    exp_ch = exp_mem[out_idx*OUT_CH + ch];
                    if (got_ch !== exp_ch) begin
                        $display("[TB][FAIL] Beat=%0d Ch=%0d Got=%0d Exp=%0d", out_idx, ch, got_ch, exp_ch);
                        err_cnt = err_cnt + 1;
                    end
                end
                out_idx = out_idx + 1;
            end
        end
    end

    // ==================== 辅助任务 (Tasks) ====================

    task init_input;
    begin
        $display("[TB] Loading input data from %s", INPUT_INIT_FILE);
        $readmemh(INPUT_INIT_FILE, img_mem);
        for (i = 0; i < FRAMES_PER_ROUND*ROW*COL; i = i + 1) img_mem[i] = $signed(img_mem[i]);
    end
    endtask

    task build_padded;
    integer f_idx, r_idx, c_idx;
    begin
        for (f_idx = 0; f_idx < FRAMES_PER_ROUND; f_idx = f_idx + 1) begin
            for (r_idx = 0; r_idx < ROW; r_idx = r_idx + 1) begin
                for (c_idx = 0; c_idx < COL; c_idx = c_idx + 1) begin
                    padded_mem[f_idx*P_ROW*P_COL + (r_idx+PAD_TOP)*P_COL + (c_idx+PAD_LEFT)] = 
                        img_mem[f_idx*ROW*COL + r_idx*COL + c_idx];
                end
            end
        end
    end
    endtask

    task print_input_matrix;
    begin
        // 检查设置的帧索引是否在合法范围内
        if (TARGET_PRINT_FRAME >= 0 && TARGET_PRINT_FRAME < FRAMES_PER_ROUND) begin
            $display("\n[TB] Printing Frame %0d (%0dx%0d):", TARGET_PRINT_FRAME, ROW, COL);
            for (r = 0; r < ROW; r = r + 1) begin
                $write("Row %2d: ", r);
                for (c = 0; c < COL; c = c + 1) begin
                    // 根据参数计算对应的显存偏移地址
                    $write("%3d ", img_mem[TARGET_PRINT_FRAME*ROW*COL + r*COL + c]);
                end
                $display("");
            end
        end else begin
            $display("\n[TB] Warning: TARGET_PRINT_FRAME (%0d) is out of bounds (0-%0d).", 
                     TARGET_PRINT_FRAME, FRAMES_PER_ROUND-1);
        end
    end
    endtask

    task load_coeff;
    integer g, ch_i, k_i;
    begin
        $readmemh(COEFF_INIT_FILE, ker_grp_mem);
        for (g = 0; g < COEFF_GRP_NUM; g = g + 1) begin
            for (ch_i = 0; ch_i < OUT_CH; ch_i = ch_i + 1) begin
                for (k_i = 0; k_i < WIN_SIZE; k_i = k_i + 1) begin
                    ker_mem[(g*OUT_CH + ch_i)*WIN_SIZE + k_i] = 
                        ker_grp_mem[g][((ch_i*WIN_SIZE + k_i)+1)*COEFF_W-1 -: COEFF_W];
                end
            end
        end
    end
    endtask

    task build_expected;
    integer f_i, oy, ox, c_i, ker_ch, acc;
    integer e_idx;
    begin
        e_idx = 0;
        for (f_i = 0; f_i < FRAMES_PER_ROUND; f_i = f_i + 1) begin
            grp = f_i % COEFF_GRP_NUM;
            for (oy = 0; oy < OUT_H; oy = oy + 1) begin
                for (ox = 0; ox < OUT_W; ox = ox + 1) begin
                    for (c_i = 0; c_i < OUT_CH; c_i = c_i + 1) begin
                        ker_ch = grp * OUT_CH + c_i;
                        acc = 0;
                        for (ky = 0; ky < K_H; ky = ky + 1) begin
                            for (kx = 0; kx < K_W; kx = kx + 1) begin
                                acc = acc + padded_mem[f_i*P_ROW*P_COL + (oy*STRIDE+ky)*P_COL + (ox*STRIDE+kx)] * ker_mem[ker_ch*WIN_SIZE + ky*K_W + kx];
                            end
                        end
                        exp_mem[e_idx*OUT_CH + c_i] = acc;
                    end
                    e_idx = e_idx + 1;
                end
            end
        end
    end
    endtask

    task feed_one_frame;
    input integer cur_f;
    input is_last_all;
    integer in_f_idx;
    begin
        in_f_idx = 0;
        while (in_f_idx < ROW*COL) begin
            @(negedge clk);
            if (ENABLE_RAND_TEST) in_valid = ($random % 100 < 70); else in_valid = 1'b1;
            if (in_valid && in_ready) begin
                in_pixel = img_mem[cur_f*ROW*COL + in_f_idx];
                in_end_all_frame = is_last_all && (in_f_idx == ROW*COL - 1);
                in_f_idx = in_f_idx + 1;
            end else if (!in_ready) begin
                // 如果被反压，保持当前像素和有效信号
            end
        end
        @(negedge clk); in_valid = 1'b0; in_end_all_frame = 1'b0;
    end
    endtask

endmodule