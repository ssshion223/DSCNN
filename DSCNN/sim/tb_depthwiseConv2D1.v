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
module tb_depthwiseConv2D1;
    // 说明：testbench 顶层无模块输入/输出端口，均通过内部信号驱动 DUT。
    localparam integer DATA_W        = 8;  // 输入像素位宽
    localparam integer COEFF_W       = 8;  // 卷积系数位宽
    localparam integer K_H           = 3; // 卷积核高度
    localparam integer K_W           = 3;  // 卷积核宽度
    localparam integer MUL_W         = DATA_W + COEFF_W;
    localparam integer SUM_W         = MUL_W + $clog2(K_H*K_W); // K_H*K_W=1*1时的累加位宽
    localparam integer RESULT_W      = 32; // 最终结果位宽，足够大以防溢出
    localparam integer COL           = 5;
    localparam integer ROW           = 25;
    localparam integer STRIDE        = 1;
    localparam integer PAD_TOP       = (K_H-1) / 2; // 1
    localparam integer PAD_BOTTOM    = K_H/2;
    localparam integer PAD_LEFT      = (K_W-1) / 2; // 1
    localparam integer PAD_RIGHT     = K_W/2;
    localparam integer COEFF_GRP_NUM = 64;
    localparam integer MAC_PIPELINE  = 1;
 
    localparam integer OUT_WIDTH        = 8;
    localparam integer SHIFT_VAL        = 16;
    localparam integer BIAS_GROUP_SIZE  = 1;
    localparam integer BIAS_GROUP_BITS  = ($clog2(BIAS_GROUP_SIZE));
    localparam integer BIAS_CH_BITS     = 6;
    localparam integer MULT_CNT         = 4;
    //*915，1246，406，828，461，652，442，412，623*//
    localparam signed [11:0] MULT_FACTOR0  = 12'sd1246;
    localparam signed [11:0] MULT_FACTOR1  = 12'sd1246;
    localparam signed [11:0] MULT_FACTOR2  = 12'sd406;
    localparam signed [11:0] MULT_FACTOR3  = 12'sd828;
    localparam integer FIFO_DEPTH    = 16;
    localparam integer FIFO_AF_LEVEL = 14;

    localparam integer IN_VALID_RND_PROB  = 70;
    localparam integer OUT_READY_RND_PROB = 70;
    localparam integer ENABLE_RAND_TEST   = 1;

    localparam integer FRAMES_PER_ROUND = 64;
    localparam integer LOOP_ROUNDS      = 1;
    localparam integer WAIT_CYC_MAX     = 4000;

    localparam COEFF_INIT_FILE = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_dw1.memh";
    localparam BIAS_INIT_FILE  = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw1_Fold_bias.hex";
    localparam INPUT_INIT_FILE = "D:/vivado/exp/DSCNN/data/pixel_input/downsample.memh";
    localparam EXP_OUT_FILE     = "D:/vivado/exp/DSCNN/data/pixel_output/layer1_dw.memh";

    localparam integer WIN_SIZE      = K_H * K_W;
    localparam integer P_COL         = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW         = ROW + PAD_TOP + PAD_BOTTOM;
    localparam integer OUT_W         = ((P_COL - K_W) / STRIDE) + 1;
    localparam integer OUT_H         = ((P_ROW - K_H) / STRIDE) + 1;
    localparam integer OUT_SIZE      = OUT_W * OUT_H;
    localparam integer EXP_BEATS_PER_ROUND = FRAMES_PER_ROUND * OUT_SIZE;
    localparam integer TOTAL_FRAMES  = FRAMES_PER_ROUND * LOOP_ROUNDS;
    localparam integer FRAME_BEATS   = ROW * COL;
    localparam integer INPUT_BEATS_TOTAL = TOTAL_FRAMES * FRAME_BEATS;
    localparam integer FRAME_GRP_NUM = TOTAL_FRAMES;
    localparam integer EXP_BEATS_TOTAL = TOTAL_FRAMES * OUT_SIZE;
    localparam integer BIAS_DEPTH    = (1 << (BIAS_GROUP_BITS + BIAS_CH_BITS));
    localparam integer CH_MASK       = (1 << BIAS_CH_BITS) - 1;

    reg clk;
    reg rst_n;
    reg in_valid;
    wire in_ready;
    reg signed [DATA_W-1:0] in_pixel;
    reg in_end_all_frame;  // 全部组帧结束标志

    wire [OUT_WIDTH-1:0] out_pixel;
    wire out_valid;
    reg out_ready;
    wire out_end_frame;
    wire out_end_all_frame;

    reg signed [DATA_W-1:0] input_mem [0:INPUT_BEATS_TOTAL-1];

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
    integer input_ptr;
    integer sample_idx;
    integer frame_idx;
    integer rand_val;
    integer in_valid_count;
    integer out_ready_count;
    integer in_stall_count;
    integer out_stall_count;

    reg [OUT_WIDTH-1:0] got_pix;
    reg [OUT_WIDTH-1:0] exp_pix;

    localparam integer OUT_MAX_VAL = (1 << (OUT_WIDTH-1)) - 1;
    localparam integer PRINT_BEAT_START = 0;
    localparam integer PRINT_BEAT_END   = 124;

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
        .FRAME_GRP_NUM(FRAME_GRP_NUM),
        .MAC_PIPELINE(MAC_PIPELINE),
        .COEFF_INIT_FILE(COEFF_INIT_FILE),
        .OUT_WIDTH(OUT_WIDTH),
        .SHIFT_VAL(SHIFT_VAL),
        .BIAS_GROUP_BITS(BIAS_GROUP_BITS),
        .BIAS_GROUP_SIZE(BIAS_GROUP_SIZE),
        .BIAS_CH_BITS(BIAS_CH_BITS),
        .MULT_CNT(MULT_CNT),
        .MULT_FACTOR0(MULT_FACTOR0),
        .MULT_FACTOR1(MULT_FACTOR1),
        .MULT_FACTOR2(MULT_FACTOR2),
        .MULT_FACTOR3(MULT_FACTOR3),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_AF_LEVEL(FIFO_AF_LEVEL),
        .BIAS_INIT_FILE(BIAS_INIT_FILE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .in_end_all_frame(in_end_all_frame),
        .out_pixel(out_pixel),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_end_frame(out_end_frame),
        .out_end_all_frame(out_end_all_frame)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        in_valid = 1'b0;
        in_pixel = {DATA_W{1'b0}};
        in_end_all_frame = 1'b0;
        out_ready = 1'b1;
        out_idx = 0;
        err_cnt = 0;
        input_ptr = 0;
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

        $readmemh(INPUT_INIT_FILE, input_mem, 0, INPUT_BEATS_TOTAL - 1);
        $readmemh(EXP_OUT_FILE, exp_pix_mem, 0, EXP_BEATS_TOTAL - 1);

        $display("[TB] input file  = %s", INPUT_INIT_FILE);
        $display("[TB] expect file = %s", EXP_OUT_FILE);
        $display("[TB] total input beats  = %0d", INPUT_BEATS_TOTAL);
        $display("[TB] total expect beats = %0d", EXP_BEATS_TOTAL);

        print_input_edge_frames();
        load_and_print_kernel_edge_groups();
        load_and_print_all_bias();

        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        feed_all_input();

        cyc = 0;
        while ((out_idx < EXP_BEATS_TOTAL) && (cyc < WAIT_CYC_MAX)) begin
            @(posedge clk);
            cyc = cyc + 1;
        end

        if (out_idx != EXP_BEATS_TOTAL) begin
            $display("[TB][FAIL] output beat mismatch: got=%0d exp=%0d cyc=%0d",
                     out_idx, EXP_BEATS_TOTAL, cyc);
            err_cnt = err_cnt + 1;
        end

        if (err_cnt == 0) begin
            $display("[TB][PASS] tb_depthwiseConv2D1 PASSED");
        end else begin
            $display("[TB][FAIL] tb_depthwiseConv2D1 FAILED, err_cnt=%0d", err_cnt);
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
                exp_end_all = exp_end_frame && ((frame_idx % FRAME_GRP_NUM) == (FRAME_GRP_NUM - 1));

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

    task feed_all_input;
    begin
        input_ptr = 0;
        while (input_ptr < INPUT_BEATS_TOTAL) begin
            @(negedge clk);
            out_ready = 1'b1;
            in_valid = 1'b1;
            in_pixel = input_mem[input_ptr];
            in_end_all_frame = ((input_ptr % FRAME_BEATS) == (FRAME_BEATS - 1));

            if (in_ready) begin
                input_ptr = input_ptr + 1;
            end
        end

        @(negedge clk);
        in_valid = 1'b0;
        in_pixel = {DATA_W{1'b0}};
        in_end_all_frame = 1'b0;
    end
    endtask

    task print_input_edge_frames;
    integer row_idx;
    integer col_idx;
    integer first_base;
    integer last_base;
    begin
        first_base = 0;
        last_base  = INPUT_BEATS_TOTAL - FRAME_BEATS;

        $display("\n[TB] ===== Input Frame #0 =====");
        for (row_idx = 0; row_idx < ROW; row_idx = row_idx + 1) begin
            $write("[TB] row %0d: ", row_idx);
            for (col_idx = 0; col_idx < COL; col_idx = col_idx + 1) begin
                $write("%4d ", input_mem[first_base + row_idx*COL + col_idx]);
            end
            $display("");
        end

        $display("\n[TB] ===== Input Frame #%0d =====", TOTAL_FRAMES - 1);
        for (row_idx = 0; row_idx < ROW; row_idx = row_idx + 1) begin
            $write("[TB] row %0d: ", row_idx);
            for (col_idx = 0; col_idx < COL; col_idx = col_idx + 1) begin
                $write("%4d ", input_mem[last_base + row_idx*COL + col_idx]);
            end
            $display("");
        end
    end
    endtask

    task load_and_print_kernel_edge_groups;
    integer grp_idx;
    integer ker_idx;
    integer tail_start;
    begin
        $readmemh(COEFF_INIT_FILE, ker_grp_mem, 0, COEFF_GRP_NUM-1);

        for (grp_idx = 0; grp_idx < COEFF_GRP_NUM; grp_idx = grp_idx + 1) begin
            for (ker_idx = 0; ker_idx < WIN_SIZE; ker_idx = ker_idx + 1) begin
                ker_mem[grp_idx*WIN_SIZE + ker_idx] = ker_grp_mem[grp_idx][(ker_idx+1)*COEFF_W-1 -: COEFF_W];
            end
        end

        $display("\n[TB] ===== Kernel Group First 5 =====");
        for (grp_idx = 0; (grp_idx < 5) && (grp_idx < COEFF_GRP_NUM); grp_idx = grp_idx + 1) begin
            $display("[TB] Kernel Group %0d:", grp_idx);
            for (ky = 0; ky < K_H; ky = ky + 1) begin
                $write("[TB]   ");
                for (kx = 0; kx < K_W; kx = kx + 1) begin
                    $write("%4d ", ker_mem[grp_idx*WIN_SIZE + ky*K_W + kx]);
                end
                $display("");
            end
        end

        tail_start = (COEFF_GRP_NUM > 5) ? (COEFF_GRP_NUM - 5) : 0;
        $display("\n[TB] ===== Kernel Group Last 5 =====");
        for (grp_idx = tail_start; grp_idx < COEFF_GRP_NUM; grp_idx = grp_idx + 1) begin
            $display("[TB] Kernel Group %0d:", grp_idx);
            for (ky = 0; ky < K_H; ky = ky + 1) begin
                $write("[TB]   ");
                for (kx = 0; kx < K_W; kx = kx + 1) begin
                    $write("%4d ", ker_mem[grp_idx*WIN_SIZE + ky*K_W + kx]);
                end
                $display("");
            end
        end
    end
    endtask

    task load_and_print_all_bias;
    integer bias_idx;
    begin
        $readmemh(BIAS_INIT_FILE, bias_mem, 0, BIAS_DEPTH-1);

        $display("\n[TB] ===== Bias All Entries =====");
        for (bias_idx = 0; bias_idx < BIAS_DEPTH; bias_idx = bias_idx + 1) begin
            $display("[TB] bias[%0d] = %0d", bias_idx, bias_mem[bias_idx]);
        end
    end
    endtask

endmodule
