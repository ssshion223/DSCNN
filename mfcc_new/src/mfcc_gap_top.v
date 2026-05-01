`include "mfcc_defs.vh"

// GAP 风格 MFCC 主链路顶层控制模块。
// 数据流向：
// 输入流 -> 帧缓存 -> Hann 窗 -> 512 点 FFT
// -> 40 路 Mel -> ln() -> 40 点 DCT -> 系数输出
module mfcc_gap_top (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           sample_valid,
    input  wire                           sample_first,
    input  wire [15:0]                    sample_data,
    input  wire                           coeff_ready,
    output wire                           busy,
    output reg                            coeff_valid,
    output reg  [`MFCC_CEP_IDX_W-1:0]     coeff_index,
    output reg  [`MFCC_DCT_OUT_W-1:0]     coeff_data,
    output reg                            frame_done
);

    localparam FRAME_LAST  = `MFCC_FRAME_LEN - 1;
    localparam OVERLAP_LEN = `MFCC_FRAME_LEN - `MFCC_FRAME_SHIFT;

    // 先积满一帧，跑完整条 MFCC 链路，输出 40 个系数，
    // 再复制重叠区，为下一帧做准备。
    localparam S_CAPTURE       = 5'd0;
    localparam S_FFT_START     = 5'd1;
    localparam S_FFT_READ_REQ  = 5'd2;
    localparam S_FFT_WIN_ISSUE = 5'd3;
    localparam S_FFT_WIN_WAIT  = 5'd4;
    localparam S_FFT_SEND      = 5'd5;
    localparam S_FFT_PAD       = 5'd6;
    localparam S_FFT_WAIT_DONE = 5'd7;
    localparam S_MEL_START     = 5'd8;
    localparam S_MEL_WAIT      = 5'd9;
    localparam S_LOG_ISSUE     = 5'd10;
    localparam S_LOG_WAIT      = 5'd11;
    localparam S_DCT_START     = 5'd12;
    localparam S_DCT_WAIT      = 5'd13;
    localparam S_OUTPUT        = 5'd14;
    localparam S_COPY_INIT     = 5'd15;
    localparam S_COPY_ISSUE    = 5'd16;
    localparam S_COPY_WRITE    = 5'd17;

    reg [4:0] state;

    reg [`MFCC_POWER_ADDR_W-1:0] capture_req_idx;
    reg [`MFCC_POWER_ADDR_W-1:0] capture_wr_idx;
    reg [`MFCC_POWER_ADDR_W-1:0] fft_in_idx;
    reg [`MFCC_POWER_ADDR_W-1:0] copy_idx;
    reg [`MFCC_FILTER_IDX_W-1:0] filter_idx;
    reg [`MFCC_CEP_IDX_W-1:0]    cep_idx;
    reg [15:0]                   fft_send_sample_r;
    reg                          power_frame_done;
    reg                          fft_frame_done;

    wire        fifo_full;
    wire        fifo_empty;
    wire [9:0]  fifo_data_count;
    wire        fifo_out_valid;
    wire        fifo_out_first;
    wire [15:0] fifo_out_sample;
    wire        fifo_out_ready;
    wire        fifo_rd_fire;

    reg                           frame_wr_en;
    reg  [`MFCC_POWER_ADDR_W-1:0] frame_wr_addr;
    reg  [15:0]                   frame_wr_data;
    reg                           frame_rd_en;
    reg  [`MFCC_POWER_ADDR_W-1:0] frame_rd_addr;
    wire [15:0]                   frame_rd_data;

    wire        win_in_valid;
    wire [15:0] win_in_sample;
    wire [15:0] win_coeff_q15;
    wire        win_out_valid;
    wire [15:0] win_out_sample;

    wire                          fft_start;
    wire                          fft_in_valid;
    wire [15:0]                   fft_in_re;
    wire [15:0]                   fft_in_im;
    wire                          fft_in_last;
    wire                          fft_in_ready;
    wire                          fft_done;
    wire                          fft_out_valid;
    wire [`MFCC_POWER_ADDR_W-1:0] fft_out_index;
    wire [`MFCC_FFT_W-1:0]        fft_out_re;
    wire [`MFCC_FFT_W-1:0]        fft_out_im;

    wire                           mel_load_valid;
    wire [`MFCC_POWER_ADDR_W-1:0]  mel_load_index;
    wire [`MFCC_FFT_W-1:0]         mel_load_re;
    wire [`MFCC_FFT_W-1:0]         mel_load_im;
    wire                           mel_start;
    wire                           mel_done;
    wire [31:0]                    mel_rd_data;

    wire                           ln_in_valid;
    wire [31:0]                    ln_in_value;
    wire                           ln_out_valid;
    wire [15:0]                    ln_value;

    wire                           dct_load_valid;
    wire [`MFCC_FILTER_IDX_W-1:0]  dct_load_index;
    wire [15:0]                    dct_load_data;
    wire                           dct_start;
    wire                           dct_done;
    wire [`MFCC_DCT_OUT_W-1:0]     dct_rd_coeff;

    // 只有捕获状态表示当前允许继续积累输入样本。
    assign busy = (state != S_CAPTURE);

    // 只有当前帧还没装满时，才继续从 FIFO 取样。
    assign fifo_out_ready = (state == S_CAPTURE) && (capture_req_idx <= FRAME_LAST);
    assign fifo_rd_fire   = fifo_out_ready && !fifo_empty;

    // 由于帧 RAM 是同步读，所以加窗输入要比读请求晚一拍发出。
    assign win_in_valid   = (state == S_FFT_WIN_ISSUE);
    assign win_in_sample  = frame_rd_data;

    // FFT 输入要么是真实窗后样本，要么是补零样本。
    assign fft_start      = (state == S_FFT_START);
    assign fft_in_valid   = (state == S_FFT_SEND) || (state == S_FFT_PAD);
    assign fft_in_re      = (state == S_FFT_SEND) ? fft_send_sample_r : 16'd0;
    assign fft_in_im      = 16'd0;
    assign fft_in_last    = (((state == S_FFT_SEND) || (state == S_FFT_PAD)) &&
                             (fft_in_idx == (`MFCC_FFT_LEN - 1)));

    // 只把实数 FFT 的前半谱装入 Mel，后半谱是镜像冗余。
    assign mel_load_valid = fft_out_valid && (fft_out_index < `MFCC_POWER_BINS);
    assign mel_load_index = fft_out_index;
    assign mel_load_re    = fft_out_re;
    assign mel_load_im    = fft_out_im;
    assign mel_start      = (state == S_MEL_START);

    assign ln_in_valid    = (state == S_LOG_ISSUE);
    assign ln_in_value    = mel_rd_data;

    assign dct_load_valid = (state == S_LOG_WAIT) && ln_out_valid;
    assign dct_load_index = filter_idx;
    assign dct_load_data  = ln_value;
    assign dct_start      = (state == S_DCT_START);

    mfcc_input_fifo u_input_fifo (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (sample_valid),
        .in_first  (sample_first),
        .in_sample (sample_data),
        .full      (fifo_full),
        .data_count(fifo_data_count),
        .out_ready (fifo_out_ready),
        .out_valid (fifo_out_valid),
        .out_first (fifo_out_first),
        .out_sample(fifo_out_sample),
        .empty     (fifo_empty)
    );

    mfcc_ram_sdp #(
        .DATA_W(16),
        .ADDR_W(`MFCC_POWER_ADDR_W),
        .DEPTH (`MFCC_FFT_LEN)
    ) u_frame_mem (
        .clk    (clk),
        .rst_n  (rst_n),
        .wr_en  (frame_wr_en),
        .wr_addr(frame_wr_addr),
        .wr_data(frame_wr_data),
        .rd_en  (frame_rd_en),
        .rd_addr(frame_rd_addr),
        .rd_data(frame_rd_data)
    );

    mfcc_gap_window_rom u_window_rom (
        .addr      (fft_in_idx),
        .coeff_q15 (win_coeff_q15)
    );

    mfcc_window_mult u_window_mult (
        .clk       (clk),
        .rst_n     (rst_n),
        .in_valid  (win_in_valid),
        .sample_in (win_in_sample),
        .window_q15(win_coeff_q15),
        .out_valid (win_out_valid),
        .sample_out(win_out_sample)
    );

    mfcc_fft512_intfftk u_fft (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (fft_start),
        .in_valid (fft_in_valid),
        .in_re    (fft_in_re),
        .in_im    (fft_in_im),
        .in_last  (fft_in_last),
        .in_ready (fft_in_ready),
        .busy     (),
        .done     (fft_done),
        .out_valid(fft_out_valid),
        .out_index(fft_out_index),
        .out_re   (fft_out_re),
        .out_im   (fft_out_im),
        .out_last ()
    );

    mfcc_gap_mel_filterbank u_mel (
        .clk       (clk),
        .rst_n     (rst_n),
        .load_valid(mel_load_valid),
        .load_index(mel_load_index),
        .load_re   (mel_load_re),
        .load_im   (mel_load_im),
        .start     (mel_start),
        .busy      (),
        .done      (mel_done),
        .rd_index  (filter_idx),
        .rd_data   (mel_rd_data)
    );

    mfcc_gap_ln_approx u_ln (
        .clk      (clk),
        .rst_n    (rst_n),
        .in_valid (ln_in_valid),
        .value_in (ln_in_value),
        .out_valid(ln_out_valid),
        .ln_q10   (ln_value)
    );

    mfcc_gap_dct40 u_dct (
        .clk       (clk),
        .rst_n     (rst_n),
        .load_valid(dct_load_valid),
        .load_index(dct_load_index),
        .load_data (dct_load_data),
        .start     (dct_start),
        .busy      (),
        .done      (dct_done),
        .rd_index  (cep_idx),
        .rd_data   (dct_rd_coeff)
    );

    // 帧 RAM 同时承担收样写入、FFT 读出、重叠复制三类访问。
    always @(*) begin
        frame_wr_en   = 1'b0;
        frame_wr_addr = {`MFCC_POWER_ADDR_W{1'b0}};
        frame_wr_data = 16'd0;
        frame_rd_en   = 1'b0;
        frame_rd_addr = {`MFCC_POWER_ADDR_W{1'b0}};

        if ((state == S_CAPTURE) && fifo_out_valid) begin
            frame_wr_en   = 1'b1;
            frame_wr_addr = capture_wr_idx;
            frame_wr_data = fifo_out_sample;
        end

        if (state == S_FFT_READ_REQ) begin
            frame_rd_en   = 1'b1;
            frame_rd_addr = fft_in_idx;
        end

        if (state == S_COPY_ISSUE) begin
            frame_rd_en   = 1'b1;
            frame_rd_addr = copy_idx + `MFCC_FRAME_SHIFT;
        end

        if (state == S_COPY_WRITE) begin
            frame_wr_en   = 1'b1;
            frame_wr_addr = copy_idx;
            frame_wr_data = frame_rd_data;
        end
    end

    // 当 coeff_ready 拉低时，输出系数保持不变。
    always @(*) begin
        coeff_valid = 1'b0;
        coeff_index = cep_idx;
        coeff_data  = dct_rd_coeff;

        if (state == S_OUTPUT) begin
            coeff_valid = 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_CAPTURE;
            capture_req_idx  <= {`MFCC_POWER_ADDR_W{1'b0}};
            capture_wr_idx   <= {`MFCC_POWER_ADDR_W{1'b0}};
            fft_in_idx       <= {`MFCC_POWER_ADDR_W{1'b0}};
            copy_idx         <= {`MFCC_POWER_ADDR_W{1'b0}};
            filter_idx       <= {`MFCC_FILTER_IDX_W{1'b0}};
            cep_idx          <= {`MFCC_CEP_IDX_W{1'b0}};
            fft_send_sample_r<= 16'd0;
            power_frame_done <= 1'b0;
            fft_frame_done   <= 1'b0;
            frame_done       <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            // 记录独立半谱 0~256 是否已经全部装入 Mel 输入 RAM。
            if (fft_out_valid && (fft_out_index == (`MFCC_POWER_BINS - 1))) begin
                power_frame_done <= 1'b1;
            end
            if (fft_done) begin
                fft_frame_done <= 1'b1;
            end

            case (state)
                S_CAPTURE: begin
                    // 从 FIFO 申请新样本，并写入当前帧缓存。
                    if (fifo_rd_fire) begin
                        capture_req_idx <= capture_req_idx + 1'b1;
                    end

                    if (fifo_out_valid) begin
                        if (capture_wr_idx == FRAME_LAST) begin
                            capture_wr_idx   <= capture_wr_idx + 1'b1;
                            fft_in_idx       <= {`MFCC_POWER_ADDR_W{1'b0}};
                            power_frame_done <= 1'b0;
                            fft_frame_done   <= 1'b0;
                            state            <= S_FFT_START;
                        end else begin
                            capture_wr_idx <= capture_wr_idx + 1'b1;
                        end
                    end
                end

                S_FFT_START: begin
                    // 给 FFT 包装层打一拍 start。
                    state <= S_FFT_READ_REQ;
                end

                S_FFT_READ_REQ: begin
                    // 对帧 RAM 发起下一点的同步读请求。
                    state <= S_FFT_WIN_ISSUE;
                end

                S_FFT_WIN_ISSUE: begin
                    // 把帧 RAM 读出的样本送进加窗模块。
                    state <= S_FFT_WIN_WAIT;
                end

                S_FFT_WIN_WAIT: begin
                    // 等待加窗流水线真正吐出一个有效样本。
                    if (win_out_valid) begin
                        fft_send_sample_r <= win_out_sample;
                        state             <= S_FFT_SEND;
                    end
                end

                S_FFT_SEND: begin
                    // 只有 FFT 真正握手接收后，才推进输入点号。
                    if (fft_in_ready) begin
                        if (fft_in_idx == FRAME_LAST) begin
                            fft_in_idx <= `MFCC_FRAME_LEN;
                            state      <= S_FFT_PAD;
                        end else begin
                            fft_in_idx <= fft_in_idx + 1'b1;
                            state      <= S_FFT_READ_REQ;
                        end
                    end
                end

                S_FFT_PAD: begin
                    // 把 400 点帧补零到 512 点 FFT 长度。
                    if (fft_in_ready) begin
                        if (fft_in_idx == (`MFCC_FFT_LEN - 1)) begin
                            state <= S_FFT_WAIT_DONE;
                        end else begin
                            fft_in_idx <= fft_in_idx + 1'b1;
                        end
                    end
                end

                S_FFT_WAIT_DONE: begin
                    // 同时等待 FFT 任务结束、Mel 装谱结束。
                    if (fft_frame_done && power_frame_done) begin
                        filter_idx <= {`MFCC_FILTER_IDX_W{1'b0}};
                        state      <= S_MEL_START;
                    end
                end

                S_MEL_START: begin
                    // 启动任务式 Mel 模块。
                    state <= S_MEL_WAIT;
                end

                S_MEL_WAIT: begin
                    // Mel 结果先存在内部存储里，done 后再逐路读取。
                    if (mel_done) begin
                        filter_idx <= {`MFCC_FILTER_IDX_W{1'b0}};
                        state      <= S_LOG_ISSUE;
                    end
                end

                S_LOG_ISSUE: begin
                    // 每次向 ln 流水线送一路 Mel 输出。
                    state <= S_LOG_WAIT;
                end

                S_LOG_WAIT: begin
                    // 把每一路 ln 输出装入 DCT 输入缓存。
                    if (ln_out_valid) begin
                        if (filter_idx == (`MFCC_NUM_FILTERS - 1)) begin
                            state <= S_DCT_START;
                        end else begin
                            filter_idx <= filter_idx + 1'b1;
                            state      <= S_LOG_ISSUE;
                        end
                    end
                end

                S_DCT_START: begin
                    // 启动任务式 DCT 模块。
                    state <= S_DCT_WAIT;
                end

                S_DCT_WAIT: begin
                    // 等待 40 个 DCT 系数全部计算完成。
                    if (dct_done) begin
                        cep_idx <= {`MFCC_CEP_IDX_W{1'b0}};
                        state   <= S_OUTPUT;
                    end
                end

                S_OUTPUT: begin
                    // 按顺序输出系数，并支持输出端反压。
                    if (coeff_ready) begin
                        if (cep_idx == (`MFCC_NUM_CEPS - 1)) begin
                            frame_done <= 1'b1;
                            copy_idx   <= {`MFCC_POWER_ADDR_W{1'b0}};
                            state      <= S_COPY_INIT;
                        end else begin
                            cep_idx <= cep_idx + 1'b1;
                        end
                    end
                end

                S_COPY_INIT: begin
                    // 帧 RAM 是同步读，所以重叠复制要拆成读请求和写回两拍。
                    state <= S_COPY_ISSUE;
                end

                S_COPY_ISSUE: begin
                    state <= S_COPY_WRITE;
                end

                S_COPY_WRITE: begin
                    // 把上一帧尾部重叠区复制到下一帧起始位置。
                    if (copy_idx == (OVERLAP_LEN - 1)) begin
                        capture_req_idx <= OVERLAP_LEN;
                        capture_wr_idx  <= OVERLAP_LEN;
                        state           <= S_CAPTURE;
                    end else begin
                        copy_idx <= copy_idx + 1'b1;
                        state    <= S_COPY_ISSUE;
                    end
                end

                default: begin
                    state <= S_CAPTURE;
                end
            endcase
        end
    end

endmodule
