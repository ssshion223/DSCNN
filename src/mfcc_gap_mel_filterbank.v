`include "mfcc_defs.vh"

// GAP 风格 Mel 滤波器组模块。
// 第一阶段先把 FFT 幅度谱写入 RAM。
// 第二阶段对 40 路 Mel 滤波器逐路扫描 257 个频点，并按稠密 Q15 权重累加。
module mfcc_gap_mel_filterbank (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           load_valid,
    input  wire [`MFCC_POWER_ADDR_W-1:0]  load_index,
    input  wire [`MFCC_FFT_W-1:0]         load_re,
    input  wire [`MFCC_FFT_W-1:0]         load_im,
    input  wire                           start,
    output reg                            busy,
    output reg                            done,
    input  wire [`MFCC_FILTER_IDX_W-1:0]  rd_index,
    output wire [31:0]                    rd_data
);

    // 每一路滤波器都被拆成一个多拍小任务，顺序扫完整个前半谱。
    localparam PHASE_FETCH = 3'd0;
    localparam PHASE_LOAD  = 3'd1;
    localparam PHASE_MUL   = 3'd2;
    localparam PHASE_ACC   = 3'd3;
    localparam PHASE_STORE = 3'd4;

    reg [31:0] mel_mem [0:`MFCC_NUM_FILTERS-1];

    reg                           power_wr_en;
    reg [`MFCC_POWER_ADDR_W-1:0]  power_wr_addr;
    reg [31:0]                    power_wr_data;
    reg                           power_rd_en;
    reg [`MFCC_POWER_ADDR_W-1:0]  power_rd_addr;
    wire [31:0]                   power_rd_data;

    reg [2:0]                     phase;
    reg [`MFCC_FILTER_IDX_W-1:0]  filter_idx;
    reg [`MFCC_POWER_ADDR_W-1:0]  bin_idx;
    reg [47:0]                    accum;
    reg [31:0]                    curr_mag_r;
    reg [15:0]                    weight_q15_r;
    reg [47:0]                    contrib_r;
    reg [47:0]                    final_sum_r;

    wire [15:0]                   weight_q15;
    wire [55:0]                   re_sq_wire;
    wire [55:0]                   im_sq_wire;
    wire [63:0]                   mag_sq_wire;

    function [31:0] sat_u32;
        input [63:0] value;
        begin
            if (|value[63:32]) begin
                sat_u32 = 32'hFFFF_FFFF;
            end else begin
                sat_u32 = value[31:0];
            end
        end
    endfunction

    // 先把 |X(k)|^2 开方成 |X(k)|，再送入 Mel 累加。
    // 这样与当前 fix16 参考链和黄金值生成方式保持一致。
    function [31:0] isqrt64;
        input [63:0] value;
        reg   [63:0] rem;
        reg   [31:0] root;
        reg   [33:0] trial;
        integer i;
        begin
            rem  = 64'd0;
            root = 32'd0;
            for (i = 31; i >= 0; i = i - 1) begin
                root  = root << 1;
                rem   = (rem << 2) | ((value >> (i * 2)) & 64'h3);
                trial = (root << 1) | 34'd1;
                if (rem >= trial) begin
                    rem  = rem - trial;
                    root = root | 32'd1;
                end
            end
            isqrt64 = root;
        end
    endfunction

    assign rd_data     = mel_mem[rd_index];
    assign mag_sq_wire = {8'd0, re_sq_wire} + {8'd0, im_sq_wire};

    mfcc_gap_mel_weight_rom u_weight_rom (
        .filter_idx (filter_idx),
        .bin_idx    (bin_idx),
        .weight_q15 (weight_q15)
    );

    mfcc_ram_sdp #(
        .DATA_W(32),
        .ADDR_W(`MFCC_POWER_ADDR_W),
        .DEPTH (`MFCC_POWER_BINS)
    ) u_mag_ram (
        .clk    (clk),
        .rst_n  (rst_n),
        .wr_en  (power_wr_en),
        .wr_addr(power_wr_addr),
        .wr_data(power_wr_data),
        .rd_en  (power_rd_en),
        .rd_addr(power_rd_addr),
        .rd_data(power_rd_data)
    );

    mfcc_tc_mul #(
        .A_W(`MFCC_FFT_W),
        .B_W(`MFCC_FFT_W),
        .P_W(56)
    ) u_re_sq (
        .a_tc(load_re),
        .b_tc(load_re),
        .p_tc(re_sq_wire)
    );

    mfcc_tc_mul #(
        .A_W(`MFCC_FFT_W),
        .B_W(`MFCC_FFT_W),
        .P_W(56)
    ) u_im_sq (
        .a_tc(load_im),
        .b_tc(load_im),
        .p_tc(im_sq_wire)
    );

    // 幅度谱 RAM 的写口用于装载 FFT 结果，
    // 读口用于驱动 Mel 相位状态机逐点读取。
    always @(*) begin
        power_wr_en   = 1'b0;
        power_wr_addr = {`MFCC_POWER_ADDR_W{1'b0}};
        power_wr_data = 32'd0;
        power_rd_en   = 1'b0;
        power_rd_addr = {`MFCC_POWER_ADDR_W{1'b0}};

        if (load_valid && (load_index < `MFCC_POWER_BINS)) begin
            power_wr_en   = 1'b1;
            power_wr_addr = load_index;
            power_wr_data = sat_u32(isqrt64(mag_sq_wire));
        end

        if (busy && (phase == PHASE_FETCH)) begin
            power_rd_en   = 1'b1;
            power_rd_addr = bin_idx;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            phase      <= PHASE_FETCH;
            filter_idx <= {`MFCC_FILTER_IDX_W{1'b0}};
            bin_idx    <= {`MFCC_POWER_ADDR_W{1'b0}};
            accum      <= 48'd0;
            curr_mag_r <= 32'd0;
            weight_q15_r <= 16'd0;
            contrib_r  <= 48'd0;
            final_sum_r<= 48'd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                // 从第 0 路滤波器、第 0 个频点开始。
                busy       <= 1'b1;
                phase      <= PHASE_FETCH;
                filter_idx <= {`MFCC_FILTER_IDX_W{1'b0}};
                bin_idx    <= {`MFCC_POWER_ADDR_W{1'b0}};
                accum      <= 48'd0;
            end else if (busy) begin
                case (phase)
                    PHASE_FETCH: begin
                        // 对当前 bin 发起同步 RAM 读请求。
                        phase <= PHASE_LOAD;
                    end

                    PHASE_LOAD: begin
                        // 接住当前 bin 的幅度值，并同时锁存对应的权重。
                        curr_mag_r   <= power_rd_data;
                        weight_q15_r <= weight_q15;
                        phase        <= PHASE_MUL;
                    end

                    PHASE_MUL: begin
                        // Q0 幅度乘 Q15 权重，再右移恢复量纲。
                        contrib_r <= (curr_mag_r * weight_q15_r) >> 15;
                        phase     <= PHASE_ACC;
                    end

                    PHASE_ACC: begin
                        // 要么继续处理下一个 bin，要么结束当前滤波器。
                        if (bin_idx == (`MFCC_POWER_BINS - 1)) begin
                            final_sum_r <= accum + contrib_r;
                            phase       <= PHASE_STORE;
                        end else begin
                            accum   <= accum + contrib_r;
                            bin_idx <= bin_idx + 1'b1;
                            phase   <= PHASE_FETCH;
                        end
                    end

                    PHASE_STORE: begin
                        // 保存当前 Mel 结果，然后切换到下一路滤波器。
                        mel_mem[filter_idx] <= sat_u32(final_sum_r);
                        accum               <= 48'd0;
                        bin_idx             <= {`MFCC_POWER_ADDR_W{1'b0}};

                        if (filter_idx == (`MFCC_NUM_FILTERS - 1)) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            phase <= PHASE_FETCH;
                        end else begin
                            filter_idx <= filter_idx + 1'b1;
                            phase      <= PHASE_FETCH;
                        end
                    end

                    default: begin
                        phase <= PHASE_FETCH;
                    end
                endcase
            end
        end
    end

endmodule
