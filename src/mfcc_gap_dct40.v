`include "mfcc_defs.vh"

// 40x40 DCT 模块。
// 先缓存全部 log-Mel 输入，再对每个输出系数执行一次串行乘加任务，
// 最后把 40 个结果存入内部存储。
module mfcc_gap_dct40 (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           load_valid,
    input  wire [`MFCC_FILTER_IDX_W-1:0]  load_index,
    input  wire [15:0]                    load_data,
    input  wire                           start,
    output reg                            busy,
    output reg                            done,
    input  wire [`MFCC_CEP_IDX_W-1:0]     rd_index,
    output wire [`MFCC_DCT_OUT_W-1:0]     rd_data
);

    // 每个输出系数都要遍历 40 路输入通道。
    localparam PHASE_FETCH = 2'd0;
    localparam PHASE_MUL   = 2'd1;
    localparam PHASE_ACC   = 2'd2;
    localparam PHASE_STORE = 2'd3;

    reg [15:0] log_mem [0:`MFCC_NUM_FILTERS-1];
    reg [`MFCC_DCT_OUT_W-1:0] cep_mem [0:`MFCC_NUM_CEPS-1];

    reg [1:0]                    phase;
    reg [`MFCC_CEP_IDX_W-1:0]    cep_idx;
    reg [`MFCC_FILTER_IDX_W-1:0] filt_idx;
    reg [47:0]                   accum;
    reg [15:0]                   log_sample_r;
    reg [15:0]                   coeff_r;
    reg [31:0]                   prod_r;
    reg [47:0]                   final_out_r;

    wire [`MFCC_DCT_ROM_ADDR_W-1:0] coeff_addr;
    wire [15:0]                     coeff_q14;
    wire [31:0]                     prod_wire;

    function [47:0] sext32_48;
        input [31:0] value;
        begin
            sext32_48 = {{16{value[31]}}, value};
        end
    endfunction

    function [47:0] ashr48_q;
        input [47:0] value;
        begin
            ashr48_q = {{`MFCC_DCT_Q{value[47]}}, value[47:`MFCC_DCT_Q]};
        end
    endfunction

    function [`MFCC_DCT_OUT_W-1:0] sat_dct;
        input [47:0] value;
        begin
            if (value[47:`MFCC_DCT_OUT_W] == {(48-`MFCC_DCT_OUT_W){value[`MFCC_DCT_OUT_W-1]}}) begin
                sat_dct = value[`MFCC_DCT_OUT_W-1:0];
            end else if (value[47]) begin
                sat_dct = {1'b1, {(`MFCC_DCT_OUT_W-1){1'b0}}};
            end else begin
                sat_dct = {1'b0, {(`MFCC_DCT_OUT_W-1){1'b1}}};
            end
        end
    endfunction

    // DCT 系数在 ROM 中按 [cep][filter] 展平成一维存放。
    assign coeff_addr = (cep_idx * `MFCC_NUM_FILTERS) + filt_idx;
    assign rd_data    = cep_mem[rd_index];

    mfcc_gap_dct_rom u_dct_rom (
        .addr      (coeff_addr),
        .coeff_q14 (coeff_q14)
    );

    mfcc_tc_mul #(
        .A_W(16),
        .B_W(16),
        .P_W(32)
    ) u_dct_mul (
        .a_tc(log_sample_r),
        .b_tc(coeff_r),
        .p_tc(prod_wire)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy         <= 1'b0;
            done         <= 1'b0;
            phase        <= PHASE_FETCH;
            cep_idx      <= {`MFCC_CEP_IDX_W{1'b0}};
            filt_idx     <= {`MFCC_FILTER_IDX_W{1'b0}};
            accum        <= 48'd0;
            log_sample_r <= 16'd0;
            coeff_r      <= 16'd0;
            prod_r       <= 32'd0;
            final_out_r  <= 48'd0;
        end else begin
            done <= 1'b0;

            // DCT 启动前，先把 40 路 log-Mel 全部装入输入缓存。
            if (load_valid && !busy) begin
                log_mem[load_index] <= load_data;
            end

            if (start && !busy) begin
                // 从第 0 个输出系数、第 0 路输入通道开始。
                busy     <= 1'b1;
                phase    <= PHASE_FETCH;
                cep_idx  <= {`MFCC_CEP_IDX_W{1'b0}};
                filt_idx <= {`MFCC_FILTER_IDX_W{1'b0}};
                accum    <= 48'd0;
            end else if (busy) begin
                case (phase)
                    PHASE_FETCH: begin
                        // 取一条输入样本和对应的 DCT 系数。
                        log_sample_r <= log_mem[filt_idx];
                        coeff_r      <= coeff_q14;
                        phase        <= PHASE_MUL;
                    end

                    PHASE_MUL: begin
                        // 乘法单独占一拍，缩短乘加关键路径。
                        prod_r <= prod_wire;
                        phase  <= PHASE_ACC;
                    end

                    PHASE_ACC: begin
                        // 为当前输出系数累加 40 项乘积。
                        if (filt_idx == (`MFCC_NUM_FILTERS - 1)) begin
                            final_out_r <= ashr48_q(accum + sext32_48(prod_r));
                            phase       <= PHASE_STORE;
                        end else begin
                            accum    <= accum + sext32_48(prod_r);
                            filt_idx <= filt_idx + 1'b1;
                            phase    <= PHASE_FETCH;
                        end
                    end

                    PHASE_STORE: begin
                        // 保存当前系数，然后决定结束还是切到下一个系数。
                        cep_mem[cep_idx] <= sat_dct(final_out_r);
                        accum            <= 48'd0;
                        filt_idx         <= {`MFCC_FILTER_IDX_W{1'b0}};

                        if (cep_idx == (`MFCC_NUM_CEPS - 1)) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            phase <= PHASE_FETCH;
                        end else begin
                            cep_idx <= cep_idx + 1'b1;
                            phase   <= PHASE_FETCH;
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
