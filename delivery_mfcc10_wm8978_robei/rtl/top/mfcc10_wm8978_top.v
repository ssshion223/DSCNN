`timescale 1ns / 1ps
module mfcc10_wm8978_top #(
    parameter SYS_CLK_HZ    = 50000000,
    parameter WM8978_I2C_HZ = 10000
) (
    input  wire                              sys_clk,
    input  wire                              sys_rst_n,

    input  wire                              wm8978_sdb,
    output wire                              wm8978_scka,
    output wire                              wm8978_fsa,
    output wire                              wm8978_mclka,
    output wire                              wm8978_iic_scl,
    inout  wire                              wm8978_iic_sda,

    output wire                              coeff_valid,
    input  wire                              coeff_ready,
    output wire [6-1:0]    coeff_frame_index,
    output wire [4-1:0]     coeff_index,
    output wire signed [32-1:0]   coeff_data,
    output wire                              block_done,

    output wire                              wm8978_init_done,
    output wire                              wm8978_init_busy,
    output wire                              wm8978_init_error,
    output wire [4:0]                        wm8978_fail_init_idx,
    output wire [1:0]                        wm8978_fail_ack_byte,
    output wire                              sample_cdc_overflow,
    output wire                              sample_fifo_full
);

    wire sys_clk_buf;
    wire audio_clk;
    wire pll_locked;
    wire core_rst_n;
    wire audio_capture_rst_n;

    reg wm8978_init_done_audio_s0;
    reg wm8978_init_done_audio_s1;

    wire                              i2s_sample_valid;
    wire signed [24-1:0] i2s_sample_data;
    wire                              sample_sys_valid;
    wire                              sample_sys_ready;
    wire signed [24-1:0] sample_sys_data;
    wire [12:0]                       sample_fifo_count;
    wire                              mfcc_sample_valid;
    wire                              mfcc_sample_ready;
    wire signed [24-1:0] mfcc_sample_data;
    wire                              sample_fifo_empty;

    assign core_rst_n = sys_rst_n & pll_locked;
    assign audio_capture_rst_n = core_rst_n & wm8978_init_done_audio_s1;

    assign sys_clk_buf = sys_clk;

    mfcc10_audio_pll_8m192_nobuf u_audio_pll (
        .clk_in  (sys_clk_buf),
        .reset   (~sys_rst_n),
        .clk_out (audio_clk),
        .locked  (pll_locked)
    );

    mfcc10_wm8978_init_adc #(
        .CLK_HZ (SYS_CLK_HZ),
        .I2C_HZ (WM8978_I2C_HZ)
    ) u_wm8978_init_adc (
        .clk        (sys_clk_buf),
        .rst_n      (core_rst_n),
        .iic_scl    (wm8978_iic_scl),
        .iic_sda    (wm8978_iic_sda),
        .init_done  (wm8978_init_done),
        .init_busy  (wm8978_init_busy),
        .init_error (wm8978_init_error),
        .fail_init_idx (wm8978_fail_init_idx),
        .fail_ack_byte (wm8978_fail_ack_byte)
    );

    mfcc10_wm8978_i2s_master_rx #(
        .SAMPLE_BITS (24),
        .SLOT_BITS   (32)
    ) u_wm8978_i2s_master_rx (
        .audio_clk    (audio_clk),
        .rst_n        (core_rst_n),
        .adc_data     (wm8978_sdb),
        .mclk         (wm8978_mclka),
        .bclk         (wm8978_scka),
        .lrclk        (wm8978_fsa),
        .sample_valid (i2s_sample_valid),
        .sample_data  (i2s_sample_data)
    );

    always @(posedge audio_clk or negedge core_rst_n) begin
        if (!core_rst_n) begin
            wm8978_init_done_audio_s0 <= 1'b0;
            wm8978_init_done_audio_s1 <= 1'b0;
        end else begin
            wm8978_init_done_audio_s0 <= wm8978_init_done;
            wm8978_init_done_audio_s1 <= wm8978_init_done_audio_s0;
        end
    end

    mfcc10_sample_cdc u_sample_cdc (
        .src_clk      (audio_clk),
        .src_rst_n    (audio_capture_rst_n),
        .src_valid    (i2s_sample_valid),
        .src_sample   (i2s_sample_data),
        .src_ready    (),
        .src_overflow (sample_cdc_overflow),
        .dst_clk      (sys_clk_buf),
        .dst_rst_n    (core_rst_n),
        .dst_valid    (sample_sys_valid),
        .dst_ready    (sample_sys_ready),
        .dst_sample   (sample_sys_data)
    );

    mfcc10_input_fifo u_input_fifo (
        .clk        (sys_clk_buf),
        .rst_n      (core_rst_n),
        .src_valid  (sample_sys_valid),
        .src_sample (sample_sys_data),
        .src_ready  (sample_sys_ready),
        .fifo_full  (sample_fifo_full),
        .data_count (sample_fifo_count),
        .dst_valid  (mfcc_sample_valid),
        .dst_ready  (mfcc_sample_ready),
        .dst_sample (mfcc_sample_data),
        .fifo_empty (sample_fifo_empty)
    );

    mfcc10_top u_mfcc10_top (
        .clk               (sys_clk_buf),
        .rst_n             (core_rst_n),
        .sample_valid      (mfcc_sample_valid),
        .sample_ready      (mfcc_sample_ready),
        .sample_data       (mfcc_sample_data),
        .coeff_valid       (coeff_valid),
        .coeff_ready       (coeff_ready),
        .coeff_frame_index (coeff_frame_index),
        .coeff_index       (coeff_index),
        .coeff_data        (coeff_data),
        .block_done        (block_done)
    );

endmodule

