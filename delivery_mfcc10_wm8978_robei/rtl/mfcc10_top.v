`timescale 1ns / 1ps
module mfcc10_top (
    input  wire                              clk,
    input  wire                              rst_n,

    input  wire                              sample_valid,
    output wire                              sample_ready,
    input  wire signed [24-1:0] sample_data,

    output wire                              coeff_valid,
    input  wire                              coeff_ready,
    output wire [6-1:0]    coeff_frame_index,
    output wire [4-1:0]     coeff_index,
    output wire signed [32-1:0]   coeff_data,
    output wire                              block_done
);

    wire frame_fft_valid;
    wire signed [24-1:0] frame_fft_re;
    wire [9-1:0] frame_fft_index;
    wire frame_fft_last;
    wire frame_done;

    wire fft_busy;
    wire fft_done;
    wire fft_out_valid;
    wire [9-1:0] fft_out_index;
    wire signed [36-1:0] fft_out_re;
    wire signed [36-1:0] fft_out_im;
    wire fft_out_last;

    wire power_valid;
    wire [9-1:0] power_index;
    wire [74-1:0] power_data;
    wire power_last;

    wire power_buf_valid;
    wire power_buf_ready;
    wire [9-1:0] power_buf_index;
    wire [74-1:0] power_buf_data;
    wire power_buf_last;

    wire mel_valid;
    wire [7-1:0] mel_index;
    wire [84-1:0] mel_data;
    wire mel_frame_done;

    wire frame_sample_ready;
    reg allow_capture;
    wire capture_enable;
    wire frame_fft_ready_int;
    wire fft_start = frame_fft_valid && !fft_busy;

    assign capture_enable = allow_capture && !frame_done;
    assign sample_ready = frame_sample_ready && capture_enable;

    mfcc10_frame_window u_frame_window (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_valid (sample_valid && capture_enable),
        .sample_ready (frame_sample_ready),
        .sample_data  (sample_data),
        .fft_ready    (frame_fft_ready_int),
        .fft_valid    (frame_fft_valid),
        .fft_re       (frame_fft_re),
        .fft_last     (frame_fft_last),
        .fft_index    (frame_fft_index),
        .frame_done   (frame_done)
    );

    mfcc10_fft512_rtl u_fft (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (fft_start),
        .in_valid  (frame_fft_valid),
        .in_re     (frame_fft_re),
        .in_im     ({24{1'b0}}),
        .in_last   (frame_fft_last),
        .in_ready  (frame_fft_ready_int),
        .busy      (fft_busy),
        .done      (fft_done),
        .out_valid (fft_out_valid),
        .out_index (fft_out_index),
        .out_re    (fft_out_re),
        .out_im    (fft_out_im),
        .out_last  (fft_out_last)
    );

    mfcc10_power_spectrum u_power (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (fft_out_valid),
        .in_ready   (),
        .in_index   (fft_out_index),
        .in_re      (fft_out_re),
        .in_im      (fft_out_im),
        .out_valid  (power_valid),
        .out_ready  (1'b1),
        .out_index  (power_index),
        .out_power  (power_data),
        .out_last   (power_last),
        .frame_done ()
    );

    mfcc10_power_buffer u_power_buffer (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_valid   (power_valid),
        .in_index   (power_index),
        .in_power   (power_data),
        .in_last    (power_last),
        .out_valid  (power_buf_valid),
        .out_ready  (power_buf_ready),
        .out_index  (power_buf_index),
        .out_power  (power_buf_data),
        .out_last   (power_buf_last),
        .frame_done ()
    );

    mfcc10_mel128_sparse u_mel (
        .clk            (clk),
        .rst_n          (rst_n),
        .power_valid    (power_buf_valid),
        .power_ready    (power_buf_ready),
        .power_index    (power_buf_index),
        .power_data     (power_buf_data),
        .power_last     (power_buf_last),
        .mel_valid      (mel_valid),
        .mel_index      (mel_index),
        .mel_data       (mel_data),
        .frame_done     (mel_frame_done)
    );

    mfcc10_mfcc_block u_mfcc_block (
        .clk               (clk),
        .rst_n             (rst_n),
        .mel_valid         (mel_valid),
        .mel_index         (mel_index),
        .mel_data          (mel_data),
        .mel_frame_done    (mel_frame_done),
        .coeff_valid       (coeff_valid),
        .coeff_ready       (coeff_ready),
        .coeff_frame_index (coeff_frame_index),
        .coeff_index       (coeff_index),
        .coeff_data        (coeff_data),
        .block_done        (block_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            allow_capture <= 1'b1;
        end else begin
            if (frame_done) begin
                allow_capture <= 1'b0;
            end

            if (mel_frame_done) begin
                allow_capture <= 1'b1;
            end
        end
    end

endmodule
