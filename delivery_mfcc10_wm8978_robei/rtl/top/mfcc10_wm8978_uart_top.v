`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_wm8978_uart_top #(
    parameter integer SYS_CLK_HZ    = 50000000,
    parameter integer WM8978_I2C_HZ = 10000,
    parameter integer UART_BAUD     = 115200
) (
    input  wire                            sys_clk,
    input  wire                            sys_rst_n,

    input  wire                            wm8978_sdb,
    output wire                            wm8978_scka,
    output wire                            wm8978_fsa,
    output wire                            wm8978_mclka,
    output wire                            wm8978_iic_scl,
    inout  wire                            wm8978_iic_sda,

    output wire                            uart_txd,

    output wire                            wm8978_init_done,
    output wire                            wm8978_init_busy,
    output wire                            wm8978_init_error,
    output wire [4:0]                      wm8978_fail_init_idx,
    output wire [1:0]                      wm8978_fail_ack_byte,
    output wire                            sample_cdc_overflow,
    output wire                            sample_fifo_full,
    output wire                            uart_busy,
    output wire                            packet_busy
);

    wire                              coeff_valid;
    wire                              coeff_ready;
    wire [`MFCC10_FRAME_IDX_W-1:0]    coeff_frame_index;
    wire [`MFCC10_MFCC_IDX_W-1:0]     coeff_index;
    wire signed [`MFCC10_OUT_W-1:0]   coeff_data;
    wire                              block_done;

    wire                              packet_valid;
    wire                              packet_ready;
    wire [7:0]                        packet_data;

    mfcc10_wm8978_top #(
        .SYS_CLK_HZ    (SYS_CLK_HZ),
        .WM8978_I2C_HZ (WM8978_I2C_HZ)
    ) u_mfcc10_wm8978_top (
        .sys_clk              (sys_clk),
        .sys_rst_n            (sys_rst_n),
        .wm8978_sdb           (wm8978_sdb),
        .wm8978_scka          (wm8978_scka),
        .wm8978_fsa           (wm8978_fsa),
        .wm8978_mclka         (wm8978_mclka),
        .wm8978_iic_scl       (wm8978_iic_scl),
        .wm8978_iic_sda       (wm8978_iic_sda),
        .coeff_valid          (coeff_valid),
        .coeff_ready          (coeff_ready),
        .coeff_frame_index    (coeff_frame_index),
        .coeff_index          (coeff_index),
        .coeff_data           (coeff_data),
        .block_done           (block_done),
        .wm8978_init_done     (wm8978_init_done),
        .wm8978_init_busy     (wm8978_init_busy),
        .wm8978_init_error    (wm8978_init_error),
        .wm8978_fail_init_idx (wm8978_fail_init_idx),
        .wm8978_fail_ack_byte (wm8978_fail_ack_byte),
        .sample_cdc_overflow  (sample_cdc_overflow),
        .sample_fifo_full     (sample_fifo_full)
    );

    mfcc10_mfcc_uart_packetizer u_mfcc_uart_packetizer (
        .clk               (sys_clk),
        .rst_n             (sys_rst_n),
        .coeff_valid       (coeff_valid),
        .coeff_ready       (coeff_ready),
        .coeff_frame_index (coeff_frame_index),
        .coeff_index       (coeff_index),
        .coeff_data        (coeff_data),
        .block_done        (block_done),
        .packet_valid      (packet_valid),
        .packet_ready      (packet_ready),
        .packet_data       (packet_data),
        .packet_busy       (packet_busy)
    );

    mfcc10_uart_tx #(
        .CLK_HZ (SYS_CLK_HZ),
        .BAUD   (UART_BAUD)
    ) u_uart_tx (
        .clk      (sys_clk),
        .rst_n    (sys_rst_n),
        .tx_valid (packet_valid),
        .tx_ready (packet_ready),
        .tx_data  (packet_data),
        .txd      (uart_txd),
        .busy     (uart_busy)
    );

endmodule
