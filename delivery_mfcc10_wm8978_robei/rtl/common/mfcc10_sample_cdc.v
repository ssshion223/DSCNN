`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_sample_cdc (
    input  wire                               src_clk,
    input  wire                               src_rst_n,
    input  wire                               src_valid,
    input  wire signed [`MFCC10_SAMPLE_W-1:0] src_sample,
    output wire                               src_ready,
    output reg                                src_overflow,

    input  wire                               dst_clk,
    input  wire                               dst_rst_n,
    output reg                                dst_valid,
    input  wire                               dst_ready,
    output reg signed [`MFCC10_SAMPLE_W-1:0]  dst_sample
);

    reg req_toggle_src;
    reg ack_sync_src_0;
    reg ack_sync_src_1;
    reg signed [`MFCC10_SAMPLE_W-1:0] sample_hold_src;

    reg req_sync_dst_0;
    reg req_sync_dst_1;
    reg req_seen_dst;
    reg ack_toggle_dst;

    wire src_can_send;
    wire dst_can_load;
    wire dst_has_new_sample;

    assign src_can_send = (req_toggle_src == ack_sync_src_1);
    assign src_ready = src_can_send;
    assign dst_can_load = !dst_valid || dst_ready;
    assign dst_has_new_sample = (req_sync_dst_1 != req_seen_dst);

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            req_toggle_src  <= 1'b0;
            ack_sync_src_0  <= 1'b0;
            ack_sync_src_1  <= 1'b0;
            sample_hold_src <= {`MFCC10_SAMPLE_W{1'b0}};
            src_overflow    <= 1'b0;
        end else begin
            ack_sync_src_0 <= ack_toggle_dst;
            ack_sync_src_1 <= ack_sync_src_0;

            if (src_valid && src_can_send) begin
                sample_hold_src <= src_sample;
                req_toggle_src  <= ~req_toggle_src;
            end else if (src_valid && !src_can_send) begin
                src_overflow <= 1'b1;
            end
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_sync_dst_0 <= 1'b0;
            req_sync_dst_1 <= 1'b0;
            req_seen_dst   <= 1'b0;
            ack_toggle_dst <= 1'b0;
            dst_valid      <= 1'b0;
            dst_sample     <= {`MFCC10_SAMPLE_W{1'b0}};
        end else begin
            req_sync_dst_0 <= req_toggle_src;
            req_sync_dst_1 <= req_sync_dst_0;

            if (dst_valid && dst_ready) begin
                dst_valid <= 1'b0;
            end

            if (dst_has_new_sample && dst_can_load) begin
                dst_sample     <= sample_hold_src;
                dst_valid      <= 1'b1;
                req_seen_dst   <= req_sync_dst_1;
                ack_toggle_dst <= req_sync_dst_1;
            end
        end
    end

endmodule
