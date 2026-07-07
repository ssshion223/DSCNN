`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_wm8978_i2s_master_rx #(
    parameter integer SAMPLE_BITS = `MFCC10_SAMPLE_W,
    parameter integer SLOT_BITS   = 32
) (
    input  wire audio_clk,
    input  wire rst_n,

    input  wire adc_data,
    output reg  mclk,
    output reg  bclk,
    output reg  lrclk,

    output reg                                sample_valid,
    output reg signed [`MFCC10_SAMPLE_W-1:0]  sample_data
);

    reg [2:0] audio_div_count;
    reg [5:0] bit_idx;
    reg [SAMPLE_BITS-1:0] shift_r;
    reg [SAMPLE_BITS-1:0] left_sample_r;

    wire bclk_tick;
    wire bclk_rise_event;
    wire capture_bit;
    wire last_capture_bit;
    wire [SAMPLE_BITS-1:0] sample_next_wire;

    assign bclk_tick = (audio_div_count == 3'd3);
    assign bclk_rise_event = bclk_tick && !bclk;
    assign capture_bit = (bit_idx >= 6'd1) && (bit_idx <= SAMPLE_BITS[5:0]);
    assign last_capture_bit = (bit_idx == SAMPLE_BITS[5:0]);
    assign sample_next_wire = {shift_r[SAMPLE_BITS-2:0], adc_data};

    always @(posedge audio_clk or negedge rst_n) begin
        if (!rst_n) begin
            audio_div_count <= 3'd0;
            mclk            <= 1'b0;
            bclk            <= 1'b0;
            bit_idx         <= 6'd0;
            lrclk           <= 1'b0;
        end else begin
            mclk <= ~mclk;

            if (bclk_tick) begin
                audio_div_count <= 3'd0;
                bclk            <= ~bclk;

                if (bclk) begin
                    if (bit_idx == (SLOT_BITS - 1)) begin
                        bit_idx <= 6'd0;
                        lrclk   <= ~lrclk;
                    end else begin
                        bit_idx <= bit_idx + 1'b1;
                    end
                end
            end else begin
                audio_div_count <= audio_div_count + 1'b1;
            end
        end
    end

    always @(posedge audio_clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_r       <= {SAMPLE_BITS{1'b0}};
            left_sample_r <= {SAMPLE_BITS{1'b0}};
            sample_valid  <= 1'b0;
            sample_data   <= {`MFCC10_SAMPLE_W{1'b0}};
        end else begin
            sample_valid <= 1'b0;

            if (bclk_rise_event) begin
                if (capture_bit) begin
                    shift_r <= sample_next_wire;

                    if (last_capture_bit) begin
                        if (lrclk == 1'b0) begin
                            left_sample_r <= sample_next_wire;
                        end else begin
                            sample_data  <= left_sample_r;
                            sample_valid <= 1'b1;
                        end
                    end
                end

                if (bit_idx == (SLOT_BITS - 1)) begin
                    shift_r <= {SAMPLE_BITS{1'b0}};
                end
            end
        end
    end

endmodule
