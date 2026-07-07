`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_power_spectrum (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              in_valid,
    output wire                              in_ready,
    input  wire [`MFCC10_FFT_ADDR_W-1:0]     in_index,
    input  wire signed [`MFCC10_FFT_OUT_W-1:0] in_re,
    input  wire signed [`MFCC10_FFT_OUT_W-1:0] in_im,
    output reg                               out_valid,
    input  wire                              out_ready,
    output reg  [`MFCC10_POWER_ADDR_W-1:0]   out_index,
    output reg  [`MFCC10_POWER_W-1:0]        out_power,
    output reg                               out_last,
    output reg                               frame_done
);

    localparam integer SQ_W = 2 * `MFCC10_FFT_OUT_W;
    localparam integer SUM_W = SQ_W + 1;

    reg                              s0_valid;
    reg [`MFCC10_POWER_ADDR_W-1:0]   s0_index;
    reg signed [`MFCC10_FFT_OUT_W-1:0] s0_re;
    reg signed [`MFCC10_FFT_OUT_W-1:0] s0_im;

    reg                              s1_valid;
    reg [`MFCC10_POWER_ADDR_W-1:0]   s1_index;
    reg [SQ_W-1:0]                   s1_re_sq;
    reg [SQ_W-1:0]                   s1_im_sq;

    reg                              s2_valid;
    reg [`MFCC10_POWER_ADDR_W-1:0]   s2_index;
    reg [SUM_W-1:0]                  s2_power_full;

    wire                             pipe_ce;
    wire                             in_fire;
    wire [SQ_W-1:0]                  re_sq_wire;
    wire [SQ_W-1:0]                  im_sq_wire;

    assign pipe_ce = !out_valid || out_ready;
    assign in_ready = pipe_ce;
    assign in_fire = in_valid && in_ready && (in_index < `MFCC10_POWER_BINS);
    assign re_sq_wire = s0_re * s0_re;
    assign im_sq_wire = s0_im * s0_im;

    function [`MFCC10_POWER_W-1:0] sat_power;
        input [SUM_W-1:0] value;
        begin
            sat_power = {{(`MFCC10_POWER_W-SUM_W){1'b0}}, value};
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid     <= 1'b0;
            s0_index     <= {`MFCC10_POWER_ADDR_W{1'b0}};
            s0_re        <= {`MFCC10_FFT_OUT_W{1'b0}};
            s0_im        <= {`MFCC10_FFT_OUT_W{1'b0}};
            s1_valid     <= 1'b0;
            s1_index     <= {`MFCC10_POWER_ADDR_W{1'b0}};
            s1_re_sq     <= {SQ_W{1'b0}};
            s1_im_sq     <= {SQ_W{1'b0}};
            s2_valid     <= 1'b0;
            s2_index     <= {`MFCC10_POWER_ADDR_W{1'b0}};
            s2_power_full <= {SUM_W{1'b0}};
            out_valid    <= 1'b0;
            out_index    <= {`MFCC10_POWER_ADDR_W{1'b0}};
            out_power    <= {`MFCC10_POWER_W{1'b0}};
            out_last     <= 1'b0;
            frame_done   <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (pipe_ce) begin
                s0_valid <= in_fire;
                if (in_fire) begin
                    s0_index <= in_index[`MFCC10_POWER_ADDR_W-1:0];
                    s0_re    <= in_re;
                    s0_im    <= in_im;
                end

                s1_valid <= s0_valid;
                if (s0_valid) begin
                    s1_index <= s0_index;
                    s1_re_sq <= re_sq_wire;
                    s1_im_sq <= im_sq_wire;
                end

                s2_valid <= s1_valid;
                if (s1_valid) begin
                    s2_index      <= s1_index;
                    s2_power_full <= {1'b0, s1_re_sq} + {1'b0, s1_im_sq};
                end

                out_valid <= s2_valid;
                if (s2_valid) begin
                    out_index <= s2_index;
                    out_power <= sat_power(s2_power_full);
                    out_last  <= (s2_index == (`MFCC10_POWER_BINS - 1));
                    if (s2_index == (`MFCC10_POWER_BINS - 1)) begin
                        frame_done <= 1'b1;
                    end
                end else begin
                    out_last <= 1'b0;
                end
            end
        end
    end

endmodule


