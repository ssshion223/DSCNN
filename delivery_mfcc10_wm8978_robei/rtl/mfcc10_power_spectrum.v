`timescale 1ns / 1ps
module mfcc10_power_spectrum (
    input  wire                              clk,
    input  wire                              rst_n,
    input  wire                              in_valid,
    output wire                              in_ready,
    input  wire [9-1:0]     in_index,
    input  wire signed [36-1:0] in_re,
    input  wire signed [36-1:0] in_im,
    output reg                               out_valid,
    input  wire                              out_ready,
    output reg  [9-1:0]   out_index,
    output reg  [74-1:0]        out_power,
    output reg                               out_last,
    output reg                               frame_done
);

    localparam SQ_W = 2 * 36;
    localparam SUM_W = SQ_W + 1;

    reg                              s0_valid;
    reg [9-1:0]   s0_index;
    reg signed [36-1:0] s0_re;
    reg signed [36-1:0] s0_im;

    reg                              s1_valid;
    reg [9-1:0]   s1_index;
    reg [SQ_W-1:0]                   s1_re_sq;
    reg [SQ_W-1:0]                   s1_im_sq;

    reg                              s2_valid;
    reg [9-1:0]   s2_index;
    reg [SUM_W-1:0]                  s2_power_full;

    wire                             pipe_ce;
    wire                             in_fire;
    wire [SQ_W-1:0]                  re_sq_wire;
    wire [SQ_W-1:0]                  im_sq_wire;

    assign pipe_ce = !out_valid || out_ready;
    assign in_ready = pipe_ce;
    assign in_fire = in_valid && in_ready && (in_index < 257);
    assign re_sq_wire = s0_re * s0_re;
    assign im_sq_wire = s0_im * s0_im;

    function [74-1:0] sat_power;
        input [SUM_W-1:0] value;
        begin
            sat_power = {{(74-SUM_W){1'b0}}, value};
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid     <= 1'b0;
            s0_index     <= {9{1'b0}};
            s0_re        <= {36{1'b0}};
            s0_im        <= {36{1'b0}};
            s1_valid     <= 1'b0;
            s1_index     <= {9{1'b0}};
            s1_re_sq     <= {SQ_W{1'b0}};
            s1_im_sq     <= {SQ_W{1'b0}};
            s2_valid     <= 1'b0;
            s2_index     <= {9{1'b0}};
            s2_power_full <= {SUM_W{1'b0}};
            out_valid    <= 1'b0;
            out_index    <= {9{1'b0}};
            out_power    <= {74{1'b0}};
            out_last     <= 1'b0;
            frame_done   <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (pipe_ce) begin
                s0_valid <= in_fire;
                if (in_fire) begin
                    s0_index <= in_index[9-1:0];
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
                    out_last  <= (s2_index == (257 - 1));
                    if (s2_index == (257 - 1)) begin
                        frame_done <= 1'b1;
                    end
                end else begin
                    out_last <= 1'b0;
                end
            end
        end
    end

endmodule


