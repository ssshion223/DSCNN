`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_log2_pipe #(
    parameter IN_W = `MFCC10_MEL_ACC_W,
    parameter USER_W = `MFCC10_MEL_IDX_W
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       in_valid,
    input  wire [IN_W-1:0]            in_data,
    input  wire [USER_W-1:0]          in_user,
    output reg                        out_valid,
    output reg  [`MFCC10_LOG_W-1:0]   out_log_q16,
    output reg  [USER_W-1:0]          out_user
);

    reg                        s0_valid;
    reg [IN_W-1:0]             s0_data;
    reg [USER_W-1:0]           s0_user;

    reg                        s1_valid;
    reg [IN_W-1:0]             s1_data;
    reg [USER_W-1:0]           s1_user;
    reg [6:0]                  s1_msb_idx;

    reg                        s2_valid;
    reg [USER_W-1:0]           s2_user;
    reg [15:0]                 s2_norm_q16;
    reg [6:0]                  s2_exp_int;

    reg                        s3_valid;
    reg [USER_W-1:0]           s3_user;
    reg [11:0]                 s3_frac;
    reg signed [17:0]          s3_y0_q16;
    reg signed [17:0]          s3_dy_q16;
    reg [6:0]                  s3_exp_int;

    reg                        s4_valid;
    reg [USER_W-1:0]           s4_user;
    reg signed [31:0]          s4_interp_q16;
    reg signed [17:0]          s4_y0_q16;
    reg [6:0]                  s4_exp_int;

    reg                        s5_valid;
    reg [USER_W-1:0]           s5_user;
    reg signed [39:0]          s5_log_q16;

    wire [IN_W-1:0]            in_clamped;
    wire [3:0]                 lut_addr;
    wire signed [17:0]         lut_y0_q16;
    wire signed [17:0]         lut_dy_q16;
    wire signed [31:0]         interp_wire;
    wire signed [39:0]         exp_q16_wire;

    assign in_clamped = (in_data == {IN_W{1'b0}}) ? {{(IN_W-1){1'b0}}, 1'b1} : in_data;
    assign lut_addr = s2_norm_q16[14:11];
    assign interp_wire = ($signed(s3_dy_q16) * $signed({1'b0, s3_frac})) >>> 12;
    assign exp_q16_wire = $signed({33'd0, s4_exp_int}) <<< `MFCC10_LOG_FRAC_W;

    mfcc10_log2_lut_q16 u_lut (
        .addr   (lut_addr),
        .y0_q16 (lut_y0_q16),
        .dy_q16 (lut_dy_q16)
    );

    function [6:0] leading_one_pos;
        input [IN_W-1:0] value;
        integer idx;
        begin
            leading_one_pos = 7'd0;
            for (idx = 0; idx < IN_W; idx = idx + 1) begin
                if (value[idx]) begin
                    leading_one_pos = idx[6:0];
                end
            end
        end
    endfunction

    function [15:0] normalize_to_q16;
        input [IN_W-1:0] value;
        input [6:0]      msb_idx;
        begin
            if (msb_idx <= 7'd15) begin
                normalize_to_q16 = value << (7'd15 - msb_idx);
            end else begin
                normalize_to_q16 = value >> (msb_idx - 7'd15);
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid    <= 1'b0;
            s0_data     <= {IN_W{1'b0}};
            s0_user     <= {USER_W{1'b0}};
            s1_valid    <= 1'b0;
            s1_data     <= {IN_W{1'b0}};
            s1_user     <= {USER_W{1'b0}};
            s1_msb_idx  <= 7'd0;
            s2_valid    <= 1'b0;
            s2_user     <= {USER_W{1'b0}};
            s2_norm_q16 <= 16'd0;
            s2_exp_int  <= 7'd0;
            s3_valid    <= 1'b0;
            s3_user     <= {USER_W{1'b0}};
            s3_frac     <= 12'd0;
            s3_y0_q16   <= 18'sd0;
            s3_dy_q16   <= 18'sd0;
            s3_exp_int  <= 7'd0;
            s4_valid    <= 1'b0;
            s4_user     <= {USER_W{1'b0}};
            s4_interp_q16 <= 32'sd0;
            s4_y0_q16   <= 18'sd0;
            s4_exp_int  <= 7'd0;
            s5_valid    <= 1'b0;
            s5_user     <= {USER_W{1'b0}};
            s5_log_q16  <= 40'sd0;
            out_valid   <= 1'b0;
            out_log_q16 <= {`MFCC10_LOG_W{1'b0}};
            out_user    <= {USER_W{1'b0}};
        end else begin
            s0_valid <= in_valid;
            if (in_valid) begin
                s0_data <= in_clamped;
                s0_user <= in_user;
            end

            s1_valid <= s0_valid;
            if (s0_valid) begin
                s1_data    <= s0_data;
                s1_user    <= s0_user;
                s1_msb_idx <= leading_one_pos(s0_data);
            end

            s2_valid <= s1_valid;
            if (s1_valid) begin
                s2_user     <= s1_user;
                s2_norm_q16 <= normalize_to_q16(s1_data, s1_msb_idx);
                s2_exp_int  <= s1_msb_idx + 7'd1;
            end

            s3_valid <= s2_valid;
            if (s2_valid) begin
                s3_user    <= s2_user;
                s3_frac    <= s2_norm_q16[10:0] << 1;
                s3_y0_q16  <= lut_y0_q16;
                s3_dy_q16  <= lut_dy_q16;
                s3_exp_int <= s2_exp_int;
            end

            s4_valid <= s3_valid;
            if (s3_valid) begin
                s4_user       <= s3_user;
                s4_interp_q16 <= interp_wire;
                s4_y0_q16     <= s3_y0_q16;
                s4_exp_int    <= s3_exp_int;
            end

            s5_valid <= s4_valid;
            if (s4_valid) begin
                s5_user    <= s4_user;
                s5_log_q16 <= exp_q16_wire + s4_y0_q16 + s4_interp_q16;
            end

            out_valid <= s5_valid;
            if (s5_valid) begin
                out_user    <= s5_user;
                out_log_q16 <= s5_log_q16[`MFCC10_LOG_W-1:0];
            end
        end
    end

endmodule


