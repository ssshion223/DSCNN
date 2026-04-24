`include "mfcc_defs.vh"

// 自然对数近似流水线。
// 内部沿用原先的 log2 查表思路，最后再把 log2(x) 转成 ln(x)。
module mfcc_gap_ln_approx #(
    parameter IN_W = 32
) (
    input  wire            clk,
    input  wire            rst_n,
    input  wire            in_valid,
    input  wire [IN_W-1:0] value_in,
    output reg             out_valid,
    output reg  [15:0]     ln_q10
);

    reg             s0_valid;
    reg [IN_W-1:0]  s0_value;

    reg             s1_valid;
    reg [IN_W-1:0]  s1_value;
    reg [4:0]       s1_msb_idx;

    reg             s2_valid;
    reg [15:0]      s2_y0_q15;
    reg [5:0]       s2_e_int;
    reg [31:0]      s2_frac_mult;

    reg             s3_valid;
    reg [31:0]      s3_log2_q15;

    reg             s4_valid;
    reg [47:0]      s4_ln_prod;

    localparam [15:0] LN2_Q15 = `MFCC_LN2_Q15;

    wire [15:0] normalized_data;
    wire [3:0]  lut_addr;
    wire [15:0] y0_lut_q15;
    wire [15:0] dy_lut_q15;
    wire [31:0] frac_mult_wire;
    wire [47:0] ln_mult_wire;

    // 找到最高位 1 的位置，用于把输入拆成指数和尾数。
    function [4:0] leading_one_pos;
        input [IN_W-1:0] value;
        integer idx;
        begin
            leading_one_pos = 5'd0;
            for (idx = 0; idx < IN_W; idx = idx + 1) begin
                if (value[idx]) begin
                    leading_one_pos = idx[4:0];
                end
            end
        end
    endfunction

    // 把输入规格化到 [0.5, 1) 的 Q15 区间，便于查表。
    function [15:0] normalize_to_q15;
        input [IN_W-1:0] value;
        input [4:0]      msb_idx;
        begin
            if (msb_idx <= 5'd14) begin
                normalize_to_q15 = value << (5'd14 - msb_idx);
            end else begin
                normalize_to_q15 = value >> (msb_idx - 5'd14);
            end
        end
    endfunction

    function [31:0] sext16_32;
        input [15:0] value;
        begin
            sext16_32 = {{16{value[15]}}, value};
        end
    endfunction

    function [31:0] ashr32_10;
        input [31:0] value;
        begin
            ashr32_10 = {{10{value[31]}}, value[31:10]};
        end
    endfunction

    function [15:0] sat16_48_20;
        input [47:0] value;
        reg [47:0] shifted;
        begin
            shifted = {{20{value[47]}}, value[47:20]};
            if (shifted[47:16] == {32{shifted[15]}}) begin
                sat16_48_20 = shifted[15:0];
            end else if (shifted[47]) begin
                sat16_48_20 = 16'h8000;
            end else begin
                sat16_48_20 = 16'h7FFF;
            end
        end
    endfunction

    // 高 4 位选查表分段，低 10 位作为段内插值小数。
    assign normalized_data = normalize_to_q15(s1_value, s1_msb_idx);
    assign lut_addr        = normalized_data[13:10];

    mfcc_log_lut u_log_lut (
        .addr   (lut_addr),
        .y0_q15 (y0_lut_q15),
        .dy_q15 (dy_lut_q15)
    );

    mfcc_tc_mul #(
        .A_W(16),
        .B_W(16),
        .P_W(32)
    ) u_frac_mul (
        .a_tc(dy_lut_q15),
        .b_tc({6'd0, normalized_data[9:0]}),
        .p_tc(frac_mult_wire)
    );

    mfcc_tc_mul #(
        .A_W(32),
        .B_W(16),
        .P_W(48)
    ) u_ln_mul (
        .a_tc(s3_log2_q15),
        .b_tc(LN2_Q15),
        .p_tc(ln_mult_wire)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid    <= 1'b0;
            s0_value    <= {IN_W{1'b0}};
            s1_valid    <= 1'b0;
            s1_value    <= {IN_W{1'b0}};
            s1_msb_idx  <= 5'd0;
            s2_valid    <= 1'b0;
            s2_y0_q15   <= 16'd0;
            s2_e_int    <= 6'd0;
            s2_frac_mult<= 32'd0;
            s3_valid    <= 1'b0;
            s3_log2_q15 <= 32'd0;
            s4_valid    <= 1'b0;
            s4_ln_prod  <= 48'd0;
            out_valid   <= 1'b0;
            ln_q10      <= 16'd0;
        end else begin
            // s0：把 0 钳成 1，避免真正去算 ln(0)。
            s0_valid <= in_valid;
            if (in_valid) begin
                if (value_in == {IN_W{1'b0}}) begin
                    s0_value <= {{(IN_W-1){1'b0}}, 1'b1};
                end else begin
                    s0_value <= value_in;
                end
            end

            // s1：锁存输入，并计算最高位 1 的位置。
            s1_valid <= s0_valid;
            if (s0_valid) begin
                s1_value   <= s0_value;
                s1_msb_idx <= leading_one_pos(s0_value);
            end

            // s2：取 LUT 基值，并计算插值斜率乘积。
            s2_valid <= s1_valid;
            if (s1_valid) begin
                s2_y0_q15    <= y0_lut_q15;
                s2_e_int     <= s1_msb_idx + 6'd1;
                s2_frac_mult <= frac_mult_wire;
            end

            // s3：把指数、LUT 基值和插值项组合成 log2(x)。
            s3_valid <= s2_valid;
            if (s2_valid) begin
                s3_log2_q15 <= ({26'd0, s2_e_int} << 15)
                             + sext16_32(s2_y0_q15)
                             + ashr32_10(s2_frac_mult);
            end

            // s4：乘 ln(2)，把 log2(x) 转成 ln(x)。
            s4_valid <= s3_valid;
            if (s3_valid) begin
                s4_ln_prod <= ln_mult_wire;
            end

            // 输出格式为 Q10 的自然对数。
            out_valid <= s4_valid;
            if (s4_valid) begin
                ln_q10 <= sat16_48_20(s4_ln_prod);
            end
        end
    end

endmodule
