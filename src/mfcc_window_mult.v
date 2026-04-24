// 加窗乘法器：sample * window_q15，再右移 15 位恢复量纲。
module mfcc_window_mult (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire [15:0] sample_in,
    input  wire [15:0] window_q15,
    output reg         out_valid,
    output reg  [15:0] sample_out
);

    // 三拍流水：锁输入 -> 乘法 -> 缩放，最后饱和输出。
    reg        stage0_valid;
    reg        stage1_valid;
    reg        stage2_valid;
    reg [15:0] sample_reg;
    reg [15:0] window_reg;
    reg [31:0] mult_reg;
    reg [31:0] scaled_reg;
    wire [31:0] mult_wire;

    mfcc_tc_mul #(
        .A_W(16),
        .B_W(16),
        .P_W(32)
    ) u_tc_mul (
        .a_tc(sample_reg),
        .b_tc(window_reg),
        .p_tc(mult_wire)
    );

    function [31:0] ashr32_15;
        input [31:0] value;
        begin
            ashr32_15 = {{15{value[31]}}, value[31:15]};
        end
    endfunction

    function [15:0] sat16;
        input [31:0] value;
        begin
            if (value[31:15] == {17{value[15]}}) begin
                sat16 = value[15:0];
            end else if (value[31]) begin
                sat16 = 16'h8000;
            end else begin
                sat16 = 16'h7FFF;
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stage0_valid <= 1'b0;
            stage1_valid <= 1'b0;
            stage2_valid <= 1'b0;
            out_valid    <= 1'b0;
            sample_reg   <= 16'd0;
            window_reg   <= 16'd0;
            mult_reg     <= 32'd0;
            scaled_reg   <= 32'd0;
            sample_out   <= 16'd0;
        end else begin
            stage0_valid <= in_valid;
            if (in_valid) begin
                sample_reg <= sample_in;
                window_reg <= window_q15;
            end

            stage1_valid <= stage0_valid;
            if (stage0_valid) begin
                mult_reg <= mult_wire;
            end

            stage2_valid <= stage1_valid;
            if (stage1_valid) begin
                scaled_reg <= ashr32_15(mult_reg);
            end

            out_valid <= stage2_valid;
            if (stage2_valid) begin
                sample_out <= sat16(scaled_reg);
            end
        end
    end

endmodule
