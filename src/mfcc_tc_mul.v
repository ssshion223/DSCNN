// 二补码有符号乘法器，不依赖 signed 关键字。
module mfcc_tc_mul #(
    parameter A_W = 16,
    parameter B_W = 16,
    parameter P_W = A_W + B_W
) (
    input  wire [A_W-1:0] a_tc,
    input  wire [B_W-1:0] b_tc,
    output reg  [P_W-1:0] p_tc
);

    reg               sign_flag;
    reg [A_W-1:0]     a_mag;
    reg [B_W-1:0]     b_mag;
    reg [P_W-1:0]     p_mag;

    function [A_W-1:0] tc_abs_a;
        input [A_W-1:0] value;
        begin
            if (value[A_W-1]) begin
                tc_abs_a = (~value) + {{(A_W-1){1'b0}}, 1'b1};
            end else begin
                tc_abs_a = value;
            end
        end
    endfunction

    function [B_W-1:0] tc_abs_b;
        input [B_W-1:0] value;
        begin
            if (value[B_W-1]) begin
                tc_abs_b = (~value) + {{(B_W-1){1'b0}}, 1'b1};
            end else begin
                tc_abs_b = value;
            end
        end
    endfunction

    always @(*) begin
        sign_flag = a_tc[A_W-1] ^ b_tc[B_W-1];
        a_mag     = tc_abs_a(a_tc);
        b_mag     = tc_abs_b(b_tc);
        p_mag     = a_mag * b_mag;

        if (sign_flag) begin
            p_tc = (~p_mag) + {{(P_W-1){1'b0}}, 1'b1};
        end else begin
            p_tc = p_mag;
        end
    end

endmodule
