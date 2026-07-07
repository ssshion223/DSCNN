`timescale 1ns / 1ps
module mfcc10_log2_lut_q16 (
    input  wire [3:0]  addr,
    output reg  signed [17:0] y0_q16,
    output reg  signed [17:0] dy_q16
);

    always @(*) begin
        case (addr)
            4'd0:  begin y0_q16 = -18'sd65536; dy_q16 = 18'sd5732; end
            4'd1:  begin y0_q16 = -18'sd59804; dy_q16 = 18'sd5404; end
            4'd2:  begin y0_q16 = -18'sd54400; dy_q16 = 18'sd5112; end
            4'd3:  begin y0_q16 = -18'sd49288; dy_q16 = 18'sd4850; end
            4'd4:  begin y0_q16 = -18'sd44438; dy_q16 = 18'sd4613; end
            4'd5:  begin y0_q16 = -18'sd39825; dy_q16 = 18'sd4398; end
            4'd6:  begin y0_q16 = -18'sd35427; dy_q16 = 18'sd4203; end
            4'd7:  begin y0_q16 = -18'sd31224; dy_q16 = 18'sd4024; end
            4'd8:  begin y0_q16 = -18'sd27200; dy_q16 = 18'sd3860; end
            4'd9:  begin y0_q16 = -18'sd23340; dy_q16 = 18'sd3708; end
            4'd10: begin y0_q16 = -18'sd19632; dy_q16 = 18'sd3568; end
            4'd11: begin y0_q16 = -18'sd16064; dy_q16 = 18'sd3439; end
            4'd12: begin y0_q16 = -18'sd12625; dy_q16 = 18'sd3318; end
            4'd13: begin y0_q16 = -18'sd9307;  dy_q16 = 18'sd3205; end
            4'd14: begin y0_q16 = -18'sd6102;  dy_q16 = 18'sd3100; end
            default: begin y0_q16 = -18'sd3002; dy_q16 = 18'sd3002; end
        endcase
    end

endmodule


