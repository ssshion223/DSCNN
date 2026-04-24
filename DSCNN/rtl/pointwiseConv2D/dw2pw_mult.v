`timescale 1ns / 1ps

//==============================================================================
// 模块名：dw2pw_mult
// 功能说明：
//   Pointwise 计算器顶层。
//   数据路径：输入像素流 -> depthwiseConv2D -> 64通道并行 PW 乘加。
//   本模块只负责计算，不包含分段累加与 RAM 存储。
//
// 典型输入组织：1x25x5x64
//   - 空间像素数：PIXEL_DEPTH = 25 * 5 = 125
//   - 通道帧数：IN_FRAME_SIZE = 64
//==============================================================================
module dw2pw_mult #(
	// ---------------- DW stage ----------------
	parameter integer IN_DATA_W            = 8,
	parameter integer DW_COEFF_W           = 8,
	parameter integer DW_K_H               = 3,
	parameter integer DW_K_W               = 3,
	parameter integer DW_MUL_W             = IN_DATA_W + DW_COEFF_W,
	parameter integer DW_SUM_W             = DW_MUL_W + $clog2(DW_K_H*DW_K_W),
	parameter integer DW_COL               = 5,
	parameter integer DW_ROW               = 25,
	parameter integer DW_STRIDE            = 1,
	parameter integer DW_PAD_TOP           = (DW_K_H-1)/2,
	parameter integer DW_PAD_BOTTOM        = (DW_K_H)/2,
	parameter integer DW_PAD_LEFT          = (DW_K_W-1)/2,
	parameter integer DW_PAD_RIGHT         = (DW_K_W)/2,
	parameter integer DW_COEFF_GRP_NUM     = 64 * 4,//一层计算64通道，复用4层系数
	parameter integer DW_FRAME_GRP_NUM     = 64,
	parameter integer DW_MAC_PIPELINE      = 1,
	parameter         DW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh",
	parameter         DW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw_pingpong_bias.hex",
	parameter integer DW_OUT_WIDTH         = 8,
	parameter integer DW_SHIFT_VAL         = 16,
	parameter integer DW_BIAS_GROUP_BITS   = 2,
	parameter integer DW_BIAS_GROUP_SIZE   = 4, //复用4层系数
	parameter integer DW_BIAS_CH_BITS      = 6, //每层64个BIAS 
 	parameter         DW_MULT_CNT     = 4,  // 量化乘数个数
	parameter signed [11:0] DW_MULT_FACTOR0 = 12'sd915,  // 乘数0
	parameter signed [11:0] DW_MULT_FACTOR1 = 12'sd768,  // 乘数1
	parameter signed [11:0] DW_MULT_FACTOR2 = 12'sd640,  // 乘数2
	parameter signed [11:0] DW_MULT_FACTOR3 = 12'sd512,  // 乘数3
	parameter integer DW_FIFO_DEPTH        = 16,
	parameter integer DW_FIFO_AF_LEVEL     = 10,

	// ---------------- PW matrix stage ----------------
	parameter integer PW_OUT_CH            = 64,
	parameter integer PW_COEFF_W           = 8,
	parameter integer PW_K_H               = 1,
	parameter integer PW_K_W               = 1,
	parameter integer PW_MUL_W             = DW_OUT_WIDTH + PW_COEFF_W,
	parameter integer PW_SUM_W             = PW_MUL_W + $clog2(PW_K_H*PW_K_W),
	parameter integer PW_COL               = DW_COL,
	parameter integer PW_ROW               = DW_ROW,
	parameter integer PW_STRIDE            = 1,
	parameter integer PW_PAD_TOP           = (PW_K_H-1)/2,
	parameter integer PW_PAD_BOTTOM        = (PW_K_H)/2,
	parameter integer PW_PAD_LEFT          = (PW_K_W-1)/2,
	parameter integer PW_PAD_RIGHT         = (PW_K_W)/2,
	parameter integer PW_COEFF_GRP_NUM     = 64*4,//每个frame的64个通道对应一组系数
	parameter integer PW_FRAME_GRP_NUM     = 64,
	parameter integer PW_MAC_PIPELINE      = 1,
	parameter         PW_COEFF_INIT_FILE   =  "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_pw.memh",
	parameter integer PW_FIFO_DEPTH        = 4,
	parameter integer PW_FIFO_AF_LEVEL     = 2

)(
	input  wire                                  clk,
	input  wire                                  rst_n,

	// 输入：按流输入原始特征图
	input  wire                                  in_valid,
	output wire                                  in_ready,
	input  wire signed [IN_DATA_W-1:0]           in_pixel,
	input  wire                                  in_end_all_frame,

	// PW 并行乘加中间结果（64通道并行）
	output wire                                  pw_mult_valid,
	input  wire                                  pw_mult_ready,
	output wire                                  pw_mult_end_frame,
	output wire                                  pw_mult_end_all_frame,
	output wire signed [PW_OUT_CH*PW_SUM_W-1:0]  pw_mult_data_bus
);

	// ---------------- DW -> PW 矩阵阶段握手 ----------------
	wire                             dw_out_valid;
	wire                             dw_out_ready;
	wire                             dw_out_end_frame;
	wire                             dw_out_end_all_frame;
	wire [DW_OUT_WIDTH-1:0]          dw_out_pixel_u;
	wire signed [DW_OUT_WIDTH-1:0]   dw_out_pixel_s;

	// ---------------- PW 矩阵 -> 累加阶段握手 ----------------
	wire                             pw_out_valid_i;
	wire                             pw_out_ready_i;
	wire                             pw_out_end_frame_i;
	wire                             pw_out_end_all_frame_i;
	wire signed [PW_OUT_CH*PW_SUM_W-1:0] pw_out_bus_i;

	assign dw_out_pixel_s       = $signed(dw_out_pixel_u);
	assign pw_mult_valid        = pw_out_valid_i;
	assign pw_mult_end_frame    = pw_out_end_frame_i;
	assign pw_mult_end_all_frame = pw_out_end_all_frame_i;
	assign pw_mult_data_bus     = pw_out_bus_i;
    assign pw_out_ready_i      = pw_mult_ready;

	depthwiseConv2D #(
		.DATA_W(IN_DATA_W),
		.COEFF_W(DW_COEFF_W),
		.K_H(DW_K_H),
		.K_W(DW_K_W),
		.MUL_W(DW_MUL_W),
		.SUM_W(DW_SUM_W),
		.COL(DW_COL),
		.ROW(DW_ROW),
		.STRIDE(DW_STRIDE),
		.PAD_TOP(DW_PAD_TOP),
		.PAD_BOTTOM(DW_PAD_BOTTOM),
		.PAD_LEFT(DW_PAD_LEFT),
		.PAD_RIGHT(DW_PAD_RIGHT),
		.COEFF_GRP_NUM(DW_COEFF_GRP_NUM),
		.FRAME_GRP_NUM(DW_FRAME_GRP_NUM),
		.MAC_PIPELINE(DW_MAC_PIPELINE),
		.COEFF_INIT_FILE(DW_COEFF_INIT_FILE),
		.OUT_WIDTH(DW_OUT_WIDTH),
		.SHIFT_VAL(DW_SHIFT_VAL),
		.BIAS_GROUP_BITS(DW_BIAS_GROUP_BITS),
		.BIAS_GROUP_SIZE(DW_BIAS_GROUP_SIZE),
		.BIAS_CH_BITS(DW_BIAS_CH_BITS),
		.MULT_CNT(DW_MULT_CNT),
		.MULT_FACTOR0(DW_MULT_FACTOR0),
		.MULT_FACTOR1(DW_MULT_FACTOR1),
		.MULT_FACTOR2(DW_MULT_FACTOR2),
		.MULT_FACTOR3(DW_MULT_FACTOR3),
		.FIFO_DEPTH(DW_FIFO_DEPTH),
		.FIFO_AF_LEVEL(DW_FIFO_AF_LEVEL),	
		.BIAS_INIT_FILE(DW_BIAS_INIT_FILE)
	) u_depthwise (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(in_valid),
		.in_ready(in_ready),
		.in_pixel(in_pixel),
		.in_end_all_frame(in_end_all_frame),
		.out_pixel(dw_out_pixel_u),
		.out_valid(dw_out_valid),
		.out_ready(dw_out_ready),
		.out_end_frame(dw_out_end_frame),
		.out_end_all_frame(dw_out_end_all_frame)
	);

	// 使用 matrix_conv2d_stream_parallel 进行 64 通道并行 PW 乘加
	matrix_conv2d_stream_parallel #(
		.DATA_W(DW_OUT_WIDTH),
		.COEFF_W(PW_COEFF_W),
		.K_H(PW_K_H),
		.K_W(PW_K_W),
		.MUL_W(PW_MUL_W),
		.SUM_W(PW_SUM_W),
		.COL(PW_COL),
		.ROW(PW_ROW),
		.STRIDE(PW_STRIDE),
		.PAD_TOP(PW_PAD_TOP),
		.PAD_BOTTOM(PW_PAD_BOTTOM),
		.PAD_LEFT(PW_PAD_LEFT),
		.PAD_RIGHT(PW_PAD_RIGHT),
		.OUT_CH(PW_OUT_CH),
		.COEFF_GRP_NUM(PW_COEFF_GRP_NUM),
		.FRAME_GRP_NUM(PW_FRAME_GRP_NUM),
		.MAC_PIPELINE(PW_MAC_PIPELINE),
		.COEFF_INIT_FILE(PW_COEFF_INIT_FILE),
		.OUT_FIFO_DEPTH(PW_FIFO_DEPTH),
		.OUT_FIFO_AF_LEVEL(PW_FIFO_AF_LEVEL)
	) u_pw_matrix (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(dw_out_valid),
		.in_ready(dw_out_ready),
		.in_pixel(dw_out_pixel_s),
		.in_end_all_frame(dw_out_end_all_frame),
		.out_valid(pw_out_valid_i),
		.out_ready(pw_out_ready_i),
		.out_end_all_frame(pw_out_end_all_frame_i),
		.out_end_frame(pw_out_end_frame_i),
		.out_pixel_data_bus(pw_out_bus_i)
	);

	// test===========================================

	// reg[PW_SUM_W-1:0] test_sum[0:PW_OUT_CH-1];
	// integer i;
	// always @(*) begin
	// 	for(i=0; i<PW_OUT_CH; i=i+1) begin
	// 		test_sum[i] = pw_out_bus_i[i*PW_SUM_W +: PW_SUM_W];
	// 	end
	// end
	// reg [31:0] in_pixel_cnt, dw_out_cnt, pw_out_cnt;
	// always @(posedge clk or negedge rst_n) begin
	// 	if(!rst_n) begin
	// 		in_pixel_cnt <= 0;
	// 		dw_out_cnt <= 0;
	// 		pw_out_cnt <= 0;
	// 	end else begin
	// 		if(in_valid&&in_ready) begin
	// 			$display("beat %d: DW_in_pixel: %d", in_pixel_cnt, $signed(in_pixel));
	// 			in_pixel_cnt <= in_pixel_cnt + 1;
	// 		end
	// 		if(dw_out_valid&&dw_out_ready) begin
	// 			$display("beat %d: DW_out_pixel: %d", dw_out_cnt, $signed(dw_out_pixel_s));
	// 			dw_out_cnt <= dw_out_cnt + 1;	
	// 		end
	// 		if(pw_out_valid_i&&pw_out_ready_i) begin
	// 			$display("beat %d: PW_out_sum[0]: %d , [1]: %d , [2]: %d , [3]: %d", pw_out_cnt, $signed(test_sum[0]), $signed(test_sum[1]), $signed(test_sum[2]), $signed(test_sum[3]));
	// 			$display("beat %d: PW_out_sum[4]: %d , [5]: %d , [6]: %d , [7]: %d", pw_out_cnt, $signed(test_sum[4]), $signed(test_sum[5]), $signed(test_sum[6]), $signed(test_sum[7]));
	// 			pw_out_cnt <= pw_out_cnt + 1;
	// 		end
	// 	end
	// end

	

endmodule
