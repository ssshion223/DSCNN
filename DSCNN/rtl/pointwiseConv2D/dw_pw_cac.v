`timescale 1ns / 1ps

//==============================================================================
// 模块名：dw_pw_cac
// 功能说明：
//   组合封装：
//   - 输入侧使用 dw2pw_mult（DW + PW 并行乘加）
//   - 输出侧使用 pw_sum（分段累加并读写 RAM）
//==============================================================================
module dw_pw_cac #(
	// ---------------- dw2pw_mult parameters ----------------
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
	parameter integer DW_COEFF_GRP_NUM     = 64 * 4,
	parameter integer DW_FRAME_GRP_NUM     = 64,
	parameter integer DW_MAC_PIPELINE      = 1,
	parameter         DW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/rtl/weights/DS-CNN_pingpong_dw.memh",
	parameter         DW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/rtl/bias/DS-CNN_dw_pingpong_bias.hex",
	parameter integer DW_OUT_WIDTH         = 8,
	parameter integer DW_SHIFT_VAL         = 16,
 	parameter integer DW_BIAS_GROUP_BITS   = 2,
 	parameter integer DW_BIAS_GROUP_SIZE   = 1*4,
 	parameter integer DW_BIAS_CH_BITS      = 6,
	parameter         DW_MULT_CNT     = 4,  // 量化乘数个数
	parameter signed [11:0] DW_MULT_FACTOR0 = 12'sd1246,  // layer1
	parameter signed [11:0] DW_MULT_FACTOR1 = 12'sd828,  // layer2
	parameter signed [11:0] DW_MULT_FACTOR2 = 12'sd412,  // layer3
	parameter signed [11:0] DW_MULT_FACTOR3 = 12'sd512,  // layer4
	parameter integer DW_FIFO_DEPTH        = 16,
	parameter integer DW_FIFO_AF_LEVEL     = 10,
	parameter integer PW_OUT_CH            = 64,
	parameter integer PW_COEFF_W           = 8,
	parameter integer PW_K_H               = 1,
	parameter integer PW_K_W               = 1,
	parameter integer PW_MUL_W             = DW_OUT_WIDTH + PW_COEFF_W,
	parameter integer PW_SUM_W             = PW_MUL_W + $clog2(PW_K_H*PW_K_W),
	parameter integer PW_COEFF_GRP_NUM     = 64,
	parameter integer PW_FRAME_GRP_NUM     = 64,
	parameter integer PW_MAC_PIPELINE      = 1,
	parameter         PW_COEFF_INIT_FILE   =  "D:/vivado/exp/DSCNN/rtl/weights/DS-CNN_pingpong_pw.memh",
	parameter integer PW_FIFO_DEPTH        = 4,
	parameter integer PW_FIFO_AF_LEVEL     = 2,
	// ---------------- pw_sum parameters ----------------
	parameter integer RAM_ONE_DATA_W       = PW_SUM_W+$clog2(PW_OUT_CH),
	parameter integer SEGMENTS             = 4,
	parameter integer PIXEL_DEPTH          = ((DW_COL + DW_PAD_LEFT + DW_PAD_RIGHT - DW_K_W) / DW_STRIDE + 1) * ((DW_ROW + DW_PAD_TOP + DW_PAD_BOTTOM - DW_K_H) / DW_STRIDE + 1),
	parameter integer RAM_ADDR_W       = $clog2(PIXEL_DEPTH * SEGMENTS),
	parameter integer IN_FRAME_SIZE        = 64
)(
	input  wire                                         clk,
	input  wire                                         rst_n,

	// 上层输入：对接 dw2pw_mult
	input  wire                                         in_valid,
	output wire                                         in_ready,
	input  wire signed [IN_DATA_W-1:0]                  in_pixel,
	input  wire                                         in_end_all_frame,

	// 上层输出：对接 pw_sum
	output wire                                         ram_re,
	output wire [RAM_ADDR_W-1:0]                    ram_raddr,
	input  wire [(PW_OUT_CH/SEGMENTS)*RAM_ONE_DATA_W-1:0]    ram_rdata,

	output wire                                         ram_we,
	output wire [RAM_ADDR_W-1:0]                    ram_waddr,
	output wire [(PW_OUT_CH/SEGMENTS)*RAM_ONE_DATA_W-1:0]    ram_wdata,

	output wire                                         end_frame,
	output wire                                         end_all_frame
);
	localparam RAM_DATA_W = (PW_OUT_CH/SEGMENTS) * RAM_ONE_DATA_W;
	wire                                 pw_mult_valid;
	wire                                 pw_mult_ready;
	wire                                 pw_mult_end_frame;
	wire                                 pw_mult_end_all_frame;
	wire signed [PW_OUT_CH*PW_SUM_W-1:0] pw_mult_data_bus;

	dw2pw_mult #(
		.IN_DATA_W(IN_DATA_W),
		.DW_COEFF_W(DW_COEFF_W),
		.DW_K_H(DW_K_H),
		.DW_K_W(DW_K_W),
		.DW_MUL_W(DW_MUL_W),
		.DW_SUM_W(DW_SUM_W),
		.DW_COL(DW_COL),
		.DW_ROW(DW_ROW),
		.DW_STRIDE(DW_STRIDE),
		.DW_PAD_TOP(DW_PAD_TOP),
		.DW_PAD_BOTTOM(DW_PAD_BOTTOM),
		.DW_PAD_LEFT(DW_PAD_LEFT),
		.DW_PAD_RIGHT(DW_PAD_RIGHT),
		.DW_COEFF_GRP_NUM(DW_COEFF_GRP_NUM),
		.DW_FRAME_GRP_NUM(DW_FRAME_GRP_NUM),
		.DW_MAC_PIPELINE(DW_MAC_PIPELINE),
		.DW_COEFF_INIT_FILE(DW_COEFF_INIT_FILE),
		.DW_OUT_WIDTH(DW_OUT_WIDTH),
		.DW_SHIFT_VAL(DW_SHIFT_VAL),
		.DW_BIAS_GROUP_BITS(DW_BIAS_GROUP_BITS),
		.DW_BIAS_GROUP_SIZE(DW_BIAS_GROUP_SIZE),
		.DW_BIAS_CH_BITS(DW_BIAS_CH_BITS),
		.DW_MULT_CNT(DW_MULT_CNT),
		.DW_MULT_FACTOR0(DW_MULT_FACTOR0),
		.DW_MULT_FACTOR1(DW_MULT_FACTOR1),
		.DW_MULT_FACTOR2(DW_MULT_FACTOR2),
		.DW_MULT_FACTOR3(DW_MULT_FACTOR3),
		.DW_FIFO_DEPTH(DW_FIFO_DEPTH),
		.DW_FIFO_AF_LEVEL(DW_FIFO_AF_LEVEL),
		.PW_OUT_CH(PW_OUT_CH),
		.PW_COEFF_W(PW_COEFF_W),
		.PW_K_H(PW_K_H),
		.PW_K_W(PW_K_W),
		.PW_MUL_W(PW_MUL_W),
		.PW_SUM_W(PW_SUM_W),
		.PW_COEFF_GRP_NUM(PW_COEFF_GRP_NUM),
		.PW_FRAME_GRP_NUM(PW_FRAME_GRP_NUM),
		.PW_MAC_PIPELINE(PW_MAC_PIPELINE),
		.PW_COEFF_INIT_FILE(PW_COEFF_INIT_FILE),
		.PW_FIFO_DEPTH(PW_FIFO_DEPTH),
		.PW_FIFO_AF_LEVEL(PW_FIFO_AF_LEVEL)
	) u_dw2pw_mult (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(in_valid),
		.in_ready(in_ready),
		.in_pixel(in_pixel),
		.in_end_all_frame(in_end_all_frame),
		.pw_mult_valid(pw_mult_valid),
		.pw_mult_ready(pw_mult_ready),
		.pw_mult_end_frame(pw_mult_end_frame),
		.pw_mult_end_all_frame(pw_mult_end_all_frame),
		.pw_mult_data_bus(pw_mult_data_bus)
	);

	pw_sum #(
		.MULT_WIDTH(PW_SUM_W),
		.RAM_DATA_W(RAM_DATA_W),
		.CHANNELS(PW_OUT_CH),
		.SEGMENTS(SEGMENTS),
		.PIXEL_DEPTH(PIXEL_DEPTH),
		.RAM_ADDR_W(RAM_ADDR_W),
		.IN_FRAME_SIZE(IN_FRAME_SIZE)
	) u_pw_sum (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(pw_mult_valid),
		.in_ready(pw_mult_ready),
		.mult_in(pw_mult_data_bus),
		.ram_re(ram_re),
		.ram_raddr(ram_raddr),
		.ram_rdata(ram_rdata),
		.ram_we(ram_we),
		.ram_waddr(ram_waddr),
		.ram_wdata(ram_wdata),
		.end_frame(end_frame),
		.end_all_frame(end_all_frame)
	);

	//test===========================================
	// reg[PW_SUM_W-1:0] test_sum[0:PW_OUT_CH-1];
	// reg[RAM_ONE_DATA_W-1:0] test_ram_wr_data[0:(PW_OUT_CH/SEGMENTS)-1];
	// reg[RAM_ONE_DATA_W-1:0] test_ram_rd_data[0:(PW_OUT_CH/SEGMENTS)-1];
	// integer i;
	// always @(*) begin
	// 	for(i=0; i < PW_OUT_CH; i=i+1) begin
	// 		test_sum[i] = pw_mult_data_bus[i*PW_SUM_W +: PW_SUM_W];
	// 	end
	// 	for(i=0; i < PW_OUT_CH/SEGMENTS; i=i+1) begin
	// 		test_ram_wr_data[i] = ram_wdata[i*RAM_ONE_DATA_W +: RAM_ONE_DATA_W];
	// 	end
	// 	for(i=0; i < PW_OUT_CH/SEGMENTS; i=i+1) begin
	// 		test_ram_rd_data[i] = ram_rdata[i*RAM_ONE_DATA_W +: RAM_ONE_DATA_W];
	// 	end
	// end
	// reg [31:0] in_pixel_cnt, ram_wr_cnt, ram_rd_cnt, pw_out_cnt;
	// reg test_ram_re_align;
	// always @(posedge clk or negedge rst_n) begin
	// 	if(!rst_n) begin
	// 		in_pixel_cnt <= 0;
	// 		ram_wr_cnt <= 0;
	// 		ram_rd_cnt <= 0;
	// 		pw_out_cnt <= 0;
	// 	end else begin
	// 		if(in_valid&&in_ready) begin
	// 			$display("beat %d: DW_in_pixel: %h", in_pixel_cnt, in_pixel);
	// 			in_pixel_cnt <= in_pixel_cnt + 1;
	// 		end
	// 		test_ram_re_align <= ram_re;
	// 		if(pw_mult_valid && pw_mult_ready) begin
	// 			$display("beat %d: PW_out_sum[0]: %h,[1]: %h,[2]: %h,[3]: %h", pw_out_cnt, test_sum[0], test_sum[1], test_sum[2], test_sum[3]);
	// 			pw_out_cnt <= pw_out_cnt + 1;
	// 		end
	// 		if(test_ram_re_align) begin
	// 		 $display("beat %d: RAM_Read_Addr: %d,   Data[0]: %d,[1]: %d,[2]: %d,[3]: %d", ram_rd_cnt, ram_raddr, $signed(test_ram_rd_data[0]), $signed(test_ram_rd_data[1]), $signed(test_ram_rd_data[2]), $signed(test_ram_rd_data[3]));
	// 		 ram_rd_cnt <= ram_rd_cnt + 1;
	// 		end
	// 		if(ram_we) begin
	// 		 $display("beat %d: RAM_Write_Addr: %d,   Data[0]: %d,[1]: %d,[2]: %d,[3]: %d", ram_wr_cnt, ram_waddr, $signed(test_ram_wr_data[0]), $signed(test_ram_wr_data[1]), $signed(test_ram_wr_data[2]), $signed(test_ram_wr_data[3]));
	// 		 ram_wr_cnt <= ram_wr_cnt + 1;
	// 		end
	// 	end
	// end


endmodule