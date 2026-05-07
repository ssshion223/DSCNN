module dw_pw_pingpong_cg #(  parameter PINGPONG_ROUNDS = 3,
  parameter PIXEL_DEPTH = 125,
  parameter IN_FRAME_SIZE = 64,
  parameter IN_DATA_W = 8,
  parameter DW_COEFF_W = 8,
  parameter DW_K_H = 3,
  parameter DW_K_W = 3,
  parameter DW_MUL_W = IN_DATA_W+DW_COEFF_W,
  parameter DW_SUM_W = DW_MUL_W+$clog2(DW_K_H*DW_K_W),
  parameter DW_COL = 5,
  parameter DW_ROW = 25,
  parameter DW_STRIDE = 1,
  parameter DW_PAD_TOP = (DW_K_H-1)/2,
  parameter DW_PAD_BOTTOM = (DW_K_H)/2,
  parameter DW_PAD_LEFT = (DW_K_W-1)/2,
  parameter DW_PAD_RIGHT = (DW_K_W)/2,
  parameter DW_COEFF_GRP_NUM = IN_FRAME_SIZE*(PINGPONG_ROUNDS+1),
  parameter DW_FRAME_GRP_NUM = IN_FRAME_SIZE,
  parameter DW_MAC_PIPELINE = 1,
  parameter DW_COEFF_INIT_FILE = "",
  parameter DW_BIAS_INIT_FILE = "",
  parameter DW_OUT_WIDTH = 8,
  parameter DW_SHIFT_VAL = 16,
  parameter DW_BIAS_GROUP_SIZE = (PINGPONG_ROUNDS+1),
  parameter DW_BIAS_GROUP_BITS = $clog2(DW_BIAS_GROUP_SIZE),
  parameter DW_BIAS_CH_BITS = $clog2(IN_FRAME_SIZE),
  parameter DW_MULT_CNT = (PINGPONG_ROUNDS+1),
  parameter DW_MULT_FACTOR0 = 12'sd1246,
  parameter DW_MULT_FACTOR1 = 12'sd828,
  parameter DW_MULT_FACTOR2 = 12'sd652,
  parameter DW_MULT_FACTOR3 = 12'sd412,
  parameter DW_FIFO_DEPTH = 16,
  parameter DW_FIFO_AF_LEVEL = 10,
  parameter PW_OUT_CH = 64,
  parameter PW_COEFF_W = 8,
  parameter PW_K_H = 1,
  parameter PW_K_W = 1,
  parameter PW_MUL_W = DW_OUT_WIDTH+PW_COEFF_W,
  parameter PW_SUM_W = PW_MUL_W+$clog2(PW_K_H*PW_K_W),
  parameter PW_COEFF_GRP_NUM = PW_OUT_CH*(PINGPONG_ROUNDS+1),
  parameter PW_FRAME_GRP_NUM = PW_OUT_CH,
  parameter PW_MAC_PIPELINE = 1,
  parameter PW_COEFF_INIT_FILE = "",
  parameter PW_BIAS_INIT_FILE = "",
  parameter PW_BIAS_GROUP_SIZE = (PINGPONG_ROUNDS+1),
  parameter PW_BIAS_GROUP_BITS = $clog2(PW_BIAS_GROUP_SIZE),
  parameter PW_BIAS_CH_BITS = $clog2(PW_OUT_CH),
  parameter PW_MULT_CNT = (PINGPONG_ROUNDS+1),
  parameter PW_MULT_FACTOR0 = 12'sd406,
  parameter PW_MULT_FACTOR1 = 12'sd461,
  parameter PW_MULT_FACTOR2 = 12'sd442,
  parameter PW_MULT_FACTOR3 = 12'sd623,
  parameter PW_FIFO_DEPTH = 4,
  parameter PW_FIFO_AF_LEVEL = 2,
  parameter RAM_SEGMENTS = 1,
  parameter RAM_ONE_DATA_W = PW_SUM_W+$clog2(PW_OUT_CH),
  parameter OUT_PIXEL_WIDTH = 8
)
(
input wire  [IN_DATA_W-1:0] in_pixel,
output wire  [OUT_PIXEL_WIDTH-1:0] out_data,
input wire rst_n,
input wire clk,
input wire in_valid,
input wire out_ready,
input wire in_end_all_frame,
input wire start,
output wire in_ready,
output wire out_valid,
output wire end_frame,
output wire end_all_frame,
output wire busy
);
  wire dw_pw_cac_cg1_in_ready;
  wire dw_pw_pingpong_ctrl_cg3_pw_conv_in_valid;
  wire [IN_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_pw_conv_in_pixel;
  wire dw_pw_pingpong_ctrl_cg3_pw_conv_in_end_all_frame;
  wire dw_pw_cac_cg1_ram_re;
  wire [RAM_ADDR_W-1:0] dw_pw_cac_cg1_ram_raddr;
  wire dw_pw_cac_cg1_ram_we;
  wire [RAM_ADDR_W-1:0] dw_pw_cac_cg1_ram_waddr;
  wire [RAM_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_pw_conv_ram_rdata;
  wire dw_pw_cac_cg1_end_frame;
  wire dw_pw_cac_cg1_end_all_frame;
  wire [(PW_OUT_CH/SEGMENTS)*RAM_ONE_DATA_W-1:0] dw_pw_cac_cg1_ram_wdata;
  wire dw_pw_pingpong_ctrl_cg3_ram_reader_start;
  wire ram_frame_to_fifo_reader_cg4_busy;
  wire ram_frame_to_fifo_reader_cg4_ram_re;
  wire [RAM_ADDR_W-1:0] ram_frame_to_fifo_reader_cg4_ram_raddr;
  wire [RAM_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_ram_reader_ram_rdata;
  wire [OUT_W-1:0] ram_frame_to_fifo_reader_cg4_out_data;
  wire ram_frame_to_fifo_reader_cg4_out_valid;
  wire dw_pw_pingpong_ctrl_cg3_ram_reader_out_ready;
  wire ram_frame_to_fifo_reader_cg4_end_frame;
  wire ram_frame_to_fifo_reader_cg4_end_all_frame;
  wire [RAM_ONE_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_bias_process_in_data;
  wire dw_pw_pingpong_ctrl_cg3_bias_process_in_valid;
  wire dw_pw_pingpong_ctrl_cg3_bias_process_in_end_frame;
  wire dw_pw_pingpong_ctrl_cg3_bias_process_in_end_all_frame;
  wire bias_process_wrapper_cg2_in_ready;
  wire bias_process_wrapper_cg2_out_valid;
  wire bias_process_wrapper_cg2_out_end_frame;
  wire bias_process_wrapper_cg2_out_end_all_frame;
  wire dw_pw_pingpong_ctrl_cg3_bias_process_out_ready;
  wire [RAM_ADDR_W-1:0] dw_pw_pingpong_ctrl_cg3_ram1_raddr;
  wire [DATA_WIDTH-1:0] psum_ram_cg6_rdata;
  wire [RAM_ADDR_W-1:0] dw_pw_pingpong_ctrl_cg3_ram1_waddr;
  wire [RAM_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_ram1_wdata;
  wire dw_pw_pingpong_ctrl_cg3_ram1_re;
  wire dw_pw_pingpong_ctrl_cg3_ram1_we;
  wire dw_pw_pingpong_ctrl_cg3_ram0_we;
  wire [DATA_WIDTH-1:0] psum_ram_cg5_rdata;
  wire [RAM_ADDR_W-1:0] dw_pw_pingpong_ctrl_cg3_ram0_raddr;
  wire [RAM_DATA_W-1:0] dw_pw_pingpong_ctrl_cg3_ram0_wdata;
  wire dw_pw_pingpong_ctrl_cg3_ram0_re;
  wire [RAM_ADDR_W-1:0] dw_pw_pingpong_ctrl_cg3_ram0_waddr;
  wire [OUT_WIDTH-1:0] bias_process_wrapper_cg2_out_pixel_data;

  //----Code starts here: integrated by Robei-----
  
      localparam integer SEG_CHANS        = PW_OUT_CH / RAM_SEGMENTS;
      localparam integer RAM_DATA_W       = SEG_CHANS * RAM_ONE_DATA_W;
      localparam integer RAM_DEPTH        = PIXEL_DEPTH * RAM_SEGMENTS;
      localparam integer RAM_ADDR_W       = (RAM_DEPTH <= 1) ? 1 : $clog2(RAM_DEPTH);
  	localparam integer OUT_WIDTH		   = OUT_PIXEL_WIDTH;
      localparam integer OUT_W			   = RAM_ONE_DATA_W;
      localparam integer SEGMENTS		   = RAM_SEGMENTS;
      localparam integer DATA_WIDTH	   = RAM_DATA_W; 
  
//---Module instantiation---
  dw_pw_cac_cg #(
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
.DW_BIAS_INIT_FILE(DW_BIAS_INIT_FILE),
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
.PW_FIFO_AF_LEVEL(PW_FIFO_AF_LEVEL),
.RAM_ONE_DATA_W(RAM_ONE_DATA_W),
.SEGMENTS(RAM_SEGMENTS),
.PIXEL_DEPTH(PIXEL_DEPTH),
.RAM_ADDR_W(RAM_ADDR_W),
.IN_FRAME_SIZE(IN_FRAME_SIZE)
) 
dw_pw_cac_cg1(
    .in_pixel(dw_pw_pingpong_ctrl_cg3_pw_conv_in_pixel),
    .ram_raddr(dw_pw_cac_cg1_ram_raddr),
    .ram_waddr(dw_pw_cac_cg1_ram_waddr),
    .in_valid(dw_pw_pingpong_ctrl_cg3_pw_conv_in_valid),
    .rst_n(rst_n),
    .clk(clk),
    .in_end_all_frame(dw_pw_pingpong_ctrl_cg3_pw_conv_in_end_all_frame),
    .ram_rdata(dw_pw_pingpong_ctrl_cg3_pw_conv_ram_rdata),
    .in_ready(dw_pw_cac_cg1_in_ready),
    .ram_re(dw_pw_cac_cg1_ram_re),
    .ram_wdata(dw_pw_cac_cg1_ram_wdata),
    .ram_we(dw_pw_cac_cg1_ram_we),
    .end_all_frame(dw_pw_cac_cg1_end_all_frame),
    .end_frame(dw_pw_cac_cg1_end_frame)
);

  bias_process_wrapper_cg #(
.IN_WIDTH(RAM_ONE_DATA_W),
.BIAS_WIDTH(32),
.OUT_WIDTH(OUT_PIXEL_WIDTH),
.SHIFT_VAL(DW_SHIFT_VAL),
.GROUP_BITS(PW_BIAS_GROUP_BITS),
.CH_BITS(PW_BIAS_CH_BITS),
.FIFO_DEPTH(16),
.FIFO_AF_LEVEL(14),
.MULT_CNT(PW_MULT_CNT),
.MULT_FACTOR0(PW_MULT_FACTOR0),
.MULT_FACTOR1(PW_MULT_FACTOR1),
.MULT_FACTOR2(PW_MULT_FACTOR2),
.MULT_FACTOR3(PW_MULT_FACTOR3),
.GROUP_SIZE(PW_BIAS_GROUP_SIZE),
.BIAS_INIT_FILE(PW_BIAS_INIT_FILE)
) 
bias_process_wrapper_cg2(
    .in_pixel_data_bus(dw_pw_pingpong_ctrl_cg3_bias_process_in_data),
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(dw_pw_pingpong_ctrl_cg3_bias_process_in_valid),
    .in_end_frame(dw_pw_pingpong_ctrl_cg3_bias_process_in_end_frame),
    .in_end_all_frame(dw_pw_pingpong_ctrl_cg3_bias_process_in_end_all_frame),
    .out_ready(dw_pw_pingpong_ctrl_cg3_bias_process_out_ready),
    .out_pixel_data(bias_process_wrapper_cg2_out_pixel_data),
    .in_ready(bias_process_wrapper_cg2_in_ready),
    .out_valid(bias_process_wrapper_cg2_out_valid),
    .out_end_frame(bias_process_wrapper_cg2_out_end_frame),
    .out_end_all_frame(bias_process_wrapper_cg2_out_end_all_frame)
);

  dw_pw_pingpong_ctrl_cg #(
.PINGPONG_ROUNDS(PINGPONG_ROUNDS),
.IN_DATA_W(IN_DATA_W),
.RAM_DATA_W(RAM_DATA_W),
.RAM_ADDR_W(RAM_ADDR_W),
.RAM_ONE_DATA_W(RAM_ONE_DATA_W),
.OUT_PIXEL_WIDTH(OUT_PIXEL_WIDTH)
) 
dw_pw_pingpong_ctrl_cg3(
    .in_pixel(in_pixel),
    .pw_conv_ram_raddr(dw_pw_cac_cg1_ram_raddr),
    .pw_conv_ram_waddr(dw_pw_cac_cg1_ram_waddr),
    .pw_conv_ram_wdata(dw_pw_cac_cg1_ram_wdata),
    .ram0_rdata(psum_ram_cg5_rdata),
    .ram1_rdata(psum_ram_cg6_rdata),
    .ram_reader_ram_raddr(ram_frame_to_fifo_reader_cg4_ram_raddr),
    .ram_reader_out_data(ram_frame_to_fifo_reader_cg4_out_data),
    .bias_process_out_data(bias_process_wrapper_cg2_out_pixel_data),
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .in_valid(in_valid),
    .in_end_all_frame(in_end_all_frame),
    .out_ready(out_ready),
    .pw_conv_in_ready(dw_pw_cac_cg1_in_ready),
    .pw_conv_end_frame(dw_pw_cac_cg1_end_frame),
    .pw_conv_end_all_frame(dw_pw_cac_cg1_end_all_frame),
    .pw_conv_ram_re(dw_pw_cac_cg1_ram_re),
    .pw_conv_ram_we(dw_pw_cac_cg1_ram_we),
    .ram_reader_busy(ram_frame_to_fifo_reader_cg4_busy),
    .ram_reader_ram_re(ram_frame_to_fifo_reader_cg4_ram_re),
    .ram_reader_out_valid(ram_frame_to_fifo_reader_cg4_out_valid),
    .ram_reader_end_frame(ram_frame_to_fifo_reader_cg4_end_frame),
    .ram_reader_end_all_frame(ram_frame_to_fifo_reader_cg4_end_all_frame),
    .bias_process_in_ready(bias_process_wrapper_cg2_in_ready),
    .bias_process_out_valid(bias_process_wrapper_cg2_out_valid),
    .bias_process_out_end_frame(bias_process_wrapper_cg2_out_end_frame),
    .bias_process_out_end_all_frame(bias_process_wrapper_cg2_out_end_all_frame),
    .out_data(out_data),
    .pw_conv_in_pixel(dw_pw_pingpong_ctrl_cg3_pw_conv_in_pixel),
    .pw_conv_ram_rdata(dw_pw_pingpong_ctrl_cg3_pw_conv_ram_rdata),
    .ram0_raddr(dw_pw_pingpong_ctrl_cg3_ram0_raddr),
    .ram0_waddr(dw_pw_pingpong_ctrl_cg3_ram0_waddr),
    .ram0_wdata(dw_pw_pingpong_ctrl_cg3_ram0_wdata),
    .ram1_raddr(dw_pw_pingpong_ctrl_cg3_ram1_raddr),
    .ram1_waddr(dw_pw_pingpong_ctrl_cg3_ram1_waddr),
    .ram1_wdata(dw_pw_pingpong_ctrl_cg3_ram1_wdata),
    .ram_reader_ram_rdata(dw_pw_pingpong_ctrl_cg3_ram_reader_ram_rdata),
    .bias_process_in_data(dw_pw_pingpong_ctrl_cg3_bias_process_in_data),
    .busy(busy),
    .in_ready(in_ready),
    .out_valid(out_valid),
    .out_end_frame(end_frame),
    .out_end_all_frame(end_all_frame),
    .pw_conv_in_valid(dw_pw_pingpong_ctrl_cg3_pw_conv_in_valid),
    .pw_conv_in_end_all_frame(dw_pw_pingpong_ctrl_cg3_pw_conv_in_end_all_frame),
    .ram0_re(dw_pw_pingpong_ctrl_cg3_ram0_re),
    .ram0_we(dw_pw_pingpong_ctrl_cg3_ram0_we),
    .ram1_re(dw_pw_pingpong_ctrl_cg3_ram1_re),
    .ram1_we(dw_pw_pingpong_ctrl_cg3_ram1_we),
    .ram_reader_start(dw_pw_pingpong_ctrl_cg3_ram_reader_start),
    .ram_reader_out_ready(dw_pw_pingpong_ctrl_cg3_ram_reader_out_ready),
    .bias_process_in_valid(dw_pw_pingpong_ctrl_cg3_bias_process_in_valid),
    .bias_process_in_end_frame(dw_pw_pingpong_ctrl_cg3_bias_process_in_end_frame),
    .bias_process_in_end_all_frame(dw_pw_pingpong_ctrl_cg3_bias_process_in_end_all_frame),
    .bias_process_out_ready(dw_pw_pingpong_ctrl_cg3_bias_process_out_ready)
);

  ram_frame_to_fifo_reader_cg #(
.PIXELS_PER_FRAME(PIXEL_DEPTH),
.SEGMENTS(RAM_SEGMENTS),
.CHANNELS(PW_OUT_CH),
.RAM_DATA_W(RAM_DATA_W),
.RAM_ADDR_W(RAM_ADDR_W),
.FIFO_DEPTH(16),
.FIFO_AF_LEVEL(10),
.OUT_W(RAM_ONE_DATA_W)
) 
ram_frame_to_fifo_reader_cg4(
    .ram_rdata(dw_pw_pingpong_ctrl_cg3_ram_reader_ram_rdata),
    .ram_raddr(ram_frame_to_fifo_reader_cg4_ram_raddr),
    .out_data(ram_frame_to_fifo_reader_cg4_out_data),
    .clk(clk),
    .rst_n(rst_n),
    .start(dw_pw_pingpong_ctrl_cg3_ram_reader_start),
    .out_ready(dw_pw_pingpong_ctrl_cg3_ram_reader_out_ready),
    .busy(ram_frame_to_fifo_reader_cg4_busy),
    .ram_re(ram_frame_to_fifo_reader_cg4_ram_re),
    .out_valid(ram_frame_to_fifo_reader_cg4_out_valid),
    .end_frame(ram_frame_to_fifo_reader_cg4_end_frame),
    .end_all_frame(ram_frame_to_fifo_reader_cg4_end_all_frame)
);

  psum_ram_cg #(
.DATA_WIDTH(RAM_DATA_W),
.ADDR_WIDTH(RAM_ADDR_W),
.DEPTH(RAM_DEPTH)
) 
psum_ram_cg5(
    .raddr(dw_pw_pingpong_ctrl_cg3_ram0_raddr),
    .waddr(dw_pw_pingpong_ctrl_cg3_ram0_waddr),
    .wdata(dw_pw_pingpong_ctrl_cg3_ram0_wdata),
    .clk(clk),
    .re(dw_pw_pingpong_ctrl_cg3_ram0_re),
    .we(dw_pw_pingpong_ctrl_cg3_ram0_we),
    .rdata(psum_ram_cg5_rdata)
);

  psum_ram_cg #(
.DATA_WIDTH(RAM_DATA_W),
.ADDR_WIDTH(RAM_ADDR_W),
.DEPTH(RAM_DEPTH)
) 
psum_ram_cg6(
    .raddr(dw_pw_pingpong_ctrl_cg3_ram1_raddr),
    .waddr(dw_pw_pingpong_ctrl_cg3_ram1_waddr),
    .wdata(dw_pw_pingpong_ctrl_cg3_ram1_wdata),
    .clk(clk),
    .re(dw_pw_pingpong_ctrl_cg3_ram1_re),
    .we(dw_pw_pingpong_ctrl_cg3_ram1_we),
    .rdata(psum_ram_cg6_rdata)
);

endmodule    //dw_pw_pingpong_cg
