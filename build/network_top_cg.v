module network_top_cg #(  parameter DS_DATA_W = 8,
  parameter DS_COEFF_W = 8,
  parameter DS_K_H = 10,
  parameter DS_K_W = 4,
  parameter DS_COL = 10,
  parameter DS_ROW = 49,
  parameter DS_STRIDE = 2,
  parameter DS_PAD_TOP = 4,
  parameter DS_PAD_BOTTOM = 5,
  parameter DS_PAD_LEFT = 1,
  parameter DS_PAD_RIGHT = 2,
  parameter DS_COEFF_GRP_NUM = 64,
  parameter DS_OUT_WIDTH = 8,
  parameter DS_MULT_CNT = 1,
  parameter DS_MULT_FACTOR0 = 12'sd915,
  parameter PP_ROUNDS = 3,
  parameter PP_PIXEL_DEPTH = 125,
  parameter PP_IN_FRAME_SIZE = 64,
  parameter PP_DW_K_H = 3,
  parameter PP_DW_K_W = 3,
  parameter PP_OUT_PIXEL_W = 8,
  parameter DS_COEFF_FILE = "./data/weights/DS-CNN_dw0.memh",
  parameter DS_BIAS_FILE = "./data/bias/DS-CNN_dw0_Fold_bias.hex",
  parameter PP_DW_COEFF_FILE = "./data/weights/DS-CNN_pingpong_dw.memh",
  parameter PP_PW_COEFF_FILE = "./data/weights/DS-CNN_pingpong_pw.memh",
  parameter PP_DW_BIAS_FILE = "./data/bias/DS-CNN_dw_pingpong_bias.hex",
  parameter PP_PW_BIAS_FILE = "./data/bias/DS-CNN_pw_pingpong_bias.hex"
)
(
input wire  [DS_DATA_W-1:0] in_pixel,
input wire clk,
input wire rst_n,
input wire start,
input wire in_valid,
input wire out_ready,
output wire  [PP_OUT_PIXEL_W-1:0] out_data,
output wire busy,
output wire in_ready,
output wire out_valid,
output wire out_end_frame,
output wire out_end_all_frame
);
  wire network_top_ctrl_cg2_ds_in_valid;
  wire [DS_DATA_W-1:0] network_top_ctrl_cg2_ds_in_pixel;
  wire network_top_ctrl_cg2_ds_in_end_all_frame;
  wire depthwiseConv2D_cg2_in_ready;
  wire network_top_ctrl_cg2_pp_start;
  wire dw_pw_pingpong_cg3_in_ready;
  wire depthwiseConv2D_cg2_out_valid;
  wire [OUT_WIDTH-1:0] depthwiseConv2D_cg2_out_pixel;
  wire depthwiseConv2D_cg2_out_end_all_frame;

  //----Code starts here: integrated by Robei-----
  	
      
      
      
     
      
      localparam K_H = DS_K_H;
      localparam K_W = DS_K_W;
      localparam IN_DATA_W=DS_OUT_WIDTH;
      localparam MUL_W=DS_DATA_W+DS_COEFF_W;
      localparam SUM_W=MUL_W+$clog2(K_H*K_W);
      localparam DW_COEFF_W=8;
      localparam OUT_WIDTH=DS_OUT_WIDTH;
      localparam BIAS_GROUP_SIZE=1;
      localparam BIAS_GROUP_BITS=$clog2(BIAS_GROUP_SIZE);
      
      
      localparam PINGPONG_ROUNDS=PP_ROUNDS;
      localparam DW_K_H=PP_DW_K_H;
      localparam DW_K_W=PP_DW_K_W;
      localparam DW_MUL_W= DS_OUT_WIDTH + 8 ;
      localparam DW_SUM_W=DW_MUL_W+$clog2(DW_K_H*DW_K_W);
      
      localparam IN_FRAME_SIZE=PP_IN_FRAME_SIZE;
      localparam DW_COEFF_GRP_NUM=IN_FRAME_SIZE*(PINGPONG_ROUNDS+1);
      localparam DW_BIAS_GROUP_SIZE=(PINGPONG_ROUNDS+1);
      localparam DW_OUT_WIDTH =8;
      
      localparam PW_COEFF_W =8;
      localparam PW_K_H=1;
      localparam PW_K_W=1;
      localparam PW_OUT_CH=64;
      
   
      localparam PW_MUL_W= DW_OUT_WIDTH + PW_COEFF_W;
      localparam PW_SUM_W=PW_MUL_W+$clog2(PW_K_H*PW_K_W);
      localparam PW_BIAS_GROUP_SIZE=(PINGPONG_ROUNDS+1);
      localparam PW_BIAS_GROUP_BITS=$clog2(PW_BIAS_GROUP_SIZE);
      
      localparam RAM_ONE_DATA_W=PW_SUM_W + $clog2(PW_OUT_CH);
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
  
  
//---Module instantiation---
  depthwiseConv2D_cg #(
.DATA_W(DS_DATA_W),
.COEFF_W(DS_COEFF_W),
.K_H(DS_K_H),
.K_W(DS_K_W),
.COL(DS_COL),
.ROW(DS_ROW),
.STRIDE(DS_STRIDE),
.COEFF_GRP_NUM(DS_COEFF_GRP_NUM),
.FRAME_GRP_NUM(DS_COEFF_GRP_NUM),
.MAC_PIPELINE(1),
.OUT_WIDTH(DS_OUT_WIDTH),
.SHIFT_VAL(16),
.BIAS_GROUP_SIZE(1),
.BIAS_CH_BITS(6),
.FIFO_DEPTH(16),
.FIFO_AF_LEVEL(10),
.MULT_CNT(DS_MULT_CNT),
.MULT_FACTOR0(DS_MULT_FACTOR0),
.MULT_FACTOR1(12'sd828),
.MULT_FACTOR2(12'sd652),
.MULT_FACTOR3(12'sd412),
.MUL_W(MUL_W),
.SUM_W(SUM_W),
.PAD_TOP(DS_PAD_TOP),
.PAD_BOTTOM(DS_PAD_BOTTOM),
.PAD_LEFT(DS_PAD_LEFT),
.PAD_RIGHT(DS_PAD_RIGHT),
.BIAS_GROUP_BITS(BIAS_GROUP_BITS),
.COEFF_INIT_FILE(DS_COEFF_FILE),
.BIAS_INIT_FILE(DS_BIAS_FILE)
) 
depthwiseConv2D_cg2(
    .in_pixel(network_top_ctrl_cg2_ds_in_pixel),
    .out_pixel(depthwiseConv2D_cg2_out_pixel),
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(network_top_ctrl_cg2_ds_in_valid),
    .out_ready(dw_pw_pingpong_cg3_in_ready),
    .in_end_all_frame(network_top_ctrl_cg2_ds_in_end_all_frame),
    .in_ready(depthwiseConv2D_cg2_in_ready),
    .out_valid(depthwiseConv2D_cg2_out_valid),
    .out_end_frame(),
    .out_end_all_frame(depthwiseConv2D_cg2_out_end_all_frame)
);

  network_top_ctrl_cg #(
.DS_DATA_W(DS_DATA_W),
.DS_COL(DS_COL),
.DS_ROW(DS_ROW),
.DS_COEFF_GRP_NUM(DS_COEFF_GRP_NUM)
) 
network_top_ctrl_cg2(
    .in_pixel(in_pixel),
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .busy(busy),
    .in_valid(in_valid),
    .ds_in_ready(depthwiseConv2D_cg2_in_ready),
    .ds_in_pixel(network_top_ctrl_cg2_ds_in_pixel),
    .in_ready(in_ready),
    .ds_in_valid(network_top_ctrl_cg2_ds_in_valid),
    .ds_in_end_all_frame(network_top_ctrl_cg2_ds_in_end_all_frame),
    .pp_start(network_top_ctrl_cg2_pp_start)
);

  dw_pw_pingpong_cg #(
.PINGPONG_ROUNDS(PP_ROUNDS),
.PIXEL_DEPTH(PP_PIXEL_DEPTH),
.IN_FRAME_SIZE(PP_IN_FRAME_SIZE),
.IN_DATA_W(DS_OUT_WIDTH),
.DW_COEFF_W(8),
.DW_K_H(PP_DW_K_H),
.DW_K_W(PP_DW_K_W),
.DW_MUL_W(IN_DATA_W+DW_COEFF_W),
.DW_SUM_W(DW_MUL_W+$clog2(DW_K_H*DW_K_W)),
.DW_COL(5),
.DW_ROW(25),
.DW_STRIDE(1),
.DW_PAD_TOP((DW_K_H-1)/2),
.DW_PAD_BOTTOM((DW_K_H)/2),
.DW_PAD_LEFT((DW_K_W-1)/2),
.DW_PAD_RIGHT((DW_K_W)/2),
.DW_COEFF_GRP_NUM(IN_FRAME_SIZE*(PINGPONG_ROUNDS+1)),
.DW_FRAME_GRP_NUM(IN_FRAME_SIZE),
.DW_MAC_PIPELINE(1),
.DW_COEFF_INIT_FILE(PP_DW_COEFF_FILE),
.DW_BIAS_INIT_FILE(PP_DW_BIAS_FILE),
.DW_OUT_WIDTH(8),
.DW_SHIFT_VAL(16),
.DW_BIAS_GROUP_SIZE((PINGPONG_ROUNDS+1)),
.DW_BIAS_GROUP_BITS($clog2(DW_BIAS_GROUP_SIZE)),
.DW_BIAS_CH_BITS($clog2(IN_FRAME_SIZE)),
.DW_MULT_CNT((PINGPONG_ROUNDS+1)),
.DW_MULT_FACTOR0(12'sd1246),
.DW_MULT_FACTOR1(12'sd828),
.DW_MULT_FACTOR2(12'sd652),
.DW_MULT_FACTOR3(12'sd412),
.DW_FIFO_DEPTH(16),
.DW_FIFO_AF_LEVEL(10),
.PW_OUT_CH(64),
.PW_COEFF_W(8),
.PW_K_H(1),
.PW_K_W(1),
.PW_MUL_W(DW_OUT_WIDTH+PW_COEFF_W),
.PW_SUM_W(PW_MUL_W+$clog2(PW_K_H*PW_K_W)),
.PW_COEFF_GRP_NUM(PW_OUT_CH*(PINGPONG_ROUNDS+1)),
.PW_FRAME_GRP_NUM(PW_OUT_CH),
.PW_MAC_PIPELINE(1),
.PW_COEFF_INIT_FILE(PP_PW_COEFF_FILE),
.PW_BIAS_INIT_FILE(PP_PW_BIAS_FILE),
.PW_BIAS_GROUP_SIZE((PINGPONG_ROUNDS+1)),
.PW_BIAS_GROUP_BITS($clog2(PW_BIAS_GROUP_SIZE)),
.PW_BIAS_CH_BITS($clog2(PW_OUT_CH)),
.PW_MULT_CNT((PINGPONG_ROUNDS+1)),
.PW_MULT_FACTOR0(12'sd406),
.PW_MULT_FACTOR1(12'sd461),
.PW_MULT_FACTOR2(12'sd442),
.PW_MULT_FACTOR3(12'sd623),
.PW_FIFO_DEPTH(4),
.PW_FIFO_AF_LEVEL(2),
.RAM_SEGMENTS(1),
.RAM_ONE_DATA_W(PW_SUM_W+$clog2(PW_OUT_CH)),
.OUT_PIXEL_WIDTH(PP_OUT_PIXEL_W)
) 
dw_pw_pingpong_cg3(
    .in_pixel(depthwiseConv2D_cg2_out_pixel),
    .out_data(out_data),
    .rst_n(rst_n),
    .clk(clk),
    .in_valid(depthwiseConv2D_cg2_out_valid),
    .out_ready(out_ready),
    .in_end_all_frame(depthwiseConv2D_cg2_out_end_all_frame),
    .start(network_top_ctrl_cg2_pp_start),
    .in_ready(dw_pw_pingpong_cg3_in_ready),
    .out_valid(out_valid),
    .end_frame(out_end_frame),
    .end_all_frame(out_end_all_frame),
    .busy(busy)
);

endmodule    //network_top_cg
