module inference_cg #(  parameter DS_DATA_W = 8,
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
  parameter POOL_DATA_W = 8,
  parameter POOL_ROW_LEN = 25,
  parameter POOL_ROW_NUM = 5,
  parameter POOL_CHANNELS = 64,
  parameter POOL_SUM_W = 16,
  parameter FC_DATA_W = 8,
  parameter FC_IN_CH = 64,
  parameter FC_OUT_CLASS = 12,
  parameter FC_WEIGHT_W = 8,
  parameter FC_BIAS_W = 32,
  parameter FC_ACC_W = 40,
  parameter FC_Q_MUL = 239,
  parameter FC_Q_SHIFT = 16,
  parameter FC_Q_OFFSET = 128,
  parameter DS_COEFF_FILE = "./data/weights/DS-CNN_dw0.memh",
  parameter DS_BIAS_FILE = "./data/bias/DS-CNN_dw0_Fold_bias.hex",
  parameter PP_DW_COEFF_FILE = "./data/weights/DS-CNN_pingpong_dw.memh",
  parameter PP_PW_COEFF_FILE = "./data/weights/DS-CNN_pingpong_pw.memh",
  parameter PP_DW_BIAS_FILE = "./data/bias/DS-CNN_dw_pingpong_bias.hex",
  parameter PP_PW_BIAS_FILE = "./data/bias/DS-CNN_pw_pingpong_bias.hex",
  parameter FC_WEIGHT_FILE = "./data/DS-CNN_fc.memh",
  parameter FC_BIAS_FILE = "./data/DS-CNN_fc_bias.memh"
)
(
input wire  [DS_DATA_W-1:0] in_pixel,
input wire clk,
input wire rst_n,
input wire start,
input wire in_valid,
input wire out_ready,
output wire  [7:0] out_class,
output wire  [FC_ACC_W-1:0] out_score,
output wire busy,
output wire in_ready,
output wire out_valid
);
  wire [PP_OUT_PIXEL_W-1:0] network_top_cg1_out_data;
  wire network_top_cg1_out_valid;
  wire network_top_cg1_out_end_all_frame;
  wire network_top_cg1_out_end_frame;
  wire [DATA_W-1:0] global_avg_pool_stream_cg3_out_data;
  wire global_avg_pool_stream_cg3_in_ready;
  wire fc_argmax_stream_cg3_in_ready;
  wire global_avg_pool_stream_cg3_out_valid;

  //----Code starts here: integrated by Robei-----
  	localparam DATA_W = POOL_DATA_W;
      
      
    
  
  
//---Module instantiation---
  network_top_cg #(
.DS_DATA_W(DS_DATA_W),
.DS_COEFF_W(DS_COEFF_W),
.DS_K_H(DS_K_H),
.DS_K_W(DS_K_W),
.DS_COL(DS_COL),
.DS_ROW(DS_ROW),
.DS_STRIDE(DS_STRIDE),
.DS_PAD_TOP(DS_PAD_TOP),
.DS_PAD_BOTTOM(DS_PAD_BOTTOM),
.DS_PAD_LEFT(DS_PAD_LEFT),
.DS_PAD_RIGHT(DS_PAD_RIGHT),
.DS_COEFF_GRP_NUM(DS_COEFF_GRP_NUM),
.DS_OUT_WIDTH(DS_OUT_WIDTH),
.DS_MULT_CNT(DS_MULT_CNT),
.DS_MULT_FACTOR0(DS_MULT_FACTOR0),
.PP_ROUNDS(PP_ROUNDS),
.PP_PIXEL_DEPTH(PP_PIXEL_DEPTH),
.PP_IN_FRAME_SIZE(PP_IN_FRAME_SIZE),
.PP_DW_K_H(PP_DW_K_H),
.PP_DW_K_W(PP_DW_K_W),
.PP_OUT_PIXEL_W(PP_OUT_PIXEL_W),
.DS_COEFF_FILE(DS_COEFF_FILE),
.DS_BIAS_FILE(DS_BIAS_FILE),
.PP_DW_COEFF_FILE(PP_DW_COEFF_FILE),
.PP_PW_COEFF_FILE(PP_PW_COEFF_FILE),
.PP_DW_BIAS_FILE(PP_DW_BIAS_FILE),
.PP_PW_BIAS_FILE(PP_PW_BIAS_FILE)
) 
network_top_cg1(
    .in_pixel(in_pixel),
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .in_valid(in_valid),
    .out_ready(global_avg_pool_stream_cg3_in_ready),
    .out_data(network_top_cg1_out_data),
    .busy(busy),
    .in_ready(in_ready),
    .out_valid(network_top_cg1_out_valid),
    .out_end_frame(network_top_cg1_out_end_frame),
    .out_end_all_frame(network_top_cg1_out_end_all_frame)
);

  fc_argmax_stream_cg #(
.DATA_W(FC_DATA_W),
.IN_CH(FC_IN_CH),
.OUT_CLASS(FC_OUT_CLASS),
.WEIGHT_W(FC_WEIGHT_W),
.BIAS_W(FC_BIAS_W),
.ACC_W(FC_ACC_W),
.Q_MUL(FC_Q_MUL),
.Q_SHIFT(FC_Q_SHIFT),
.Q_OFFSET(FC_Q_OFFSET),
.WEIGHT_INIT_FILE(FC_WEIGHT_FILE),
.BIAS_INIT_FILE(FC_BIAS_FILE)
) 
fc_argmax_stream_cg3(
    .in_data(global_avg_pool_stream_cg3_out_data),
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(global_avg_pool_stream_cg3_out_valid),
    .out_ready(out_ready),
    .out_class(out_class),
    .out_score(out_score),
    .in_ready(fc_argmax_stream_cg3_in_ready),
    .out_valid(out_valid)
);

  global_avg_pool_stream_cg #(
.DATA_W(POOL_DATA_W),
.ROW_LEN(POOL_ROW_LEN),
.ROW_NUM(POOL_ROW_NUM),
.CHANNELS(POOL_CHANNELS),
.SUM_W(POOL_SUM_W)
) 
global_avg_pool_stream_cg3(
    .in_data(network_top_cg1_out_data),
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(network_top_cg1_out_valid),
    .in_channel_last(network_top_cg1_out_end_frame),
    .in_frame_last(network_top_cg1_out_end_all_frame),
    .out_ready(fc_argmax_stream_cg3_in_ready),
    .out_data(global_avg_pool_stream_cg3_out_data),
    .in_ready(global_avg_pool_stream_cg3_in_ready),
    .out_valid(global_avg_pool_stream_cg3_out_valid)
);

endmodule    //inference_cg
