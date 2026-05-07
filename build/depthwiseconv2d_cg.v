module depthwiseConv2D_cg #(  parameter DATA_W = 8,
  parameter COEFF_W = 8,
  parameter K_H = 3,
  parameter K_W = 3,
  parameter COL = 49,
  parameter ROW = 10,
  parameter STRIDE = 2,
  parameter COEFF_GRP_NUM = 64,
  parameter FRAME_GRP_NUM = 64,
  parameter MAC_PIPELINE = 1,
  parameter OUT_WIDTH = 8,
  parameter SHIFT_VAL = 16,
  parameter BIAS_GROUP_SIZE = 1,
  parameter BIAS_CH_BITS = 6,
  parameter FIFO_DEPTH = 16,
  parameter FIFO_AF_LEVEL = 10,
  parameter MULT_CNT = 4,
  parameter MULT_FACTOR0 = 12'sd1246,
  parameter MULT_FACTOR1 = 12'sd828,
  parameter MULT_FACTOR2 = 12'sd652,
  parameter MULT_FACTOR3 = 12'sd412,
  parameter MUL_W = DATA_W+COEFF_W,
  parameter SUM_W = MUL_W+$clog2(K_H*K_W),
  parameter PAD_TOP = (K_H-1)/2,
  parameter PAD_BOTTOM = K_H/2,
  parameter PAD_LEFT = (K_W-1)/2,
  parameter PAD_RIGHT = K_W/2,
  parameter BIAS_GROUP_BITS = $clog2(BIAS_GROUP_SIZE),
  parameter COEFF_INIT_FILE = "",
  parameter BIAS_INIT_FILE = ""
)
(
input wire  [DATA_W-1:0] in_pixel,
output wire  [OUT_WIDTH-1:0] out_pixel,
input wire clk,
input wire rst_n,
input wire in_valid,
input wire out_ready,
input wire in_end_all_frame,
output wire in_ready,
output wire out_valid,
output wire out_end_frame,
output wire out_end_all_frame
);

  //----Code starts here: integrated by Robei-----
  
      wire                               conv_out_valid;
      wire                               conv_out_ready;
      wire                               conv_out_end_all_frame;
      wire                               conv_out_end_frame;
      wire signed [DATA_W-1:0]           in_pixel_s = $signed(in_pixel);
      wire signed [SUM_W-1:0]            conv_out_pixel_data_bus;
  
      
      
      
      matrix_conv2d_stream_parallel #(
          .DATA_W          (DATA_W),
          .COEFF_W         (COEFF_W),
          .SUM_W           (SUM_W),
          .COL             (COL),
          .ROW             (ROW),
          .K_H             (K_H),
          .K_W             (K_W),
          .STRIDE          (STRIDE),
          .PAD_TOP         (PAD_TOP),
          .PAD_BOTTOM      (PAD_BOTTOM),
          .PAD_LEFT        (PAD_LEFT),
          .PAD_RIGHT       (PAD_RIGHT),
          .OUT_CH          (1),
          .COEFF_GRP_NUM   (COEFF_GRP_NUM),
          .FRAME_GRP_NUM   (FRAME_GRP_NUM),
          .MAC_PIPELINE    (MAC_PIPELINE),
          .COEFF_INIT_FILE (COEFF_INIT_FILE),
          .OUT_FIFO_DEPTH  (FIFO_DEPTH),
          .OUT_FIFO_AF_LEVEL(FIFO_AF_LEVEL)
      ) u_conv2d (
          .clk                 (clk),
          .rst_n               (rst_n),
          
          .in_valid            (in_valid),
          .in_ready            (in_ready),
          .in_pixel            (in_pixel_s),
          .in_end_all_frame    (in_end_all_frame),
          
          
          .out_valid           (conv_out_valid),
          .out_ready           (conv_out_ready),
          .out_end_all_frame   (conv_out_end_all_frame),
          .out_end_frame       (conv_out_end_frame),
          .out_pixel_data_bus  (conv_out_pixel_data_bus)
      );
  
      
      
      
      bias_process_wrapper #(
          .IN_WIDTH      (SUM_W), 
          .OUT_WIDTH     (OUT_WIDTH),
          .SHIFT_VAL     (SHIFT_VAL),
          .GROUP_BITS    (BIAS_GROUP_BITS),
          .GROUP_SIZE    (BIAS_GROUP_SIZE),
          .CH_BITS       (BIAS_CH_BITS),
          .FIFO_DEPTH    (FIFO_DEPTH),
          .FIFO_AF_LEVEL (FIFO_AF_LEVEL),
          .BIAS_INIT_FILE(BIAS_INIT_FILE),
          .MULT_CNT      (MULT_CNT),
          .MULT_FACTOR0  (MULT_FACTOR0),
          .MULT_FACTOR1  (MULT_FACTOR1),
          .MULT_FACTOR2  (MULT_FACTOR2),
          .MULT_FACTOR3  (MULT_FACTOR3)
      ) u_bias_quant_relu (
          .clk              (clk),
          .rst_n            (rst_n),
  
          
          .in_pixel_data_bus (conv_out_pixel_data_bus),
          .in_valid          (conv_out_valid),
          .in_end_frame      (conv_out_end_frame),
          .in_end_all_frame  (conv_out_end_all_frame),
          .in_ready          (conv_out_ready),
  
          
          .out_pixel_data     (out_pixel),
          .out_valid          (out_valid),
          .out_end_frame      (out_end_frame),
          .out_end_all_frame  (out_end_all_frame),
          .out_ready          (out_ready)
      );
  
      
      
      
      
      
      
      
      
   
   
   
      
      
      
      
      
      
  
  
  
endmodule    //depthwiseConv2D_cg
