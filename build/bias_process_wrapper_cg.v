module bias_process_wrapper_cg #(  parameter IN_WIDTH = 32,
  parameter BIAS_WIDTH = 32,
  parameter OUT_WIDTH = 8,
  parameter SHIFT_VAL = 24,
  parameter GROUP_BITS = 2,
  parameter CH_BITS = 6,
  parameter FIFO_DEPTH = 16,
  parameter FIFO_AF_LEVEL = 14,
  parameter MULT_CNT = 4,
  parameter MULT_FACTOR0 = 12'sd915,
  parameter MULT_FACTOR1 = 12'sd768,
  parameter MULT_FACTOR2 = 12'sd640,
  parameter MULT_FACTOR3 = 12'sd512,
  parameter GROUP_SIZE = 1<<GROUP_BITS,
  parameter BIAS_INIT_FILE = ""
)
(
input wire  [IN_WIDTH-1:0] in_pixel_data_bus,
input wire clk,
input wire rst_n,
input wire in_valid,
input wire in_end_frame,
input wire in_end_all_frame,
input wire out_ready,
output wire  [OUT_WIDTH-1:0] out_pixel_data,
output wire in_ready,
output wire out_valid,
output wire out_end_frame,
output wire out_end_all_frame
);

  //----Code starts here: integrated by Robei-----
  wire fifo_almost_full;
      wire fifo_full;
      assign in_ready = ~fifo_almost_full;
      wire handshake = in_valid && in_ready;
      localparam integer GROUP_BITS_INT = (GROUP_BITS < 1) ? 1 : GROUP_BITS;
  
      
      reg [GROUP_BITS_INT-1:0] curr_group;
      reg [CH_BITS-1:0]    curr_channel;
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              curr_group   <= {GROUP_BITS_INT{1'b0}};
              curr_channel <= {CH_BITS{1'b0}};
          end else if (handshake) begin
              if (in_end_all_frame) begin
                  if(curr_group == GROUP_SIZE - 1) begin
                      curr_group <= {GROUP_BITS_INT{1'b0}};
                  end else begin
                      curr_group <= curr_group + 1'b1;
                  end
                  curr_channel <= {CH_BITS{1'b0}};
              end else if (in_end_frame) begin
                  curr_channel <= curr_channel + 1'b1;
              end
          end
      end
  
      
      wire [BIAS_WIDTH-1:0] current_bias;
      bias_memory #(
          .DATA_WIDTH (BIAS_WIDTH),  
          .GROUP_BITS (GROUP_BITS),
          .CH_BITS    (CH_BITS),
          .BIAS_INIT_FILE (BIAS_INIT_FILE)
      ) u_bias_mem (
          .clk        (clk),
          .read_en    (handshake),
          .group_id   (curr_group),
          .channel_id (curr_channel),
          .bias_out   (current_bias)
      );
  
      
      reg signed [IN_WIDTH-1:0] pipe_data;
      reg                       pipe_valid;
      reg                       pipe_end_frame;
      reg                       pipe_end_all_frame;
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              pipe_valid         <= 1'b0;
              pipe_data          <= 0;
              pipe_end_frame     <= 1'b0;
              pipe_end_all_frame <= 1'b0;
          end else begin
              pipe_valid <= handshake; 
              if (handshake) begin
                  pipe_data          <= $signed(in_pixel_data_bus);
                  pipe_end_frame     <= in_end_frame;
                  pipe_end_all_frame <= in_end_all_frame;
              end
          end
      end
  
      
      wire [OUT_WIDTH-1:0] alu_result;
      wire alu_valid;
      bias_quant_relu #(
          .IN_WIDTH  (IN_WIDTH),
          .BIAS_WIDTH (BIAS_WIDTH),
          .OUT_WIDTH (OUT_WIDTH),
          .SHIFT_VAL (SHIFT_VAL),
          .MULT_CNT   (MULT_CNT),
          .MULT_FACTOR0 (MULT_FACTOR0),
          .MULT_FACTOR1 (MULT_FACTOR1),
          .MULT_FACTOR2 (MULT_FACTOR2),
          .MULT_FACTOR3 (MULT_FACTOR3)
      ) u_alu (
          .clk      (clk),
          .rst_n    (rst_n),
          .wr_valid (pipe_valid),
          .end_all_frame (pipe_end_all_frame),
          .data_in  (pipe_data),
          .bias     (current_bias),
          .data_out (alu_result),
          .out_valid (alu_valid)
      );
  
      
      reg alu_end_frame_d;
      reg alu_end_all_frame_d;
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              alu_end_frame_d <= 1'b0;
              alu_end_all_frame_d <= 1'b0;
          end else if (pipe_valid) begin
              alu_end_frame_d <= pipe_end_frame;
              alu_end_all_frame_d <= pipe_end_all_frame;
          end
      end
  
      
      
      
      
      localparam FIFO_WIDTH = OUT_WIDTH + 2; 
  
      wire [FIFO_WIDTH-1:0] fifo_din = {alu_end_all_frame_d, alu_end_frame_d, alu_result};
      wire [FIFO_WIDTH-1:0] fifo_dout;
  
      fwft_fifo_reg #(
          .WIDTH    (FIFO_WIDTH),  
          .DEPTH    (FIFO_DEPTH),
          .AF_LEVEL (FIFO_AF_LEVEL)
      ) u_output_buffer (
          .clk         (clk),
          .rst_n       (rst_n),
          .din         (fifo_din),
          .wr_en       (alu_valid), 
          .full        (fifo_full),
          .almost_full (fifo_almost_full),
          .rd_en       (out_ready),
          .dout        (fifo_dout),
          .valid       (out_valid),
          .empty       () 
      );
  
      
      
      
      assign out_pixel_data    = fifo_dout[OUT_WIDTH-1 : 0];
      assign out_end_frame     = fifo_dout[OUT_WIDTH];     
      assign out_end_all_frame = fifo_dout[OUT_WIDTH+1];   
  
  
endmodule    //bias_process_wrapper_cg
