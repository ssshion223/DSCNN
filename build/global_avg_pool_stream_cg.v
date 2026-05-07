module global_avg_pool_stream_cg #(  parameter DATA_W   = 8,
  parameter ROW_LEN  = 25,
  parameter ROW_NUM  = 5,
  parameter CHANNELS = 64,
  parameter SUM_W    = 16
)
(
input wire  [DATA_W-1:0] in_data,
input wire clk,
input wire rst_n,
input wire in_valid,
input wire in_channel_last,
input wire in_frame_last,
input wire out_ready,
output reg [DATA_W-1:0] out_data,
output wire in_ready,
output reg out_valid
);

  //----Code starts here: integrated by Robei-----
  localparam integer PIXELS_PER_CH = ROW_LEN * ROW_NUM;
      localparam integer PIX_CNT_W     = $clog2(PIXELS_PER_CH);
      localparam integer CH_CNT_W      = $clog2(CHANNELS);
  
      reg  [SUM_W-1:0]     sum_reg;
      reg  [PIX_CNT_W-1:0] pixel_count;
      reg  [CH_CNT_W-1:0]  channel_count;
      wire [SUM_W-1:0]     sample_ext;
      wire [SUM_W-1:0]     sum_next;
      wire                 in_channel_last_d= in_channel_last; 
      wire                 in_frame_last_d= in_frame_last;
  
      function [DATA_W-1:0] tc_data;
          input [DATA_W-1:0] value;
          begin
              tc_data = (~value) + {{(DATA_W-1){1'b0}}, 1'b1};
          end
      endfunction
  
      function [SUM_W-1:0] tc_sum;
          input [SUM_W-1:0] value;
          begin
              tc_sum = (~value) + {{(SUM_W-1){1'b0}}, 1'b1};
          end
      endfunction
  
      function [SUM_W-1:0] sx_data_to_sum;
          input [DATA_W-1:0] value;
          begin
              sx_data_to_sum = {{(SUM_W-DATA_W){value[DATA_W-1]}}, value};
          end
      endfunction
  
      function [DATA_W-1:0] avg_trunc_zero;
          input [SUM_W-1:0] value_tc;
          reg               value_neg;
          reg [SUM_W-1:0]   value_mag;
          reg [SUM_W-1:0]   quot_mag;
          reg [DATA_W-1:0]  quot_data;
          begin
              
              value_neg = value_tc[SUM_W-1];
              if (value_neg == 1'b1) begin
                  value_mag = tc_sum(value_tc);
              end else begin
                  value_mag = value_tc;
              end
  
              quot_mag  = value_mag / PIXELS_PER_CH;
              quot_data = quot_mag[DATA_W-1:0];
  
              if ((value_neg == 1'b1) && (quot_data != {DATA_W{1'b0}})) begin
                  avg_trunc_zero = tc_data(quot_data);  
              end else begin
                  avg_trunc_zero = quot_data;
              end
          end
      endfunction
  
      assign sample_ext = sx_data_to_sum(in_data);
      assign sum_next   = sum_reg + sample_ext;
      assign in_ready   = ~out_valid;  
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              sum_reg           <= {SUM_W{1'b0}};
              pixel_count       <= {PIX_CNT_W{1'b0}};
              channel_count     <= {CH_CNT_W{1'b0}};
              out_valid         <= 1'b0;
              out_data          <= {DATA_W{1'b0}};
          end else begin
              if (out_valid && out_ready) begin  
                  out_valid        <= 1'b0;
              end
  
              if (in_valid && in_ready) begin
                  if (pixel_count == PIXELS_PER_CH - 1) begin  
                      out_valid        <= 1'b1;
                      out_data         <= avg_trunc_zero(sum_next);
                      sum_reg          <= {SUM_W{1'b0}};
                      pixel_count      <= {PIX_CNT_W{1'b0}};
                      if (channel_count == CHANNELS - 1) begin
                          channel_count <= {CH_CNT_W{1'b0}};
                      end else begin
                          channel_count <= channel_count + 1'b1;
                      end
                  end else begin  
                      sum_reg     <= sum_next;
                      pixel_count <= pixel_count + 1'b1;
                  end
              end
          end
      end
  
  
endmodule    //global_avg_pool_stream_cg
