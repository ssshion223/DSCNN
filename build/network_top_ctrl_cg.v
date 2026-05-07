module network_top_ctrl_cg #(  parameter DS_DATA_W        = 8,
  parameter DS_COL           = 10,
  parameter DS_ROW           = 49,
  parameter DS_COEFF_GRP_NUM = 64
)
(
input wire  [DS_DATA_W-1:0] in_pixel,
input wire clk,
input wire rst_n,
input wire start,
input wire busy,
input wire in_valid,
input wire ds_in_ready,
output wire  [DS_DATA_W-1:0] ds_in_pixel,
output wire in_ready,
output wire ds_in_valid,
output wire ds_in_end_all_frame,
output wire pp_start
);

  //----Code starts here: integrated by Robei-----
  localparam integer IN_PIXEL_MAX = DS_COL * DS_ROW * DS_COEFF_GRP_NUM;
   localparam integer IN_PIXEL_W   = (IN_PIXEL_MAX <= 1) ? 1 : $clog2(IN_PIXEL_MAX);
  
   wire in_fire;
   reg  start_flag;
   reg  input_processing;
   reg [IN_PIXEL_W-1:0] in_pixel_cnt;
   wire in_pixel_cnt_wrap;
  
   assign in_fire = in_valid && in_ready;
   assign in_pixel_cnt_wrap = (in_pixel_cnt == IN_PIXEL_MAX - 1);
  
   always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
     start_flag <= 1'b0;
    end else if (start && !busy) begin
     start_flag <= 1'b1;
    end else begin
     start_flag <= 1'b0;
    end
   end
  
   always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
     in_pixel_cnt <= {IN_PIXEL_W{1'b0}};
     input_processing <= 1'b0;
    end else if (start_flag) begin
     in_pixel_cnt <= {IN_PIXEL_W{1'b0}};
     input_processing <= 1'b1;
    end else if (in_fire) begin
     if (in_pixel_cnt_wrap) begin
      in_pixel_cnt <= {IN_PIXEL_W{1'b0}};
      input_processing <= 1'b0;
     end else begin
      in_pixel_cnt <= in_pixel_cnt + 1'b1;
     end
    end
   end
  
   assign in_ready = input_processing ? ds_in_ready : 1'b0;
   assign ds_in_valid = input_processing ? in_valid : 1'b0;
   assign ds_in_pixel = $signed(in_pixel);
   assign ds_in_end_all_frame = in_pixel_cnt_wrap;
   assign pp_start = start_flag;
  
  
endmodule    //network_top_ctrl_cg
