module ram_frame_to_fifo_reader_cg #(  parameter PIXELS_PER_FRAME = 12,
  parameter SEGMENTS = 4,
  parameter CHANNELS = 64,
  parameter RAM_DATA_W = 22*(CHANNELS/SEGMENTS),
  parameter RAM_ADDR_W = $clog2(PIXELS_PER_FRAME*SEGMENTS),
  parameter FIFO_DEPTH = 16,
  parameter FIFO_AF_LEVEL = 10,
  parameter OUT_W = RAM_DATA_W/(CHANNELS/SEGMENTS)
)
(
input wire  [RAM_DATA_W-1:0] ram_rdata,
output wire  [RAM_ADDR_W-1:0] ram_raddr,
output wire  [OUT_W-1:0] out_data,
input wire clk,
input wire rst_n,
input wire start,
input wire out_ready,
output wire busy,
output wire ram_re,
output wire out_valid,
output wire end_frame,
output wire end_all_frame
);

  //----Code starts here: integrated by Robei-----
  
  
      localparam integer RAM_DEPTH   = PIXELS_PER_FRAME * SEGMENTS;
      localparam integer SEG_CHANS    = CHANNELS / SEGMENTS;
      localparam integer CH_W = (CHANNELS <= 1) ? 1 : $clog2(CHANNELS);
      localparam integer SEG_W = (SEGMENTS <= 1) ? 1 : $clog2(SEGMENTS);
      localparam integer CHN_W = (SEG_CHANS <= 1) ? 1 : $clog2(SEG_CHANS);
      localparam integer PIXEL_W = $clog2(PIXELS_PER_FRAME);
  
      reg                         busy_reg;
      reg                         processing;
  
      reg                         ram_re_reg;
      reg                         ram_data_valid_reg;
      reg [RAM_ADDR_W-1:0]        ram_raddr_reg;
      reg [SEG_W-1:0]             ram_seg_cnt;
  
      reg [CH_W-1:0]              out_ch_cnt; 
      reg [CH_W-1:0]              ram_ch_cnt;
  
  
      wire start_fire = start && !busy_reg; 
      wire raddr_wrap = (ram_raddr_reg == RAM_DEPTH - 1);
      wire ram_seg_cnt_wrap = (ram_seg_cnt == SEGMENTS - 1);
      wire ram_ch_cnt_wrap = (ram_ch_cnt == CHANNELS - 1);
      wire done = (ram_ch_cnt_wrap && raddr_wrap); 
      reg raddr_wrap_flag; 
      reg start_flag;
  
      always @(posedge clk) begin
          if (start_fire) begin
              start_flag <= 1'b1;
          end else begin
              start_flag <= 1'b0;
          end
      end
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              processing <= 1'b0;
          end else if(start_flag) begin
              processing <= 1'b1; 
          end else if(done && ram_re_reg) begin
              processing <= 1'b0; 
          end
      end
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              busy_reg <= 1'b0;
          end else if (start_fire) begin
              busy_reg <= 1'b1; 
          end else if (out_valid && out_ready && end_all_frame) begin
              busy_reg <= 1'b0; 
          end
      end
  
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              ram_raddr_reg <= {RAM_ADDR_W{1'b0}};
          end else if(start_fire) begin
              ram_raddr_reg <= {RAM_ADDR_W{1'b0}}; 
          end else if(ram_re_reg) begin
              ram_raddr_reg <= (raddr_wrap) ? {RAM_ADDR_W{1'b0}} : ram_raddr_reg + 1'b1; 
          end
      end
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              ram_ch_cnt <= {CH_W{1'b0}};
          end else if(start_flag) begin
              ram_ch_cnt <= {CH_W{1'b0}};
          end else if(raddr_wrap && ram_re_reg) begin
              ram_ch_cnt <= (ram_ch_cnt_wrap) ? {CH_W{1'b0}} : ram_ch_cnt + 1'b1;
          end
      end
  
      wire can_read = processing && !fifo_almost_full;
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              ram_re_reg <= 1'b0;
          end else if(start_fire) begin
              ram_re_reg <= 1'b0;
          end else if (done) begin
              ram_re_reg <= 1'b0;
          end else begin
              ram_re_reg <= can_read;
          end
      end
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              ram_data_valid_reg <= 1'b0;
          end else if(start_flag) begin
              ram_data_valid_reg <= 1'b0; 
          end else begin
              ram_data_valid_reg <= ram_re_reg; 
          end
      end
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              ram_seg_cnt <= {SEG_W{1'b0}};
          end else if (start_flag) begin
              ram_seg_cnt <= {SEG_W{1'b0}}; 
          end else if (ram_data_valid_reg) begin
              ram_seg_cnt <= (ram_seg_cnt_wrap) ? {SEG_W{1'b0}} : ram_seg_cnt + 1'b1; 
          end
      end
  
  
      always @(posedge clk or negedge rst_n) begin
          if(!rst_n) begin
              raddr_wrap_flag <= 1'b0;
          end else begin
              raddr_wrap_flag <= raddr_wrap; 
          end
      end
  
      wire out_ch_wrap = (out_ch_cnt == CHANNELS - 1);
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              out_ch_cnt <= {CH_W{1'b0}};
          end else if (start_flag) begin
              out_ch_cnt <= {CH_W{1'b0}};
          end else if (ram_data_valid_reg && raddr_wrap_flag ) begin
              out_ch_cnt <= (out_ch_wrap) ? {CH_W{1'b0}} : out_ch_cnt + 1'b1; 
          end
      end
  
      wire [CHN_W-1:0] seg_ch_cnt = out_ch_cnt[CHN_W-1:0]; 
      wire [SEG_W-1:0] seg_cnt;
      generate
          if(SEGMENTS > 1) 
              assign seg_cnt = out_ch_cnt[CH_W-1:CHN_W]; 
          else
              assign seg_cnt = ram_seg_cnt; 
      endgenerate
  
  
      wire [OUT_W-1:0] ram_data_sel = ram_rdata[seg_ch_cnt * OUT_W +: OUT_W]; 
      wire seg_aligned = (ram_seg_cnt == seg_cnt); 
      reg [OUT_W-1:0] fifo_din_reg;
      reg fifo_wr_en_reg;
      reg [PIXEL_W-1:0] out_pixel_cnt;
      reg end_frame_align, end_all_frame_align;
      wire out_pixel_cnt_wrap = (out_pixel_cnt == PIXELS_PER_FRAME - 1);
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              fifo_din_reg <= {OUT_W{1'b0}};
              fifo_wr_en_reg <= 1'b0;
              out_pixel_cnt <= {PIXEL_W{1'b0}};
              end_frame_align <= 1'b0;
              end_all_frame_align <= 1'b0;
          end else if (ram_data_valid_reg && seg_aligned) begin
              fifo_din_reg <= ram_data_sel;
              fifo_wr_en_reg <= 1'b1;
              out_pixel_cnt <= (out_pixel_cnt_wrap) ? {PIXEL_W{1'b0}} : out_pixel_cnt + 1'b1; 
              end_frame_align <= out_pixel_cnt_wrap; 
              end_all_frame_align <= out_pixel_cnt_wrap && out_ch_wrap;
          end else begin
              fifo_wr_en_reg <= 1'b0;
              end_frame_align <= 1'b0;
              end_all_frame_align <= 1'b0;
          end
      end
  
      wire                        fifo_valid;
      wire [OUT_W + 2 -1:0]       fifo_dout;
  
      assign busy = busy_reg;
  
      assign ram_re = ram_re_reg;
      assign ram_raddr = ram_raddr_reg;
  
      fwft_fifo_reg #(
          .WIDTH(OUT_W+2), 
          .DEPTH(FIFO_DEPTH),
          .AF_LEVEL(FIFO_AF_LEVEL)
      ) u_out_fifo (
          .clk(clk),
          .rst_n(rst_n),
          .din({end_all_frame_align, end_frame_align, fifo_din_reg}),
          .wr_en(fifo_wr_en_reg),
          .full(),
          .almost_full(fifo_almost_full),
          .rd_en(out_ready),
          .dout(fifo_dout),
          .valid(fifo_valid),
          .empty());
      assign out_valid = fifo_valid;
      assign out_data = fifo_dout[OUT_W-1:0];
      assign end_frame = fifo_dout[OUT_W];
      assign end_all_frame = fifo_dout[OUT_W + 1];
  
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
  
  
endmodule    //ram_frame_to_fifo_reader_cg
