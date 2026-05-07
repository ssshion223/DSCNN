module fc_argmax_stream_cg #(  parameter DATA_W = 8,
  parameter IN_CH = 64,
  parameter OUT_CLASS = 12,
  parameter WEIGHT_W = 8,
  parameter BIAS_W = 32,
  parameter ACC_W = 40,
  parameter Q_MUL = 239,
  parameter Q_SHIFT = 16,
  parameter Q_OFFSET = 128,
  parameter WEIGHT_INIT_FILE = "./data/DS-CNN_fc.memh",
  parameter BIAS_INIT_FILE = "./data/DS-CNN_fc_bias.memh"
)
(
input wire  [DATA_W-1:0] in_data,
input wire clk,
input wire rst_n,
input wire in_valid,
input wire out_ready,
output reg [7:0] out_class,
output reg [ACC_W-1:0] out_score,
output wire in_ready,
output reg out_valid
);

  //----Code starts here: integrated by Robei-----
  localparam integer INDEX_W  = $clog2(IN_CH);
      localparam integer SCALE_W  = ACC_W + 16;
      localparam [1:0]   ST_INIT  = 2'd0;
      localparam [1:0]   ST_RUN   = 2'd1;
      localparam [1:0]   ST_OUT   = 2'd2;
  
      reg [1:0]          state_reg;
      reg [INDEX_W-1:0]  input_index;
      reg [WEIGHT_W-1:0] weight_mem [0:(IN_CH*OUT_CLASS)-1];
      reg [BIAS_W-1:0]   bias_mem   [0:OUT_CLASS-1];
      reg [ACC_W-1:0]    acc_reg    [0:OUT_CLASS-1];
      reg [ACC_W-1:0]    quant_reg  [0:OUT_CLASS-1];
  
      reg [ACC_W-1:0]    stage1_score [0:5];
      reg [7:0]          stage1_class [0:5];
      reg [ACC_W-1:0]    stage2_score [0:2];
      reg [7:0]          stage2_class [0:2];
      reg [ACC_W-1:0]    stage3_score [0:1];
      reg [7:0]          stage3_class [0:1];
      reg [7:0]          best_class_next;
      reg [ACC_W-1:0]    best_score_next;
  
      integer            cls_idx;
      integer            q_idx;
  
      function [DATA_W-1:0] tc_data;
          input [DATA_W-1:0] value;
          begin
              tc_data = (~value) + {{(DATA_W-1){1'b0}}, 1'b1};
          end
      endfunction
  
      function [WEIGHT_W-1:0] tc_weight;
          input [WEIGHT_W-1:0] value;
          begin
              tc_weight = (~value) + {{(WEIGHT_W-1){1'b0}}, 1'b1};
          end
      endfunction
  
      function [15:0] tc_mul;
          input [15:0] value;
          begin
              tc_mul = (~value) + 16'h0001;
          end
      endfunction
  
      function [ACC_W-1:0] tc_acc;
          input [ACC_W-1:0] value;
          begin
              tc_acc = (~value) + {{(ACC_W-1){1'b0}}, 1'b1};
          end
      endfunction
  
      function [ACC_W-1:0] sx_bias_to_acc;
          input [BIAS_W-1:0] value;
          begin
              sx_bias_to_acc = {{(ACC_W-BIAS_W){value[BIAS_W-1]}}, value};
          end
      endfunction
  
      function [ACC_W-1:0] sx_mul_to_acc;
          input [15:0] value;
          begin
              sx_mul_to_acc = {{(ACC_W-16){value[15]}}, value};
          end
      endfunction
  
      function [15:0] mul_tc_8x8;
          input [DATA_W-1:0]   lhs;
          input [WEIGHT_W-1:0] rhs;
          reg                  lhs_neg;
          reg                  rhs_neg;
          reg                  prod_neg;
          reg [DATA_W-1:0]     lhs_mag;
          reg [WEIGHT_W-1:0]   rhs_mag;
          reg [15:0]           prod_mag;
          begin
              lhs_neg  = lhs[DATA_W-1];
              rhs_neg  = rhs[WEIGHT_W-1];
              prod_neg = lhs_neg ^ rhs_neg;
  
              if (lhs_neg == 1'b1) begin
                  lhs_mag = tc_data(lhs);
              end else begin
                  lhs_mag = lhs;
              end
  
              if (rhs_neg == 1'b1) begin
                  rhs_mag = tc_weight(rhs);
              end else begin
                  rhs_mag = rhs;
              end
  
              prod_mag = lhs_mag * rhs_mag;
              if ((prod_neg == 1'b1) && (prod_mag != 16'h0000)) begin
                  mul_tc_8x8 = tc_mul(prod_mag);
              end else begin
                  mul_tc_8x8 = prod_mag;
              end
          end
      endfunction
  
      function tc_gt;
          input [ACC_W-1:0] lhs;
          input [ACC_W-1:0] rhs;
          begin
              if (lhs[ACC_W-1] != rhs[ACC_W-1]) begin
                  tc_gt = (lhs[ACC_W-1] == 1'b0);
              end else begin
                  tc_gt = (lhs > rhs);
              end
          end
      endfunction
  
      function [ACC_W-1:0] quantize_acc;
          input [ACC_W-1:0] value;
          reg               value_neg;
          reg [ACC_W-1:0]   value_mag;
          reg [SCALE_W-1:0] scaled_mag;
          reg [ACC_W-1:0]   shifted_mag;
          reg [ACC_W-1:0]   quant_signed;
          reg [ACC_W-1:0]   quant_offset;
          begin
              value_neg = value[ACC_W-1];
              if (value_neg == 1'b1) begin
                  value_mag = tc_acc(value);
              end else begin
                  value_mag = value;
              end
  
              scaled_mag  = value_mag * Q_MUL;
              shifted_mag = scaled_mag >> Q_SHIFT;
  
              if ((value_neg == 1'b1) && (shifted_mag != {ACC_W{1'b0}})) begin
                  quant_signed = tc_acc(shifted_mag);
              end else begin
                  quant_signed = shifted_mag;
              end
  
              quant_offset = quant_signed + Q_OFFSET;
              if (quant_offset[ACC_W-1] == 1'b1) begin
                  quantize_acc = {ACC_W{1'b0}};
              end else if (quant_offset > {{(ACC_W-8){1'b0}}, 8'hff}) begin
                  quantize_acc = {{(ACC_W-8){1'b0}}, 8'hff};
              end else begin
                  quantize_acc = quant_offset;
              end
          end
      endfunction
  
      initial begin
          $readmemh(WEIGHT_INIT_FILE, weight_mem);
          $readmemh(BIAS_INIT_FILE, bias_mem);
      end
  
      assign in_ready = (state_reg == ST_RUN);
  
      always @(*) begin
          for (q_idx = 0; q_idx < OUT_CLASS; q_idx = q_idx + 1) begin
              quant_reg[q_idx] = quantize_acc(acc_reg[q_idx]);
          end
  
          if (tc_gt(quant_reg[1], quant_reg[0])) begin
              stage1_score[0] = quant_reg[1];
              stage1_class[0] = 8'd1;
          end else begin
              stage1_score[0] = quant_reg[0];
              stage1_class[0] = 8'd0;
          end
  
          if (tc_gt(quant_reg[3], quant_reg[2])) begin
              stage1_score[1] = quant_reg[3];
              stage1_class[1] = 8'd3;
          end else begin
              stage1_score[1] = quant_reg[2];
              stage1_class[1] = 8'd2;
          end
  
          if (tc_gt(quant_reg[5], quant_reg[4])) begin
              stage1_score[2] = quant_reg[5];
              stage1_class[2] = 8'd5;
          end else begin
              stage1_score[2] = quant_reg[4];
              stage1_class[2] = 8'd4;
          end
  
          if (tc_gt(quant_reg[7], quant_reg[6])) begin
              stage1_score[3] = quant_reg[7];
              stage1_class[3] = 8'd7;
          end else begin
              stage1_score[3] = quant_reg[6];
              stage1_class[3] = 8'd6;
          end
  
          if (tc_gt(quant_reg[9], quant_reg[8])) begin
              stage1_score[4] = quant_reg[9];
              stage1_class[4] = 8'd9;
          end else begin
              stage1_score[4] = quant_reg[8];
              stage1_class[4] = 8'd8;
          end
  
          if (tc_gt(quant_reg[11], quant_reg[10])) begin
              stage1_score[5] = quant_reg[11];
              stage1_class[5] = 8'd11;
          end else begin
              stage1_score[5] = quant_reg[10];
              stage1_class[5] = 8'd10;
          end
  
          if (tc_gt(stage1_score[1], stage1_score[0])) begin
              stage2_score[0] = stage1_score[1];
              stage2_class[0] = stage1_class[1];
          end else begin
              stage2_score[0] = stage1_score[0];
              stage2_class[0] = stage1_class[0];
          end
  
          if (tc_gt(stage1_score[3], stage1_score[2])) begin
              stage2_score[1] = stage1_score[3];
              stage2_class[1] = stage1_class[3];
          end else begin
              stage2_score[1] = stage1_score[2];
              stage2_class[1] = stage1_class[2];
          end
  
          if (tc_gt(stage1_score[5], stage1_score[4])) begin
              stage2_score[2] = stage1_score[5];
              stage2_class[2] = stage1_class[5];
          end else begin
              stage2_score[2] = stage1_score[4];
              stage2_class[2] = stage1_class[4];
          end
  
          if (tc_gt(stage2_score[1], stage2_score[0])) begin
              stage3_score[0] = stage2_score[1];
              stage3_class[0] = stage2_class[1];
          end else begin
              stage3_score[0] = stage2_score[0];
              stage3_class[0] = stage2_class[0];
          end
  
          stage3_score[1] = stage2_score[2];
          stage3_class[1] = stage2_class[2];
  
          if (tc_gt(stage3_score[1], stage3_score[0])) begin
              best_score_next = stage3_score[1];
              best_class_next = stage3_class[1];
          end else begin
              best_score_next = stage3_score[0];
              best_class_next = stage3_class[0];
          end
      end
  
      always @(posedge clk or negedge rst_n) begin
          if (!rst_n) begin
              state_reg   <= ST_INIT;
              input_index <= {INDEX_W{1'b0}};
              out_valid   <= 1'b0;
              out_class   <= 8'd0;
              out_score   <= {ACC_W{1'b0}};
              for (cls_idx = 0; cls_idx < OUT_CLASS; cls_idx = cls_idx + 1) begin
                  acc_reg[cls_idx] <= {ACC_W{1'b0}};
              end
          end else begin
              out_valid <= 1'b0;
              case (state_reg)
                  ST_INIT: begin
                      for (cls_idx = 0; cls_idx < OUT_CLASS; cls_idx = cls_idx + 1) begin
                          acc_reg[cls_idx] <= sx_bias_to_acc(bias_mem[cls_idx]);
                      end
                      input_index <= {INDEX_W{1'b0}};
                      state_reg   <= ST_RUN;
                  end
  
                  ST_RUN: begin
                      if (in_valid && in_ready) begin
                          for (cls_idx = 0; cls_idx < OUT_CLASS; cls_idx = cls_idx + 1) begin
                              acc_reg[cls_idx] <= acc_reg[cls_idx] +
                                                  sx_mul_to_acc(mul_tc_8x8(in_data, weight_mem[(input_index*OUT_CLASS)+cls_idx]));
                          end
  
                          if (input_index == IN_CH - 1) begin
                              input_index <= {INDEX_W{1'b0}};
                              state_reg   <= ST_OUT;
                          end else begin
                              input_index <= input_index + 1'b1;
                          end
                      end
                  end
  
                  ST_OUT: begin
                      out_valid <= 1'b1;
                      out_class <= best_class_next;
                      out_score <= best_score_next;
                      if (out_valid && out_ready) begin
                          out_valid <= 1'b0;
                          state_reg <= ST_INIT;
                      end else begin
                          state_reg <= ST_OUT;
                      end
                  end
  
                  default: begin
                      state_reg <= ST_INIT;
                  end
              endcase
          end
      end
  
  
endmodule    //fc_argmax_stream_cg
