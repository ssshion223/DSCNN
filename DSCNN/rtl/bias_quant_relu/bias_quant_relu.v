`timescale 1ns / 1ps

//==============================================================================
// 模块名：bias_quant_relu
// 功能说明：
//   对卷积结果加 Bias，执行定点量化，随后进行 ReLU 与饱和截断。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   wr_valid/data_in/bias：输入样本与对应 Bias
// 输出接口：
//   data_out/out_valid：量化激活结果与有效标志
// 时序特性：
//   乘法为 1 级流水，out_valid 与寄存级对齐。
//==============================================================================
module bias_quant_relu #(
    parameter IN_WIDTH  = 32,  // 输入数据位宽
    parameter BIAS_WIDTH = 32, // Bias 数据位宽
    parameter OUT_WIDTH = 8,   // 输出量化位宽
    parameter SHIFT_VAL = 16   // 定点右移位数
)(
    input  wire                        clk,       // 时钟信号
    input  wire                        rst_n,     // 低有效复位
    input  wire                        wr_valid,    // 输入数据有效信号
    input  wire signed [IN_WIDTH-1:0]  data_in,   // 待处理输入数据
    input  wire signed [BIAS_WIDTH-1:0]  bias,     // 对应 Bias
    output reg         [OUT_WIDTH-1:0] data_out,      // 量化激活后的输出数据
    output wire                        out_valid    // 输出有效标志
);

    localparam integer ALIGN_WIDTH  = (IN_WIDTH > BIAS_WIDTH) ? IN_WIDTH : BIAS_WIDTH;
    localparam integer SUM_WIDTH    = ALIGN_WIDTH + 1;
    localparam integer MULT_WIDTH   = 11;
    localparam integer REG_WIDTH    = SUM_WIDTH + MULT_WIDTH;
    localparam integer VALID_WIDTH  = (REG_WIDTH > SHIFT_VAL) ? (REG_WIDTH - SHIFT_VAL) : 1;
    
    // 最大正数值 (8位时为 8'h7F = 127)
    localparam MAX_VAL = {1'b0, {(OUT_WIDTH-1){1'b1}}};
    
    // 量化乘数 (硬编码为提供的 915)
    // 915 需要 10 位 (2^9 = 512, 2^10 = 1024)。加上符号位需要 11 位。
    wire signed [10:0] MULT_FACTOR = 11'sd915; 

    wire signed [ALIGN_WIDTH-1:0] data_in_aligned = {{(ALIGN_WIDTH-IN_WIDTH){data_in[IN_WIDTH-1]}}, data_in};
    wire signed [ALIGN_WIDTH-1:0] bias_aligned    = {{(ALIGN_WIDTH-BIAS_WIDTH){bias[BIAS_WIDTH-1]}}, bias};

    // ==========================================
    // 1. 加法器 (先对齐位宽，再扩展 1 位防止溢出)
    // ==========================================
    wire signed [SUM_WIDTH-1:0] sum;
    assign sum = {data_in_aligned[ALIGN_WIDTH-1], data_in_aligned} + {bias_aligned[ALIGN_WIDTH-1], bias_aligned};

    // ==========================================
    // 2. 乘法与流水线寄存
    // ==========================================
    reg signed [REG_WIDTH-1:0] data_reg;
    reg valid_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_reg  <= {REG_WIDTH{1'b0}};
            valid_reg <= 1'b0;
        end else if (wr_valid) begin
            data_reg  <= sum * MULT_FACTOR; // 安全的有符号乘法
            valid_reg <= 1'b1;
        end else begin
            valid_reg <= 1'b0;
        end
    end
    
    // ==========================================
    // 3. 量化与动态截位 (组合逻辑)
    // ==========================================
    // 从 SHIFT_VAL 开始，向上截取 VALID_WIDTH 位
    wire [VALID_WIDTH-1:0] quant_val = data_reg[SHIFT_VAL +: VALID_WIDTH];

    // ==========================================
    // 4. ReLU 激活与饱和截断
    // ==========================================
    always @(*) begin
        // 判断原乘法结果真实的符号位 (最高位)
        if (data_reg[REG_WIDTH-1] == 1'b1) begin 
            // 如果是负数，直接 ReLU 清零
            data_out = {OUT_WIDTH{1'b0}};
            
        end else if (quant_val[VALID_WIDTH-2 : OUT_WIDTH-1] != {(VALID_WIDTH - OUT_WIDTH){1'b0}}) begin 
            // 正数饱和度判断：修正了数组越界 Bug，使用 VALID_WIDTH-2
            data_out = MAX_VAL; 
            
        end else begin
            // 安全截断
            data_out = quant_val[OUT_WIDTH-1:0];
        end
    end

    assign out_valid = valid_reg;

endmodule