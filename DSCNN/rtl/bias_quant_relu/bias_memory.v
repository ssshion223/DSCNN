//==============================================================================
// 模块名：bias_memory
// 功能说明：
//   以 {group_id, channel_id} 为地址索引的 Bias ROM。
// 时钟与复位：
//   clk：上升沿时钟
// 输入接口：
//   read_en/group_id/channel_id：同步读请求与地址字段
// 输出接口：
//   bias_out：有符号 Bias 输出
// 时序特性：
//   在 read_en 有效时进行寄存器同步读。
//==============================================================================
module bias_memory #(
    parameter DATA_WIDTH = 32,      // Bias 数据位宽
    parameter GROUP_BITS = 4,       // 组号的位宽 
    parameter CH_BITS    = 4,        // 通道号的位宽
    parameter BIAS_INIT_FILE = "D:/vivado/exp/DSCNN/rtl/dw/DS-CNN_dw0_Fold_bias.hex" // Bias 初始化文件路径
)(
    input  wire                              clk,         // 时钟信号
    input  wire                              read_en,    // 读使能
    input  wire [GROUP_BITS-1:0]             group_id,   // 外部输入的组号
    input  wire [CH_BITS-1:0]                channel_id, // 外部输入的通道号
    
    output reg  signed [DATA_WIDTH-1:0]      bias_out    // 输出 Bias 数据
);

    // ==========================================
    // 1. 定义内部存储器
    // ==========================================
    localparam MEM_DEPTH = 1 << (GROUP_BITS + CH_BITS); 
    reg signed [DATA_WIDTH-1:0] rom [0:MEM_DEPTH-1];

    // ==========================================
    // 2. 存储器初始化
    // ==========================================
    initial begin
        $readmemh(BIAS_INIT_FILE, rom); 
    end

    // ==========================================
    // 3. 地址映射 (二维转一维)
    // ==========================================
    wire [GROUP_BITS+CH_BITS-1:0] read_addr;
    assign read_addr = {group_id, channel_id};

    // ==========================================
    // 4. 同步读逻辑
    // ==========================================
    always @(posedge clk) begin
        if (read_en) begin
            bias_out <= rom[read_addr];
        end
    end

endmodule