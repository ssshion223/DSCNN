//==============================================================================
// 模块名：conv_coeff_store
// 功能说明：
//   同步读取的系数分组 ROM。
//   每次读出一整组 OUT_CH 通道所需窗口系数。
// 时钟与复位：
//   clk：上升沿时钟
// 输入接口：
//   coeff_rd_grp：系数组索引
// 输出接口：
//   coeff_bus：单组打包系数总线
// 时序特性：
//   寄存器读出，地址到数据 1 个时钟周期延迟。
//==============================================================================
module conv_coeff_store #(
    parameter integer COEFF_W       = 8,   // 系数位宽
    parameter integer K_H           = 3,   // 卷积核高度
    parameter integer K_W           = 3,   // 卷积核宽度
    parameter integer OUT_CH        = 1,   // 输出通道数
    parameter integer COEFF_GRP_NUM = 1,   // 系数组数量
    parameter         INIT_FILE     = "", // 系数文件路径
    parameter integer COEFF_GRP_W   = ((COEFF_GRP_NUM <= 1) ? 1 : $clog2(COEFF_GRP_NUM)) // 组索引位宽
)(
    input  wire                                  clk,          // 时钟信号
    input  wire [COEFF_GRP_W-1:0]                coeff_rd_grp, // 读取系数组号
    output reg  signed [OUT_CH*K_H*K_W*COEFF_W-1:0] coeff_bus // 打包系数输出总线
);
    localparam integer WIN_SIZE      = K_H * K_W;
    localparam integer GROUP_SIZE    = OUT_CH * WIN_SIZE;
    localparam integer BUS_WIDTH     = GROUP_SIZE * COEFF_W;

    // 关键修改：定义一个“浅而宽”的存储器
    // 深度 = 组数 (COEFF_GRP_NUM)，宽度 = 一次卷积所需的所有权重总位宽 (BUS_WIDTH)
    reg [BUS_WIDTH-1:0] coeff_mem [0:COEFF_GRP_NUM-1];

    initial begin
        // 注意：这里的初始化文件内容也需要适配宽数据格式！
        $readmemh(INIT_FILE, coeff_mem);
    end

    // 读逻辑：一个时钟周期只进行 1 次读操作，完美契合单端口 BRAM！
    always @(posedge clk) begin
        coeff_bus <= coeff_mem[coeff_rd_grp]; 
    end

endmodule