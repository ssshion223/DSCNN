module conv_coeff_store #(
    parameter integer COEFF_W       = 8,
    parameter integer K_H           = 3,
    parameter integer K_W           = 3,
    parameter integer OUT_CH        = 1,
    parameter integer COEFF_GRP_NUM = 1,
    parameter         INIT_FILE     = "",
    parameter integer COEFF_GRP_W   = ((COEFF_GRP_NUM <= 1) ? 1 : $clog2(COEFF_GRP_NUM))
)(
    input  wire                                  clk,
    input  wire [COEFF_GRP_W-1:0]                coeff_rd_grp,
    output reg  signed [OUT_CH*K_H*K_W*COEFF_W-1:0] coeff_bus
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