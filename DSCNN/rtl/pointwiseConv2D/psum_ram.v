module psum_ram #(
    parameter DATA_WIDTH = 32,  // 数据位宽 (例如：单通道32-bit，或全通道64*32=2048-bit)
    parameter ADDR_WIDTH = 7,   // 地址位宽 (对于125深度，2^7 = 128)
    parameter DEPTH      = 125  // RAM 实际深度 (25x5 = 125 像素)
)(
    input  wire                   clk,
    
    // 读端口 (Read Port) - 用于读取上一通道的 Partial Sum
    input  wire                   re,      // 读使能
    input  wire [ADDR_WIDTH-1:0]  raddr,   // 读地址 (0 ~ 124)
    output reg  [DATA_WIDTH-1:0]  rdata,   // 读出数据 (延后1个周期有效)

    // 写端口 (Write Port) - 用于写入累加后的新 Partial Sum
    input  wire                   we,      // 写使能
    input  wire [ADDR_WIDTH-1:0]  waddr,   // 写地址 (0 ~ 124)
    input  wire [DATA_WIDTH-1:0]  wdata    // 写入数据
);

    // 声明二维寄存器数组作为 RAM 存储体
    // 综合工具（如 Vivado, Quartus）会自动将其映射为底层的 BRAM 资源
    reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) begin
            ram[i] = {DATA_WIDTH{1'b0}}; // 初始化为0
        end
    end

    //---------------------------------------------------------
    // 读逻辑：同步读出 (Synchronous Read)
    // 必须使用时钟上升沿，这样才能映射到硬核 BRAM
    //---------------------------------------------------------
    always @(posedge clk) begin
        if (re) begin
            rdata <= ram[raddr];
        end
    end

    //---------------------------------------------------------
    // 写逻辑：同步写入 (Synchronous Write)
    //---------------------------------------------------------
    always @(posedge clk) begin
        if (we) begin
            ram[waddr] <= wdata;
        end
    end

endmodule