//==============================================================================
// 模块名：linebuf_bank_ram
// 功能说明：
//   供滑窗生成器使用的单端口行缓冲 RAM bank。
// 时钟与复位：
//   clk：上升沿时钟
// 输入接口：
//   we/addr/din：写控制、地址与写入数据
// 输出接口：
//   dout：当前地址异步读出数据
// 时序特性：
//   同步写、组合读。读出数据在addr变化后下一个时钟周期内有效。
//==============================================================================
module linebuf_bank_ram #(
    parameter integer DATA_W = 8,                          // 存储数据位宽
    parameter integer DEPTH  = 64,                         // RAM 深度
    parameter integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH) // 地址位宽
)(
    input  wire                     clk,   // 时钟信号
    input  wire                     we,    // 写使能
    input  wire [ADDR_W-1:0]        addr,  // 读写地址
    input  wire signed [DATA_W-1:0] din,   // 写入数据
    output wire signed [DATA_W-1:0] dout   // 当前地址读出数据
);
    (* ram_style = "distributed" *)
    reg signed [DATA_W-1:0] mem [0:DEPTH-1];

    assign dout = mem[addr];

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din;
        end
    end
endmodule
