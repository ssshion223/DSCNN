//==============================================================================
// 模块名：fwft_fifo_reg
// 功能说明：
//   基于寄存器实现的 FWFT（首字直通）FIFO，并提供 almost_full 指示。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   din/wr_en：写入侧接口
//   rd_en：读取确认/就绪
// 输出接口：
//   dout/valid：FWFT 输出通道
//   full/almost_full/empty：FIFO 状态标志
// 时序特性：
//   当有数据可用时，dout 对应数据且 valid 拉高。
//==============================================================================
module fwft_fifo_reg #(
    parameter WIDTH = 8,              // FIFO 数据位宽
    parameter DEPTH = 4,              // FIFO 深度（只能是 2 的幂）
    parameter AF_LEVEL = 3,           // almost_full 阈值
    parameter ADDR_W = $clog2(DEPTH)  // 地址位宽
)(
    input  wire             clk,      // 时钟信号
    input  wire             rst_n,    // 低有效复位
    // 写接口
    input  wire [WIDTH-1:0] din,      // 写入数据
    input  wire             wr_en,    // 写使能
    output wire             full,     // FIFO 满标志
    output reg              almost_full, // FIFO 将满标志
    // 读接口 (FWFT 风格)
    input  wire             rd_en,   // 这里的 rd_en 相当于 "ack" 或 "ready"
    output reg  [WIDTH-1:0] dout,     // 读出数据
    output reg              valid,   // 输出数据有效（FWFT）
    output wire             empty    // FIFO 空标志
);

    // 1. 核心存储区 (Standard FIFO 逻辑)
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_W:0]  wr_ptr, rd_ptr;
    
    wire mem_empty = (wr_ptr == rd_ptr);
    wire [ADDR_W:0] mem_count   = wr_ptr - rd_ptr;
    wire [ADDR_W:0] total_count = mem_count + (valid ? 1'b1 : 1'b0);

    assign full    = (wr_ptr == {~rd_ptr[ADDR_W], rd_ptr[ADDR_W-1:0]});
    assign empty   = mem_empty && !valid; // 整体空的定义：存储空且输出寄存器也空

    // 2. 写逻辑
    always @(posedge clk) begin
        if (!rst_n) 
            wr_ptr <= 1'b0;
        else if (wr_en && !full) begin
            mem[wr_ptr[ADDR_W-1:0]] <= din;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // 3. Almost Full 逻辑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            almost_full <= 1'b0;
        end else begin
            // 当总数据量大于等于设定的阈值时拉高
            almost_full <= (total_count >= AF_LEVEL);
        end
    end

    // 4. 关键的 FWFT 输出状态机逻辑
    // 逻辑：如果 (输出寄存器空) 或者 (当前数据正被取走)，且 (存储区有货) -> 搬运！
    wire go_fetch = (mem_empty == 1'b0) && (valid == 1'b0 || rd_en == 1'b1);

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr <= 1'b0;
            valid  <= 1'b0;
            dout   <= {WIDTH{1'b0}};
        end else begin
            if (go_fetch) begin
                dout   <= mem[rd_ptr[ADDR_W-1:0]];
                valid  <= 1'b1;
                rd_ptr <= rd_ptr + 1'b1;
            end else if (rd_en && mem_empty) begin
                // 如果下游取走了最后一颗数据，且存储区没货了
                valid  <= 1'b0;
            end
        end
    end

endmodule