module fwft_fifo_reg #(
    parameter WIDTH = 8,
    parameter DEPTH = 4,             // 必须为 2 的幂
    parameter AF_LEVEL = 3,
    parameter ADDR_W = $clog2(DEPTH)
)(
    input  wire             clk,
    input  wire             rst_n,
    // 写接口
    input  wire [WIDTH-1:0] din,
    input  wire             wr_en,
    output wire             full,
    output reg              almost_full,
    // 读接口 (FWFT 风格)
    input  wire             rd_en,   // 这里的 rd_en 相当于 "ack" 或 "ready"
    output reg  [WIDTH-1:0] dout,
    output reg              valid,   // 对应 FWFT 的 !empty
    output wire             empty    // 内部存储是否为空
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