// 输入 FIFO：把 first 标志和 16bit 样本打包后缓存。
module mfcc_input_fifo (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_valid,
    input  wire        in_first,
    input  wire [15:0] in_sample,
    output wire        full,
    output wire [9:0]  data_count,
    input  wire        out_ready,
    output wire        out_valid,
    output wire        out_first,
    output wire [15:0] out_sample,
    output wire        empty
);

    wire [16:0] din_packed;
    wire [16:0] dout_packed;
    wire        wr_en;
    wire        rd_en;
    wire        srst;

    assign din_packed = {in_first, in_sample};
    assign wr_en      = in_valid && !full;
    assign rd_en      = out_ready && !empty;
    assign srst       = ~rst_n;

    assign out_first  = dout_packed[16];
    assign out_sample = dout_packed[15:0];

    sample_fifo_0 u_sample_fifo_0 (
        .clk       (clk),
        .srst      (srst),
        .din       (din_packed),
        .wr_en     (wr_en),
        .rd_en     (rd_en),
        .dout      (dout_packed),
        .full      (full),
        .empty     (empty),
        .valid     (out_valid),
        .data_count(data_count)
    );

endmodule
