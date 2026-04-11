module conv_sliding_window_kxk #(
    parameter integer DATA_W = 8,
    parameter integer COL  = 64,
    parameter integer ROW  = 64,
    parameter integer K      = 3,
    parameter integer STRIDE = 1
)(
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         in_valid,
    output wire                         in_ready,
    input  wire signed [DATA_W-1:0]     in_pixel,
    output reg                          out_valid,
    input  wire                         out_ready,
    output wire signed [K*K*DATA_W-1:0] window_bus
);
    localparam integer WIN_SIZE      = K * K;
    localparam integer COL_W         = (COL <= 1) ? 1 : $clog2(COL);
    localparam integer ROW_W         = (ROW <= 1) ? 1 : $clog2(ROW);
    localparam integer STRIDE_W      = (STRIDE <= 1) ? 1 : $clog2(STRIDE);
    localparam integer LINEBUF_BANKS = (K <= 1) ? 1 : (K - 1);
    localparam integer LINEBUF_BUS_W = DATA_W * LINEBUF_BANKS;

    reg signed [DATA_W-1:0] window [0:WIN_SIZE-1];
    reg [COL_W-1:0] col_idx;
    reg [ROW_W-1:0] row_idx;

    wire signed [LINEBUF_BUS_W-1:0] linebuf_q_bus;
    wire signed [LINEBUF_BUS_W-1:0] linebuf_din_bus;

    wire col_wrap;
    wire row_wrap;
    wire in_fire;

    integer r;
    integer c;
    genvar gw;
    genvar gb;

    assign col_wrap = (col_idx == COL - 1);
    assign row_wrap = (row_idx == ROW - 1);
    assign in_ready = !out_valid || out_ready;
    assign in_fire  = in_valid && in_ready;


    generate
        for (gw = 0; gw < WIN_SIZE; gw = gw + 1) begin : gen_window_pack
            assign window_bus[gw*DATA_W +: DATA_W] = window[gw];
        end
    endgenerate

    generate
        if (K > 1) begin : gen_linebuf
            for (gb = 0; gb < (K-1); gb = gb + 1) begin : gen_bank
                localparam integer LB_POS = (K - 2 - gb);
                if (gb == 0) begin : gen_din0
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] = in_pixel;
                end else begin : gen_dinx
                    localparam integer PREV_POS = (K - 1 - gb);
                    assign linebuf_din_bus[LB_POS*DATA_W +: DATA_W] =
                        linebuf_q_bus[PREV_POS*DATA_W +: DATA_W];
                end

                linebuf_bank_ram #(
                    .DATA_W (DATA_W),
                    .DEPTH  (COL),
                    .ADDR_W (COL_W)
                ) u_linebuf_bank_ram (
                    .clk  (clk),
                    .we   (in_fire),
                    .addr (col_idx),
                    .din  (linebuf_din_bus[LB_POS*DATA_W +: DATA_W]),
                    .dout (linebuf_q_bus[LB_POS*DATA_W +: DATA_W])
                );
            end
        end else begin : gen_no_linebuf
            assign linebuf_q_bus   = {LINEBUF_BUS_W{1'b0}};
            assign linebuf_din_bus = {LINEBUF_BUS_W{1'b0}};
        end
    endgenerate

reg [STRIDE_W-1:0] stride_col_cnt;
reg [STRIDE_W-1:0] stride_row_cnt;

always @(posedge clk) begin
    if (!rst_n) begin
        col_idx <= {COL_W{1'b0}};
        row_idx <= {ROW_W{1'b0}};
    end else if (in_fire) begin
        if (col_wrap) begin
            col_idx <= {COL_W{1'b0}};   
            row_idx <= (row_wrap) ? {ROW_W{1'b0}} : (row_idx + 1'b1);
        end else begin
            col_idx <= col_idx + 1'b1;
        end
    end
end
always @(posedge clk) begin
    if (!rst_n) begin
        stride_col_cnt <= {STRIDE_W{1'b0}};
        stride_row_cnt <= {STRIDE_W{1'b0}};
    end else if (in_fire) begin
        // 水平 Stride 计数更新
        if (col_wrap) begin
            stride_col_cnt <= {STRIDE_W{1'b0}}; // 列坐标回绕时重置水平 Stride 计数
            // 垂直 Stride 计数更新
            if (row_wrap) begin
                stride_row_cnt <= {STRIDE_W{1'b0}}; // 行坐标回绕时重置垂直 Stride 计数
            end else if (row_idx >= K-1) begin
                stride_row_cnt <= (stride_row_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_row_cnt + 1;
            end
        end else if (col_idx >= K-1) begin
            stride_col_cnt <= (stride_col_cnt == STRIDE - 1) ? {STRIDE_W{1'b0}} : stride_col_cnt + 1;
        end
    end
end


always @(posedge clk) begin
    if (in_fire) begin
        for (r = 0; r < K; r = r + 1) begin
            for (c = 0; c < K-1; c = c + 1) begin
                window[r*K + c] <= window[r*K + c + 1];
            end
        end
        for (r = 0; r < K - 1; r = r + 1) begin
            window[r*K + (K-1)] <= linebuf_q_bus[r*DATA_W +: DATA_W];
        end
        window[(K-1)*K + (K-1)] <= in_pixel;
    end
end


always @(posedge clk) begin
    if (!rst_n) begin
        out_valid <= 1'b0;
    end else if (out_valid && !out_ready) begin
        out_valid <= 1'b1;
    end else if (in_fire) begin
        out_valid <= (row_idx >= (K-1)) && 
                     (col_idx >= (K-1)) && 
                     (stride_col_cnt == 0) && 
                     (stride_row_cnt == 0);
    end else begin
        out_valid <= 1'b0;
    end
end
endmodule
