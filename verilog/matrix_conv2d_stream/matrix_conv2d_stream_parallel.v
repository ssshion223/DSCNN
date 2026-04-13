//==============================================================================
// Module: matrix_conv2d_stream_parallel (all-channel parallel)
//==============================================================================

module matrix_conv2d_stream_parallel #(
    parameter integer DATA_W   = 8,
    parameter integer COEFF_W  = 8,
    parameter integer ACC_W    = 32,
    parameter integer COL      = 20,
    parameter integer ROW      = 20,
    parameter integer K_H      = 1,
    parameter integer K_W      = 1,
    parameter integer STRIDE   = 1,
    parameter integer PAD_TOP    = K_H/2,
    parameter integer PAD_BOTTOM = K_H/2,
    parameter integer PAD_LEFT   = K_W/2,
    parameter integer PAD_RIGHT  = K_W/2,
    parameter integer OUT_CH   = 64,
    parameter integer COEFF_GRP_NUM = 2,
    parameter integer MAC_PIPELINE = 1,
    parameter         COEFF_INIT_FILE = "D:/vivado/exp/SmartCar/sim/coeff_init.memh",
    parameter integer COEFF_CH_W = ((OUT_CH <= 1) ? 1 : $clog2(OUT_CH))
)(
    input  wire                                 clk,
    input  wire                                 rst_n,

    input  wire                                 in_valid,
    output wire                                 in_ready,
    input  wire signed [DATA_W-1:0]             in_pixel,

    output wire                                 out_valid,
    input  wire                                 out_ready,

    output wire                                 out_end_all_frame,
    output wire                                 out_end_frame,
    output wire signed [OUT_CH*ACC_W-1:0]       out_pixel_data_bus
);

    localparam integer WIN_SIZE   = K_H * K_W;
    localparam integer P_COL      = COL + PAD_LEFT + PAD_RIGHT;
    localparam integer P_ROW      = ROW + PAD_TOP  + PAD_BOTTOM;
    localparam integer OUT_COL    = ((P_COL - K_W) / STRIDE) + 1;
    localparam integer OUT_ROW    = ((P_ROW - K_H) / STRIDE) + 1;
    localparam integer OUT_COL_W  = (OUT_COL <= 1) ? 1 : $clog2(OUT_COL);
    localparam integer OUT_ROW_W  = (OUT_ROW <= 1) ? 1 : $clog2(OUT_ROW);
    localparam integer ROM_OUT_CH = OUT_CH * COEFF_GRP_NUM;
    localparam integer GRP_W      = (COEFF_GRP_NUM <= 1) ? 1 : $clog2(COEFF_GRP_NUM);
    localparam integer CH_COEFF_BUS_W = K_H*K_W*COEFF_W;
    localparam integer GRP_COEFF_BUS_W = OUT_CH*CH_COEFF_BUS_W;

    wire window_in_valid;
    wire window_in_ready;
    wire window_out_valid;
    wire window_out_ready;
    wire signed [K_H*K_W*DATA_W-1:0] window_bus;
    wire window_out_fire;

    reg signed [K_H*K_W*DATA_W-1:0]  window_bus_hold;
    reg                          end_all_frame_hold;
    reg                          end_frame_hold;
    reg                          mac_start_d;

    reg [OUT_COL_W-1:0] win_col_counter;
    reg [OUT_ROW_W-1:0] win_row_counter;
    wire end_frame,end_all_frame;

    wire win_col_wrap;
    wire win_row_wrap;
    wire coeff_grp_wrap;
    wire frame_end_fire;

    wire fifo_almost_full;

    wire signed [OUT_CH*ACC_W-1:0] mac_sum_bus;
    wire [OUT_CH-1:0]              mac_valid_bus;
    wire [OUT_CH*2-1:0]            mac_user_bus;

    wire [1:0] out_user;
    wire out_fire;
    reg  [GRP_W-1:0] coeff_grp_idx;
    wire signed [GRP_COEFF_BUS_W-1:0] coeff_bus_grp;

    assign win_col_wrap = (win_col_counter == OUT_COL - 1);
    assign win_row_wrap = (win_row_counter == OUT_ROW - 1);
    assign coeff_grp_wrap = (coeff_grp_idx == COEFF_GRP_NUM - 1);
    assign end_frame = win_col_wrap && win_row_wrap;
    assign end_all_frame = end_frame&& coeff_grp_wrap;
    assign frame_end_fire = window_out_fire && end_frame;

    // 只要后端 FIFO 不接近满，就继续接收窗口
    assign window_out_ready = !fifo_almost_full;
    assign window_in_valid  = in_valid && !fifo_almost_full;
    assign window_out_fire  = window_out_valid && window_out_ready;

    // 对上游输入的 backpressure
    assign in_ready = window_in_ready && !fifo_almost_full;
    assign out_fire = out_valid && out_ready;

    // 每输出完整一帧后，切换到下一组系数；每组系数包含 OUT_CH 个输出�?�道的权�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coeff_grp_idx <= {GRP_W{1'b0}};
        end else if (frame_end_fire) begin
            if (coeff_grp_wrap) begin
                coeff_grp_idx <= {GRP_W{1'b0}};
            end else begin
                coeff_grp_idx <= coeff_grp_idx + 1'b1;
            end
        end
    end

    // 统计窗口输出坐标，用于生成当前窗口的 end_line / end_frame
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_col_counter <= {OUT_COL_W{1'b0}};
            win_row_counter <= {OUT_ROW_W{1'b0}};
        end else if (window_out_fire) begin
            if (win_col_wrap) begin
                win_col_counter <= {OUT_COL_W{1'b0}};
                if (win_row_wrap) begin
                    win_row_counter <= {OUT_ROW_W{1'b0}};
                end else begin
                    win_row_counter <= win_row_counter + 1'b1;
                end
            end else begin
                win_col_counter <= win_col_counter + 1'b1;
            end
        end
    end

    // 锁存窗口及其标志；window_out_fire有效�? mac_start_d 延时�?个周期启�? MAC 计算，以便在窗口数据准备好时能同步输�?
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            window_bus_hold <= {(K_H*K_W*DATA_W){1'b0}};
            end_all_frame_hold   <= 1'b0;
            end_frame_hold  <= 1'b0;
            mac_start_d     <= 1'b0;
        end else begin
            mac_start_d <= window_out_fire;
            if (window_out_fire) begin
                window_bus_hold <= window_bus;
                end_all_frame_hold   <= end_all_frame;
                end_frame_hold  <= end_frame;
            end
        end
    end

    conv_sliding_padding #(
        .DATA_W(DATA_W),
        .COL(COL),
        .ROW(ROW),
        .K_H(K_H),
        .K_W(K_W),
        .STRIDE(STRIDE),
        .PAD_TOP(PAD_TOP),
        .PAD_BOTTOM(PAD_BOTTOM),
        .PAD_LEFT(PAD_LEFT),
        .PAD_RIGHT(PAD_RIGHT)
    ) u_conv_sliding_padding (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(window_in_valid),
        .in_ready(window_in_ready),
        .in_pixel(in_pixel),
        .out_valid(window_out_valid),
        .out_ready(window_out_ready),
        .window_bus(window_bus)
    );

    conv_coeff_store #(
        .COEFF_W(COEFF_W),
        .K_H(K_H),
        .K_W(K_W),
        .OUT_CH(OUT_CH),
        .COEFF_GRP_NUM(COEFF_GRP_NUM),
        .INIT_FILE(COEFF_INIT_FILE)
    ) u_conv_coeff_store_grp (
        .clk(clk),
        .coeff_rd_grp(coeff_grp_idx),
        .coeff_bus(coeff_bus_grp)
    );

    genvar gch;
    generate
        for (gch = 0; gch < OUT_CH; gch = gch + 1) begin : gen_ch_parallel
            wire signed [K_H*K_W*COEFF_W-1:0] coeff_bus_ch;
            assign coeff_bus_ch = coeff_bus_grp[gch*CH_COEFF_BUS_W +: CH_COEFF_BUS_W];

            conv_mac_tree #(
                .DATA_W(DATA_W),
                .COEFF_W(COEFF_W),
                .ACC_W(ACC_W),
                .K_H(K_H),
                .K_W(K_W),
                .PIPELINE(MAC_PIPELINE),
                .USER_W(2)
            ) u_conv_mac_tree (
                .clk(clk),
                .rst_n(rst_n),
                .in_valid(mac_start_d),
                .in_user({end_all_frame_hold, end_frame_hold}),
                .window_bus(window_bus_hold),
                .coeff_bus(coeff_bus_ch),
                .mac_sum(mac_sum_bus[gch*ACC_W +: ACC_W]),
                .out_valid(mac_valid_bus[gch]),
                .out_user(mac_user_bus[gch*2 +: 2])
            );
        end
    endgenerate

    // 并行通道 valid/user 理论上一致，取ch[0]
    fwft_fifo_reg #(
        .WIDTH(OUT_CH*ACC_W + 2),
        .DEPTH(16),
        .AF_LEVEL(10)
    ) u_fwft_fifo_reg (
        .clk(clk),
        .rst_n(rst_n),
        .din({mac_user_bus[1:0], mac_sum_bus}),
        .wr_en(mac_valid_bus[0]),
        .full(),
        .almost_full(fifo_almost_full),
        .rd_en(out_ready),
        .dout({out_user, out_pixel_data_bus}),
        .valid(out_valid),
        .empty()
    );

    assign out_end_all_frame = out_user[1];
    assign out_end_frame  = out_user[0];

endmodule
