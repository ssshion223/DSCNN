`timescale 1ns / 1ps

//==============================================================================
// Module: dw_pw_pingpong (wrapper)
// Description:
//   Wrapper module that instantiates dw_pw_pingpong_ctrl + 5 submodules
//   with direct signal connections (no combinational logic).
//==============================================================================
module dw_pw_pingpong #(
    parameter PINGPONG_ROUNDS      = 3,
    parameter PIXEL_DEPTH          = 125,
    parameter IN_FRAME_SIZE        = 64,
    parameter IN_DATA_W            = 8,
    parameter DW_COEFF_W           = 8,
    parameter DW_K_H               = 3,
    parameter DW_K_W               = 3,
    parameter DW_MUL_W             = IN_DATA_W+DW_COEFF_W,
    parameter DW_SUM_W             = DW_MUL_W+$clog2(DW_K_H*DW_K_W),
    parameter DW_COL               = 5,
    parameter DW_ROW               = 25,
    parameter DW_STRIDE            = 1,
    parameter DW_PAD_TOP           = (DW_K_H-1)/2,
    parameter DW_PAD_BOTTOM        = (DW_K_H)/2,
    parameter DW_PAD_LEFT          = (DW_K_W-1)/2,
    parameter DW_PAD_RIGHT         = (DW_K_W)/2,
    parameter DW_COEFF_GRP_NUM     = IN_FRAME_SIZE*(PINGPONG_ROUNDS+1),
    parameter DW_FRAME_GRP_NUM     = IN_FRAME_SIZE,
    parameter DW_MAC_PIPELINE      = 1,
    parameter DW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh",
    parameter DW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw_pingpong_bias.hex",
    parameter DW_OUT_WIDTH         = 8,
    parameter DW_SHIFT_VAL         = 16,
    parameter DW_BIAS_GROUP_SIZE   = (PINGPONG_ROUNDS+1),
    parameter DW_BIAS_GROUP_BITS   = $clog2(DW_BIAS_GROUP_SIZE),
    parameter DW_BIAS_CH_BITS      = $clog2(IN_FRAME_SIZE),
    parameter DW_MULT_CNT          = (PINGPONG_ROUNDS+1),
    parameter [11:0] DW_MULT_FACTOR0 = 12'sd1246,
    parameter [11:0] DW_MULT_FACTOR1 = 12'sd828,
    parameter [11:0] DW_MULT_FACTOR2 = 12'sd652,
    parameter [11:0] DW_MULT_FACTOR3 = 12'sd412,
    parameter DW_FIFO_DEPTH        = 16,
    parameter DW_FIFO_AF_LEVEL     = 10,
    parameter PW_OUT_CH            = 64,
    parameter PW_COEFF_W           = 8,
    parameter PW_K_H               = 1,
    parameter PW_K_W               = 1,
    parameter PW_MUL_W             = DW_OUT_WIDTH+PW_COEFF_W,
    parameter PW_SUM_W             = PW_MUL_W+$clog2(PW_K_H*PW_K_W),
    parameter PW_COEFF_GRP_NUM     = PW_OUT_CH*(PINGPONG_ROUNDS+1),
    parameter PW_FRAME_GRP_NUM     = PW_OUT_CH,
    parameter PW_MAC_PIPELINE      = 1,
    parameter PW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_pw.memh",
    parameter PW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_pw_pingpong_bias.hex",
    parameter PW_BIAS_GROUP_SIZE   = (PINGPONG_ROUNDS+1),
    parameter PW_BIAS_GROUP_BITS   = $clog2(PW_BIAS_GROUP_SIZE),
    parameter PW_BIAS_CH_BITS      = $clog2(PW_OUT_CH),
    parameter PW_MULT_CNT          = (PINGPONG_ROUNDS+1),
    parameter [11:0] PW_MULT_FACTOR0 = 12'sd406,
    parameter [11:0] PW_MULT_FACTOR1 = 12'sd461,
    parameter [11:0] PW_MULT_FACTOR2 = 12'sd442,
    parameter [11:0] PW_MULT_FACTOR3 = 12'sd623,
    parameter PW_FIFO_DEPTH        = 4,
    parameter PW_FIFO_AF_LEVEL     = 2,
    parameter RAM_SEGMENTS         = 1,
    parameter RAM_ONE_DATA_W       = PW_SUM_W+$clog2(PW_OUT_CH),
    parameter OUT_PIXEL_WIDTH      = 8
)(
    input  wire clk,
    input  wire rst_n,

    input  wire in_valid,
    output wire in_ready,
    input  wire [IN_DATA_W-1:0] in_pixel,
    input  wire in_end_all_frame,

    output wire out_valid,
    input  wire out_ready,
    output wire end_frame,
    output wire end_all_frame,
    output wire [OUT_PIXEL_WIDTH-1:0] out_data,

    input  wire start,
    output wire busy
);

    localparam integer SEG_CHANS        = PW_OUT_CH / RAM_SEGMENTS;
    localparam integer RAM_DATA_W       = SEG_CHANS * RAM_ONE_DATA_W;
    localparam integer RAM_DEPTH        = PIXEL_DEPTH * RAM_SEGMENTS;
    localparam integer RAM_ADDR_W       = (RAM_DEPTH <= 1) ? 1 : $clog2(RAM_DEPTH);

    // ===== Internal wires connecting controller to submodules =====

    // dw_pw_cac interface
    wire pw_conv_in_valid;
    wire pw_conv_in_ready;
    wire [IN_DATA_W-1:0] pw_conv_in_pixel;
    wire pw_conv_in_end_all_frame;
    wire pw_conv_ram_re;
    wire [RAM_ADDR_W-1:0] pw_conv_ram_raddr;
    wire [RAM_DATA_W-1:0] pw_conv_ram_rdata;
    wire pw_conv_ram_we;
    wire [RAM_ADDR_W-1:0] pw_conv_ram_waddr;
    wire [RAM_DATA_W-1:0] pw_conv_ram_wdata;
    wire pw_conv_end_frame;
    wire pw_conv_end_all_frame;

    // RAM0 interface
    wire ram0_re;
    wire [RAM_ADDR_W-1:0] ram0_raddr;
    wire [RAM_DATA_W-1:0] ram0_rdata;
    wire ram0_we;
    wire [RAM_ADDR_W-1:0] ram0_waddr;
    wire [RAM_DATA_W-1:0] ram0_wdata;

    // RAM1 interface
    wire ram1_re;
    wire [RAM_ADDR_W-1:0] ram1_raddr;
    wire [RAM_DATA_W-1:0] ram1_rdata;
    wire ram1_we;
    wire [RAM_ADDR_W-1:0] ram1_waddr;
    wire [RAM_DATA_W-1:0] ram1_wdata;

    // RAM reader interface
    wire ram_reader_start;
    wire ram_reader_busy;
    wire ram_reader_ram_re;
    wire [RAM_ADDR_W-1:0] ram_reader_ram_raddr;
    wire [RAM_DATA_W-1:0] ram_reader_ram_rdata;
    wire [RAM_ONE_DATA_W-1:0] ram_reader_out_data;
    wire ram_reader_out_valid;
    wire ram_reader_end_frame;
    wire ram_reader_end_all_frame;
    wire ram_reader_out_ready;

    // bias_process interface
    wire bias_process_in_valid;
    wire bias_process_in_ready;
    wire bias_process_in_end_frame;
    wire bias_process_in_end_all_frame;
    wire [RAM_ONE_DATA_W-1:0] bias_process_in_data;
    wire bias_process_out_valid;
    wire bias_process_out_end_frame;
    wire bias_process_out_end_all_frame;
    wire bias_process_out_ready;
    wire [OUT_PIXEL_WIDTH-1:0] bias_process_out_data;

    // ===== Instantiate controller =====
    dw_pw_pingpong_ctrl #(
        .PINGPONG_ROUNDS(PINGPONG_ROUNDS),
        .IN_DATA_W(IN_DATA_W),
        .RAM_DATA_W(RAM_DATA_W),
        .RAM_ADDR_W(RAM_ADDR_W),
        .RAM_ONE_DATA_W(RAM_ONE_DATA_W),
        .OUT_PIXEL_WIDTH(OUT_PIXEL_WIDTH)
    ) u_ctrl (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .in_valid(in_valid),
        .in_pixel(in_pixel),
        .in_end_all_frame(in_end_all_frame),
        .out_ready(out_ready),
        .busy(busy),
        .in_ready(in_ready),
        .out_valid(out_valid),
        .out_data(out_data),
        .out_end_frame(end_frame),
        .out_end_all_frame(end_all_frame),
        
        // Submodule feedback inputs
        .pw_conv_in_ready(pw_conv_in_ready),
        .pw_conv_end_frame(pw_conv_end_frame),
        .pw_conv_end_all_frame(pw_conv_end_all_frame),
        .pw_conv_ram_re(pw_conv_ram_re),
        .pw_conv_ram_we(pw_conv_ram_we),
        .pw_conv_ram_raddr(pw_conv_ram_raddr),
        .pw_conv_ram_waddr(pw_conv_ram_waddr),
        .pw_conv_ram_wdata(pw_conv_ram_wdata),
        
        .ram0_rdata(ram0_rdata),
        .ram1_rdata(ram1_rdata),
        
        .ram_reader_busy(ram_reader_busy),
        .ram_reader_ram_re(ram_reader_ram_re),
        .ram_reader_ram_raddr(ram_reader_ram_raddr),
        .ram_reader_out_valid(ram_reader_out_valid),
        .ram_reader_out_data(ram_reader_out_data),
        .ram_reader_end_frame(ram_reader_end_frame),
        .ram_reader_end_all_frame(ram_reader_end_all_frame),
        
        .bias_process_in_ready(bias_process_in_ready),
        .bias_process_out_valid(bias_process_out_valid),
        .bias_process_out_data(bias_process_out_data),
        .bias_process_out_end_frame(bias_process_out_end_frame),
        .bias_process_out_end_all_frame(bias_process_out_end_all_frame),
        
        // Submodule drive outputs
        .pw_conv_in_valid(pw_conv_in_valid),
        .pw_conv_in_pixel(pw_conv_in_pixel),
        .pw_conv_in_end_all_frame(pw_conv_in_end_all_frame),
        .pw_conv_ram_rdata(pw_conv_ram_rdata),
        
        .ram0_re(ram0_re),
        .ram0_raddr(ram0_raddr),
        .ram0_we(ram0_we),
        .ram0_waddr(ram0_waddr),
        .ram0_wdata(ram0_wdata),
        
        .ram1_re(ram1_re),
        .ram1_raddr(ram1_raddr),
        .ram1_we(ram1_we),
        .ram1_waddr(ram1_waddr),
        .ram1_wdata(ram1_wdata),
        
        .ram_reader_start(ram_reader_start),
        .ram_reader_ram_rdata(ram_reader_ram_rdata),
        .ram_reader_out_ready(ram_reader_out_ready),
        
        .bias_process_in_valid(bias_process_in_valid),
        .bias_process_in_data(bias_process_in_data),
        .bias_process_in_end_frame(bias_process_in_end_frame),
        .bias_process_in_end_all_frame(bias_process_in_end_all_frame),
        .bias_process_out_ready(bias_process_out_ready)
    );

    // ===== Instantiate 5 submodules =====

    dw_pw_cac #(
        .IN_DATA_W(IN_DATA_W),
        .DW_COEFF_W(DW_COEFF_W),
        .DW_K_H(DW_K_H),
        .DW_K_W(DW_K_W),
        .DW_MUL_W(DW_MUL_W),
        .DW_SUM_W(DW_SUM_W),
        .DW_COL(DW_COL),
        .DW_ROW(DW_ROW),
        .DW_STRIDE(DW_STRIDE),
        .DW_PAD_TOP(DW_PAD_TOP),
        .DW_PAD_BOTTOM(DW_PAD_BOTTOM),
        .DW_PAD_LEFT(DW_PAD_LEFT),
        .DW_PAD_RIGHT(DW_PAD_RIGHT),
        .DW_COEFF_GRP_NUM(DW_COEFF_GRP_NUM),
        .DW_FRAME_GRP_NUM(DW_FRAME_GRP_NUM),
        .DW_MAC_PIPELINE(DW_MAC_PIPELINE),
        .DW_COEFF_INIT_FILE(DW_COEFF_INIT_FILE),
        .DW_OUT_WIDTH(DW_OUT_WIDTH),
        .DW_SHIFT_VAL(DW_SHIFT_VAL),
        .DW_BIAS_GROUP_BITS(DW_BIAS_GROUP_BITS),
        .DW_BIAS_GROUP_SIZE(DW_BIAS_GROUP_SIZE),
        .DW_BIAS_CH_BITS(DW_BIAS_CH_BITS),
        .DW_BIAS_INIT_FILE(DW_BIAS_INIT_FILE),
        .DW_MULT_CNT(DW_MULT_CNT),
        .DW_MULT_FACTOR0(DW_MULT_FACTOR0),
        .DW_MULT_FACTOR1(DW_MULT_FACTOR1),
        .DW_MULT_FACTOR2(DW_MULT_FACTOR2),
        .DW_MULT_FACTOR3(DW_MULT_FACTOR3),
        .DW_FIFO_DEPTH(DW_FIFO_DEPTH),
        .DW_FIFO_AF_LEVEL(DW_FIFO_AF_LEVEL),
        .PW_OUT_CH(PW_OUT_CH),
        .PW_COEFF_W(PW_COEFF_W),
        .PW_K_H(PW_K_H),
        .PW_K_W(PW_K_W),
        .PW_MUL_W(PW_MUL_W),
        .PW_SUM_W(PW_SUM_W),
        .PW_COEFF_GRP_NUM(PW_COEFF_GRP_NUM),
        .PW_FRAME_GRP_NUM(PW_FRAME_GRP_NUM),
        .PW_MAC_PIPELINE(PW_MAC_PIPELINE),
        .PW_COEFF_INIT_FILE(PW_COEFF_INIT_FILE),
        .PW_FIFO_DEPTH(PW_FIFO_DEPTH),
        .PW_FIFO_AF_LEVEL(PW_FIFO_AF_LEVEL),
        .RAM_ONE_DATA_W(RAM_ONE_DATA_W),
        .SEGMENTS(RAM_SEGMENTS),
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .RAM_ADDR_W(RAM_ADDR_W),
        .IN_FRAME_SIZE(IN_FRAME_SIZE)
    ) u_dw_pw_cac (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(pw_conv_in_valid),
        .in_ready(pw_conv_in_ready),
        .in_pixel(pw_conv_in_pixel),
        .in_end_all_frame(pw_conv_in_end_all_frame),
        .ram_re(pw_conv_ram_re),
        .ram_raddr(pw_conv_ram_raddr),
        .ram_rdata(pw_conv_ram_rdata),
        .ram_we(pw_conv_ram_we),
        .ram_waddr(pw_conv_ram_waddr),
        .ram_wdata(pw_conv_ram_wdata),
        .end_frame(pw_conv_end_frame),
        .end_all_frame(pw_conv_end_all_frame)
    );

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_W),
        .ADDR_WIDTH(RAM_ADDR_W),
        .DEPTH(RAM_DEPTH)
    ) u_ram0 (
        .clk(clk),
        .re(ram0_re),
        .raddr(ram0_raddr),
        .rdata(ram0_rdata),
        .we(ram0_we),
        .waddr(ram0_waddr),
        .wdata(ram0_wdata)
    );

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_W),
        .ADDR_WIDTH(RAM_ADDR_W),
        .DEPTH(RAM_DEPTH)
    ) u_ram1 (
        .clk(clk),
        .re(ram1_re),
        .raddr(ram1_raddr),
        .rdata(ram1_rdata),
        .we(ram1_we),
        .waddr(ram1_waddr),
        .wdata(ram1_wdata)
    );

    ram_frame_to_fifo_reader #(
        .PIXELS_PER_FRAME(PIXEL_DEPTH),
        .SEGMENTS(RAM_SEGMENTS),
        .CHANNELS(PW_OUT_CH),
        .RAM_DATA_W(RAM_DATA_W),
        .RAM_ADDR_W(RAM_ADDR_W),
        .FIFO_DEPTH(16),
        .FIFO_AF_LEVEL(10),
        .OUT_W(RAM_ONE_DATA_W)
    ) u_ram_reader (
        .clk(clk),
        .rst_n(rst_n),
        .start(ram_reader_start),
        .busy(ram_reader_busy),
        .ram_re(ram_reader_ram_re),
        .ram_raddr(ram_reader_ram_raddr),
        .ram_rdata(ram_reader_ram_rdata),
        .out_valid(ram_reader_out_valid),
        .out_ready(ram_reader_out_ready),
        .out_data(ram_reader_out_data),
        .end_frame(ram_reader_end_frame),
        .end_all_frame(ram_reader_end_all_frame)
    );

    bias_process_wrapper #(
        .IN_WIDTH(RAM_ONE_DATA_W),
        .BIAS_WIDTH(32),
        .OUT_WIDTH(OUT_PIXEL_WIDTH),
        .SHIFT_VAL(DW_SHIFT_VAL),
        .BIAS_INIT_FILE(PW_BIAS_INIT_FILE),
        .GROUP_BITS(PW_BIAS_GROUP_BITS),
        .GROUP_SIZE(PW_BIAS_GROUP_SIZE),
        .CH_BITS(PW_BIAS_CH_BITS),
        .MULT_CNT(PW_MULT_CNT),
        .MULT_FACTOR0(PW_MULT_FACTOR0),
        .MULT_FACTOR1(PW_MULT_FACTOR1),
        .MULT_FACTOR2(PW_MULT_FACTOR2),
        .MULT_FACTOR3(PW_MULT_FACTOR3)
    ) u_bias_process (
        .clk(clk),
        .rst_n(rst_n),
        .in_pixel_data_bus(bias_process_in_data),
        .in_valid(bias_process_in_valid),
        .in_end_frame(bias_process_in_end_frame),
        .in_end_all_frame(bias_process_in_end_all_frame),
        .in_ready(bias_process_in_ready),
        .out_pixel_data(bias_process_out_data),
        .out_valid(bias_process_out_valid),
        .out_end_frame(bias_process_out_end_frame),
        .out_end_all_frame(bias_process_out_end_all_frame),
        .out_ready(bias_process_out_ready)
    );

endmodule