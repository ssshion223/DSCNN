`timescale 1ns / 1ps

module mfcc10_audio_pll_8m192_nobuf (
    input  wire clk_in,
    input  wire reset,
    output wire clk_out,
    output wire locked
);

    wire clkfbout;
    wire clkfbout_buf;
    wire clkout0;
    wire locked_int;

    wire clkfboutb_unused;
    wire clkout0b_unused;
    wire clkout1_unused;
    wire clkout1b_unused;
    wire clkout2_unused;
    wire clkout2b_unused;
    wire clkout3_unused;
    wire clkout3b_unused;
    wire clkout4_unused;
    wire clkout5_unused;
    wire clkout6_unused;
    wire [15:0] do_unused;
    wire drdy_unused;
    wire psdone_unused;
    wire clkfbstopped_unused;
    wire clkinstopped_unused;

    MMCME2_ADV #(
        .BANDWIDTH            ("OPTIMIZED"),
        .CLKOUT4_CASCADE      ("FALSE"),
        .COMPENSATION         ("ZHOLD"),
        .STARTUP_WAIT         ("FALSE"),
        .DIVCLK_DIVIDE        (5),
        .CLKFBOUT_MULT_F      (64.000),
        .CLKFBOUT_PHASE       (0.000),
        .CLKFBOUT_USE_FINE_PS ("FALSE"),
        .CLKOUT0_DIVIDE_F     (78.125),
        .CLKOUT0_PHASE        (0.000),
        .CLKOUT0_DUTY_CYCLE   (0.500),
        .CLKOUT0_USE_FINE_PS  ("FALSE"),
        .CLKIN1_PERIOD        (20.000)
    ) u_mmcm (
        .CLKFBOUT     (clkfbout),
        .CLKFBOUTB    (clkfboutb_unused),
        .CLKOUT0      (clkout0),
        .CLKOUT0B     (clkout0b_unused),
        .CLKOUT1      (clkout1_unused),
        .CLKOUT1B     (clkout1b_unused),
        .CLKOUT2      (clkout2_unused),
        .CLKOUT2B     (clkout2b_unused),
        .CLKOUT3      (clkout3_unused),
        .CLKOUT3B     (clkout3b_unused),
        .CLKOUT4      (clkout4_unused),
        .CLKOUT5      (clkout5_unused),
        .CLKOUT6      (clkout6_unused),
        .CLKFBIN      (clkfbout_buf),
        .CLKIN1       (clk_in),
        .CLKIN2       (1'b0),
        .CLKINSEL     (1'b1),
        .DADDR        (7'h00),
        .DCLK         (1'b0),
        .DEN          (1'b0),
        .DI           (16'h0000),
        .DO           (do_unused),
        .DRDY         (drdy_unused),
        .DWE          (1'b0),
        .PSCLK        (1'b0),
        .PSEN         (1'b0),
        .PSINCDEC     (1'b0),
        .PSDONE       (psdone_unused),
        .LOCKED       (locked_int),
        .CLKINSTOPPED (clkinstopped_unused),
        .CLKFBSTOPPED (clkfbstopped_unused),
        .PWRDWN       (1'b0),
        .RST          (reset)
    );

    BUFG u_clkfb_buf (
        .I (clkfbout),
        .O (clkfbout_buf)
    );

    BUFG u_audio_clk_buf (
        .I (clkout0),
        .O (clk_out)
    );

    assign locked = locked_int;

endmodule
