`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_fft512_twiddle_rom #(
    parameter RE_FILE = "rom/fft512_tw_re_q17.mem",
    parameter IM_FILE = "rom/fft512_tw_im_q17.mem"
) (
    input  wire [`MFCC10_FFT_ADDR_W-2:0] addr,
    output reg signed [`MFCC10_FFT_TW_W-1:0] tw_re,
    output reg signed [`MFCC10_FFT_TW_W-1:0] tw_im
);

    reg [`MFCC10_FFT_TW_W-1:0] rom_re [0:(`MFCC10_FFT_LEN/2)-1];
    reg [`MFCC10_FFT_TW_W-1:0] rom_im [0:(`MFCC10_FFT_LEN/2)-1];

    initial begin
        $readmemh(RE_FILE, rom_re);
        $readmemh(IM_FILE, rom_im);
    end

    always @(*) begin
        tw_re = rom_re[addr];
        tw_im = rom_im[addr];
    end

endmodule


