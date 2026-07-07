`timescale 1ns / 1ps
module mfcc10_fft512_twiddle_rom #(
    parameter RE_FILE = "rom/fft512_tw_re_q17.mem",
    parameter IM_FILE = "rom/fft512_tw_im_q17.mem"
) (
    input  wire [9-2:0] addr,
    output reg signed [18-1:0] tw_re,
    output reg signed [18-1:0] tw_im
);

    reg [18-1:0] rom_re [0:(512/2)-1];
    reg [18-1:0] rom_im [0:(512/2)-1];

    initial begin
        $readmemh(RE_FILE, rom_re);
        $readmemh(IM_FILE, rom_im);
    end

    always @(*) begin
        tw_re = rom_re[addr];
        tw_im = rom_im[addr];
    end

endmodule


