`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_dct_rom #(
    parameter ROM_FILE = "rom/dct10x128_ortho_q17.mem"
) (
    input  wire [`MFCC10_DCT_ADDR_W-1:0] addr,
    output reg signed [`MFCC10_DCT_W-1:0] coeff_q17
);

    reg [`MFCC10_DCT_W-1:0] rom [0:(`MFCC10_NUM_MFCC*`MFCC10_NUM_MELS)-1];

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always @(*) begin
        coeff_q17 = rom[addr];
    end

endmodule



