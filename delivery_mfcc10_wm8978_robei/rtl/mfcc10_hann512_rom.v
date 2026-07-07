`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_hann512_rom #(
    parameter ROM_FILE = "rom/hann512_q17.mem"
) (
    input  wire [`MFCC10_FRAME_ADDR_W-1:0] addr,
    output reg  [`MFCC10_HANN_W-1:0]       coeff_q17
);

    reg [`MFCC10_HANN_W-1:0] rom [0:`MFCC10_FRAME_LEN-1];

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always @(*) begin
        coeff_q17 = rom[addr];
    end

endmodule



