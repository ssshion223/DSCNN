`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_sparse_mel_rom #(
    parameter ROM_FILE = "rom/sparse_mel128_16000_512_q17.mem"
) (
    input  wire                               clk,
    input  wire [`MFCC10_POWER_ADDR_W-1:0] addr,
    output reg  [`MFCC10_MEL_IDX_W-1:0]    mel_a,
    output reg  [`MFCC10_MEL_WEIGHT_W-1:0] weight_a_q17,
    output reg  [`MFCC10_MEL_IDX_W-1:0]    mel_b,
    output reg  [`MFCC10_MEL_WEIGHT_W-1:0] weight_b_q17
);

    (* rom_style = "block" *) reg [`MFCC10_MEL_ROM_W-1:0] rom [0:`MFCC10_POWER_BINS-1];
    reg [`MFCC10_MEL_ROM_W-1:0] rom_word;

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always @(posedge clk) begin
        rom_word <= rom[addr];
    end

    always @(*) begin
        mel_a        = rom_word[6:0];
        weight_a_q17 = rom_word[24:7];
        mel_b        = rom_word[31:25];
        weight_b_q17 = rom_word[49:32];
    end

endmodule




