`timescale 1ns / 1ps
module mfcc10_sparse_mel_rom #(
    parameter ROM_FILE = "rom/sparse_mel128_16000_512_q17.mem"
) (
    input  wire                               clk,
    input  wire [9-1:0] addr,
    output reg  [7-1:0]    mel_a,
    output reg  [18-1:0] weight_a_q17,
    output reg  [7-1:0]    mel_b,
    output reg  [18-1:0] weight_b_q17
);

    (* rom_style = "block" *) reg [50-1:0] rom [0:257-1];
    reg [50-1:0] rom_word;

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




