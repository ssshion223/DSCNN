`timescale 1ns / 1ps
module mfcc10_hann512_rom #(
    parameter ROM_FILE = "rom/hann512_q17.mem"
) (
    input  wire [9-1:0] addr,
    output reg  [18-1:0]       coeff_q17
);

    reg [18-1:0] rom [0:512-1];

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always @(*) begin
        coeff_q17 = rom[addr];
    end

endmodule



