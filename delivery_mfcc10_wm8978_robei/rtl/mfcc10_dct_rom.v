`timescale 1ns / 1ps
module mfcc10_dct_rom #(
    parameter ROM_FILE = "rom/dct10x128_ortho_q17.mem"
) (
    input  wire [11-1:0] addr,
    output reg signed [18-1:0] coeff_q17
);

    reg [18-1:0] rom [0:(10*128)-1];

    initial begin
        $readmemh(ROM_FILE, rom);
    end

    always @(*) begin
        coeff_q17 = rom[addr];
    end

endmodule



