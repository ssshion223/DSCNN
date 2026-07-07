`timescale 1ns / 1ps

module glbl;
    wire GSR;
    wire GTS;
    wire PRLD;
    wire GWE;
    wire PLL_LOCKG;

    assign GSR       = 1'b0;
    assign GTS       = 1'b0;
    assign PRLD      = 1'b0;
    assign GWE       = 1'b1;
    assign PLL_LOCKG = 1'b0;
endmodule
