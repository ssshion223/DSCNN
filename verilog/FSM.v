module FSM #(
    localparam IDLE       = 3'b000,
    localparam LOAD_DATA  = 3'b001,
    localparam DOWNSAMPLE = 3'b010,
    localparam DEPITHWISE = 3'b011,
    localparam AVGPOOL    = 3'b100,
    localparam FC         = 3'b101
)(
    reg [2:0] state,
    input   clk,
    input   rst,

    input   start,
    input   input_ready,
    input   output_ready,
    output  
);


