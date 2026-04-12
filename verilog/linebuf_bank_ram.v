module linebuf_bank_ram #(
    parameter integer DATA_W = 8,
    parameter integer DEPTH  = 64,
    parameter integer ADDR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH)
)(
    input  wire                     clk,
    input  wire                     we,
    input  wire [ADDR_W-1:0]        addr,
    input  wire signed [DATA_W-1:0] din,
    output wire signed [DATA_W-1:0] dout
);
    (* ram_style = "distributed" *)
    reg signed [DATA_W-1:0] mem [0:DEPTH-1];

    assign dout = mem[addr];

    always @(posedge clk) begin
        if (we) begin
            mem[addr] <= din;
        end
    end
endmodule
