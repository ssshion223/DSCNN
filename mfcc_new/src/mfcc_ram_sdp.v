// 通用同步简单双口 RAM：一写口、一读口，同一时钟域。
module mfcc_ram_sdp #(
    parameter integer DATA_W = 16,
    parameter integer ADDR_W = 10,
    parameter integer DEPTH  = 1024
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              wr_en,
    input  wire [ADDR_W-1:0] wr_addr,
    input  wire [DATA_W-1:0] wr_data,
    input  wire              rd_en,
    input  wire [ADDR_W-1:0] rd_addr,
    output reg  [DATA_W-1:0] rd_data
);

    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (!rst_n) begin
            rd_data <= {DATA_W{1'b0}};
        end else begin
            if (wr_en) begin
                mem[wr_addr] <= wr_data;
            end

            if (rd_en) begin
                rd_data <= mem[rd_addr];
            end
        end
    end

endmodule
