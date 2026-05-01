`timescale 1ns / 1ps

module sample_fifo_0 (
    input  wire        clk,
    input  wire        srst,
    input  wire [16:0] din,
    input  wire        wr_en,
    input  wire        rd_en,
    output reg  [16:0] dout,
    output wire        full,
    output wire        empty,
    output reg         valid,
    output wire [9:0]  data_count
);

    localparam integer DATA_W  = 17;
    localparam integer DEPTH   = 1024;
    localparam integer ADDR_W  = 10;
    localparam integer COUNT_W = 11;
    localparam [COUNT_W-1:0] DEPTH_COUNT = DEPTH;

    (* ram_style = "block" *) reg [DATA_W-1:0] mem [0:DEPTH-1];

    reg [ADDR_W-1:0]  wr_ptr;
    reg [ADDR_W-1:0]  rd_ptr;
    reg [COUNT_W-1:0] count;

    wire wr_fire;
    wire rd_fire;

    assign full       = (count == DEPTH_COUNT);
    assign empty      = (count == {COUNT_W{1'b0}});
    assign wr_fire    = wr_en && !full;
    assign rd_fire    = rd_en && !empty;
    assign data_count = count[ADDR_W-1:0];

    always @(posedge clk) begin
        if (wr_fire) begin
            mem[wr_ptr] <= din;
        end
    end

    always @(posedge clk) begin
        if (srst) begin
            wr_ptr <= {ADDR_W{1'b0}};
            rd_ptr <= {ADDR_W{1'b0}};
            count  <= {COUNT_W{1'b0}};
            dout   <= {DATA_W{1'b0}};
            valid  <= 1'b0;
        end else begin
            valid <= rd_fire;

            if (rd_fire) begin
                dout   <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1'b1;
            end

            if (wr_fire) begin
                wr_ptr <= wr_ptr + 1'b1;
            end

            case ({wr_fire, rd_fire})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
