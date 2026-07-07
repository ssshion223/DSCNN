`timescale 1ns / 1ps
module mfcc10_input_fifo #(
    parameter DEPTH = 4096,
    parameter ADDR_W = 12,
    parameter COUNT_W = 13
) (
    input  wire                               clk,
    input  wire                               rst_n,

    input  wire                               src_valid,
    input  wire signed [24-1:0] src_sample,
    output wire                               src_ready,
    output wire                               fifo_full,
    output wire [COUNT_W-1:0]                 data_count,

    output reg                                dst_valid,
    input  wire                               dst_ready,
    output reg signed [24-1:0]  dst_sample,
    output wire                               fifo_empty
);

    localparam [COUNT_W-1:0] DEPTH_COUNT = DEPTH[COUNT_W-1:0];

    (* ram_style = "block" *)
    reg signed [24-1:0] mem [0:DEPTH-1];
    reg [ADDR_W-1:0] wr_ptr;
    reg [ADDR_W-1:0] rd_ptr;
    reg [COUNT_W-1:0] count_r;

    wire fifo_wr_en;
    wire fifo_rd_en;
    wire dst_can_load;

    assign fifo_full    = (count_r == DEPTH_COUNT);
    assign fifo_empty   = (count_r == {COUNT_W{1'b0}}) && !dst_valid;
    assign src_ready    = !fifo_full;
    assign data_count   = count_r + {{(COUNT_W-1){1'b0}}, dst_valid};
    assign fifo_wr_en   = src_valid && src_ready;
    assign dst_can_load = !dst_valid || dst_ready;
    assign fifo_rd_en   = dst_can_load && (count_r != {COUNT_W{1'b0}});

    always @(posedge clk) begin
        if (fifo_wr_en) begin
            mem[wr_ptr] <= src_sample;
        end

        if (fifo_rd_en) begin
            dst_sample <= mem[rd_ptr];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr    <= {ADDR_W{1'b0}};
            rd_ptr    <= {ADDR_W{1'b0}};
            count_r   <= {COUNT_W{1'b0}};
            dst_valid <= 1'b0;
        end else begin
            if (dst_valid && dst_ready && !fifo_rd_en) begin
                dst_valid <= 1'b0;
            end

            if (fifo_rd_en) begin
                rd_ptr    <= rd_ptr + 1'b1;
                dst_valid <= 1'b1;
            end

            if (fifo_wr_en) begin
                wr_ptr <= wr_ptr + 1'b1;
            end

            case ({fifo_wr_en, fifo_rd_en})
                2'b10: count_r <= count_r + 1'b1;
                2'b01: count_r <= count_r - 1'b1;
                default: count_r <= count_r;
            endcase
        end
    end

endmodule
