`timescale 1ns / 1ps

module mfcc10_uart_tx #(
    parameter CLK_HZ = 50000000,
    parameter BAUD   = 115200
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       tx_valid,
    output wire       tx_ready,
    input  wire [7:0] tx_data,

    output reg        txd,
    output reg        busy
);

    localparam CLKS_PER_BIT = CLK_HZ / BAUD;

    reg [9:0]  shift_reg;
    reg [15:0] clk_count;
    reg [3:0]  bit_index;

    assign tx_ready = !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg <= 10'h3ff;
            clk_count <= 16'd0;
            bit_index <= 4'd0;
            txd       <= 1'b1;
            busy      <= 1'b0;
        end else begin
            if (!busy) begin
                txd <= 1'b1;
                if (tx_valid) begin
                    shift_reg <= {1'b1, tx_data, 1'b0};
                    clk_count <= 16'd0;
                    bit_index <= 4'd0;
                    txd       <= 1'b0;
                    busy      <= 1'b1;
                end
            end else begin
                if (clk_count == (CLKS_PER_BIT - 1)) begin
                    clk_count <= 16'd0;
                    if (bit_index == 4'd9) begin
                        bit_index <= 4'd0;
                        txd       <= 1'b1;
                        busy      <= 1'b0;
                    end else begin
                        bit_index <= bit_index + 1'b1;
                        shift_reg <= {1'b1, shift_reg[9:1]};
                        txd       <= shift_reg[1];
                    end
                end else begin
                    clk_count <= clk_count + 1'b1;
                end
            end
        end
    end

endmodule
