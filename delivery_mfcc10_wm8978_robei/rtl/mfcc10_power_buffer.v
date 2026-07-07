`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_power_buffer (
    input  wire                              clk,
    input  wire                              rst_n,

    input  wire                              in_valid,
    input  wire [`MFCC10_POWER_ADDR_W-1:0]   in_index,
    input  wire [`MFCC10_POWER_W-1:0]        in_power,
    input  wire                              in_last,

    output reg                               out_valid,
    input  wire                              out_ready,
    output reg [`MFCC10_POWER_ADDR_W-1:0]    out_index,
    output reg [`MFCC10_POWER_W-1:0]         out_power,
    output reg                               out_last,
    output reg                               frame_done
);

    localparam S_LOAD = 1'b0;
    localparam S_SEND = 1'b1;

    reg state;
    reg [`MFCC10_POWER_ADDR_W:0] send_idx;
    (* ram_style = "block" *) reg [`MFCC10_POWER_W-1:0] power_mem [0:`MFCC10_POWER_BINS-1];
    reg send_rd_valid;
    reg [`MFCC10_POWER_ADDR_W-1:0] power_rd_index;
    reg [`MFCC10_POWER_W-1:0] power_rd_data;
    reg power_rd_last;

    wire send_stall = out_valid && !out_ready;
    wire [`MFCC10_POWER_ADDR_W-1:0] send_addr = send_idx[`MFCC10_POWER_ADDR_W-1:0];
    wire send_read_fire = (state == S_SEND) && !send_stall &&
                          (send_idx != `MFCC10_POWER_BINS);

    always @(posedge clk) begin
        if (in_valid) begin
            power_mem[in_index] <= in_power;
        end

        if (send_read_fire) begin
            power_rd_index <= send_addr;
            power_rd_data  <= power_mem[send_addr];
            power_rd_last  <= (send_idx == (`MFCC10_POWER_BINS - 1));
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_LOAD;
            send_idx   <= {(`MFCC10_POWER_ADDR_W+1){1'b0}};
            send_rd_valid <= 1'b0;
            out_valid  <= 1'b0;
            out_index  <= {`MFCC10_POWER_ADDR_W{1'b0}};
            out_power  <= {`MFCC10_POWER_W{1'b0}};
            out_last   <= 1'b0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            case (state)
                S_LOAD: begin
                    out_valid <= 1'b0;
                    out_last  <= 1'b0;
                    send_idx  <= {(`MFCC10_POWER_ADDR_W+1){1'b0}};
                    send_rd_valid <= 1'b0;

                    if (in_valid) begin
                        if (in_last) begin
                            state <= S_SEND;
                        end
                    end
                end

                S_SEND: begin
                    if (send_stall) begin
                        out_valid <= out_valid;
                    end else begin
                        if (send_rd_valid) begin
                            out_valid <= 1'b1;
                            out_index <= power_rd_index;
                            out_power <= power_rd_data;
                            out_last  <= power_rd_last;
                        end else begin
                            out_valid <= 1'b0;
                            out_last  <= 1'b0;
                        end

                        send_rd_valid <= send_read_fire;
                        if (send_read_fire) begin
                            send_idx <= send_idx + 1'b1;
                        end else if (!send_rd_valid) begin
                            out_valid  <= 1'b0;
                            out_last   <= 1'b0;
                            send_idx   <= {(`MFCC10_POWER_ADDR_W+1){1'b0}};
                            frame_done <= 1'b1;
                            state      <= S_LOAD;
                        end
                    end
                end

                default: begin
                    state <= S_LOAD;
                end
            endcase
        end
    end

endmodule
