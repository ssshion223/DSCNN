`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_frame_window (
    input  wire                               clk,
    input  wire                               rst_n,
    input  wire                               sample_valid,
    output wire                               sample_ready,
    input  wire signed [`MFCC10_SAMPLE_W-1:0] sample_data,
    input  wire                               fft_ready,
    output reg                                fft_valid,
    output reg signed [`MFCC10_FFT_IN_W-1:0]  fft_re,
    output reg                                fft_last,
    output reg  [`MFCC10_FFT_ADDR_W-1:0]      fft_index,
    output reg                                frame_done
);

    localparam S_CAPTURE = 2'd0;
    localparam S_WINDOW  = 2'd1;

    reg [1:0] state;
    (* ram_style = "block" *) reg signed [`MFCC10_SAMPLE_W-1:0] frame_mem [0:`MFCC10_FRAME_LEN-1];
    reg [`MFCC10_FRAME_ADDR_W-1:0] capture_idx;
    reg [`MFCC10_FRAME_ADDR_W:0] window_idx;
    reg frame_rd_valid;
    reg signed [`MFCC10_SAMPLE_W-1:0] frame_rd_data;
    reg [`MFCC10_HANN_W-1:0] hann_rd_q17;
    reg [`MFCC10_FFT_ADDR_W-1:0] frame_rd_index;
    reg frame_rd_last;

    wire sample_fire = sample_valid && sample_ready;
    wire fft_fire = fft_valid && fft_ready;
    wire fft_stall = fft_valid && !fft_ready;
    wire [`MFCC10_FRAME_ADDR_W-1:0] window_addr = window_idx[`MFCC10_FRAME_ADDR_W-1:0];
    wire window_read_fire = (state == S_WINDOW) && !fft_stall &&
                            (window_idx != `MFCC10_FRAME_LEN);
    wire [`MFCC10_HANN_W-1:0] hann_q17;
    wire signed [(`MFCC10_SAMPLE_W+`MFCC10_HANN_W)-1:0] win_product;
    wire signed [`MFCC10_FFT_IN_W-1:0] win_scaled;

    assign sample_ready = (state == S_CAPTURE);
    assign win_product = frame_rd_data * $signed({1'b0, hann_rd_q17});
    assign win_scaled = win_product >>> `MFCC10_HANN_FRAC_W;

    mfcc10_hann512_rom u_hann_rom (
        .addr      (window_addr),
        .coeff_q17 (hann_q17)
    );

    always @(posedge clk) begin
        if (sample_fire) begin
            frame_mem[capture_idx] <= sample_data;
        end

        if (window_read_fire) begin
            frame_rd_data  <= frame_mem[window_addr];
            hann_rd_q17    <= hann_q17;
            frame_rd_index <= window_addr;
            frame_rd_last  <= (window_idx == (`MFCC10_FRAME_LEN - 1));
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_CAPTURE;
            capture_idx <= {`MFCC10_FRAME_ADDR_W{1'b0}};
            window_idx  <= {(`MFCC10_FRAME_ADDR_W+1){1'b0}};
            frame_rd_valid <= 1'b0;
            fft_valid   <= 1'b0;
            fft_re      <= {`MFCC10_FFT_IN_W{1'b0}};
            fft_last    <= 1'b0;
            fft_index   <= {`MFCC10_FFT_ADDR_W{1'b0}};
            frame_done  <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            case (state)
                S_CAPTURE: begin
                    fft_valid <= 1'b0;
                    fft_last  <= 1'b0;

                    if (sample_fire) begin
                        if (capture_idx == (`MFCC10_FRAME_LEN - 1)) begin
                            capture_idx <= {`MFCC10_FRAME_ADDR_W{1'b0}};
                            window_idx  <= {(`MFCC10_FRAME_ADDR_W+1){1'b0}};
                            frame_rd_valid <= 1'b0;
                            state       <= S_WINDOW;
                        end else begin
                            capture_idx <= capture_idx + 1'b1;
                        end
                    end
                end

                S_WINDOW: begin
                    if (fft_stall) begin
                        fft_valid <= fft_valid;
                    end else begin
                        if (frame_rd_valid) begin
                            fft_valid <= 1'b1;
                            fft_re    <= win_scaled;
                            fft_index <= frame_rd_index;
                            fft_last  <= frame_rd_last;
                        end else begin
                            fft_valid <= 1'b0;
                            fft_last  <= 1'b0;
                        end

                        frame_rd_valid <= window_read_fire;
                        if (window_read_fire) begin
                            window_idx <= window_idx + 1'b1;
                        end else if (!frame_rd_valid) begin
                            fft_valid  <= 1'b0;
                            fft_last   <= 1'b0;
                            window_idx <= {(`MFCC10_FRAME_ADDR_W+1){1'b0}};
                            frame_done <= 1'b1;
                            state      <= S_CAPTURE;
                        end
                    end
                end

                default: begin
                    state <= S_CAPTURE;
                end
            endcase
        end
    end

endmodule

