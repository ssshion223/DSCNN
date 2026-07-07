`timescale 1ns / 1ps

module mfcc10_audio_pll_8m192_nobuf (
    input  wire clk_in,
    input  wire reset,
    output wire clk_out,
    output wire locked
);

    localparam [31:0] PHASE_INC_8M192_FROM_50M = 32'd703687442;
    localparam [7:0]  LOCK_DELAY_CYCLES        = 8'd64;

    reg [31:0] phase_acc;
    reg [7:0]  lock_count;
    reg        locked_r;

    assign clk_out = phase_acc[31];
    assign locked  = locked_r;

    always @(posedge clk_in or posedge reset) begin
        if (reset) begin
            phase_acc  <= 32'd0;
            lock_count <= 8'd0;
            locked_r   <= 1'b0;
        end else begin
            phase_acc <= phase_acc + PHASE_INC_8M192_FROM_50M;

            if (!locked_r) begin
                if (lock_count == LOCK_DELAY_CYCLES) begin
                    locked_r <= 1'b1;
                end else begin
                    lock_count <= lock_count + 1'b1;
                end
            end
        end
    end

endmodule
