`include "mfcc_defs.vh"

`timescale 1ns / 1ps

module mfcc_fft512_intfftk #(
    parameter IN_W   = `MFCC_SAMPLE_W,
    parameter FFT_W  = `MFCC_FFT_W,
    parameter POINTS = `MFCC_FFT_LEN
) (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           start,
    input  wire                           in_valid,
    input  wire [IN_W-1:0]                in_re,
    input  wire [IN_W-1:0]                in_im,
    input  wire                           in_last,
    output wire                           in_ready,
    output reg                            busy,
    output reg                            done,
    output reg                            out_valid,
    output reg  [`MFCC_POWER_ADDR_W-1:0]  out_index,
    output reg  [FFT_W-1:0]               out_re,
    output reg  [FFT_W-1:0]               out_im,
    output reg                            out_last
);

    localparam integer NFFT   = 9;
    localparam integer ADDR_W = `MFCC_POWER_ADDR_W;
    localparam integer TW_W   = 16;
    localparam integer MEM_W  = 2 * FFT_W;
    localparam [ADDR_W:0] POINTS_EXT = POINTS;

    localparam S_IDLE        = 3'd0;
    localparam S_LOAD        = 3'd1;
    localparam S_CALC_READ   = 3'd2;
    localparam S_CALC_LATCH  = 3'd3;
    localparam S_CALC_MUL    = 3'd4;
    localparam S_CALC_WRITE  = 3'd5;
    localparam S_OUTPUT_READ = 3'd6;
    localparam S_OUTPUT_EMIT = 3'd7;

    reg [2:0]        state;
    reg [ADDR_W-1:0] load_count;
    reg [ADDR_W-1:0] output_count;
    reg [3:0]        stage;
    reg [ADDR_W-1:0] block_base;
    reg [ADDR_W-1:0] butterfly_j;

    reg signed [TW_W-1:0] tw_re [0:(POINTS/2)-1];
    reg signed [TW_W-1:0] tw_im [0:(POINTS/2)-1];

    reg [ADDR_W-1:0] ram_a_addr;
    reg [ADDR_W-1:0] ram_b_addr;
    reg [MEM_W-1:0]  ram_a_din;
    reg [MEM_W-1:0]  ram_b_din;
    reg              ram_a_we;
    reg              ram_b_we;
    reg [MEM_W-1:0]  ram_a_dout;
    reg [MEM_W-1:0]  ram_b_dout;

    (* ram_style = "block" *) reg [MEM_W-1:0] work_mem [0:POINTS-1];

    reg signed [FFT_W-1:0] u_re_r;
    reg signed [FFT_W-1:0] u_im_r;
    reg signed [FFT_W-1:0] b_re_r;
    reg signed [FFT_W-1:0] b_im_r;
    reg signed [FFT_W-1:0] t_re_r;
    reg signed [FFT_W-1:0] t_im_r;
    reg signed [TW_W-1:0]  w_re_r;
    reg signed [TW_W-1:0]  w_im_r;

    wire load_fire = (state == S_LOAD) && in_valid && in_ready;
    assign in_ready = (state == S_LOAD);

    wire [ADDR_W:0] half_size = ({{ADDR_W{1'b0}}, 1'b1} << (stage - 1'b1));
    wire [ADDR_W:0] step_size = ({{ADDR_W{1'b0}}, 1'b1} << stage);
    wire [ADDR_W:0] idx_a_ext = {1'b0, block_base} + {1'b0, butterfly_j};
    wire [ADDR_W:0] idx_b_ext = idx_a_ext + half_size;
    wire [ADDR_W-1:0] idx_a = idx_a_ext[ADDR_W-1:0];
    wire [ADDR_W-1:0] idx_b = idx_b_ext[ADDR_W-1:0];
    wire [ADDR_W-1:0] tw_idx = butterfly_j << (NFFT - stage);
    wire [ADDR_W:0] next_block = {1'b0, block_base} + step_size;

    wire signed [FFT_W+TW_W-1:0] prod_rr = b_re_r * w_re_r;
    wire signed [FFT_W+TW_W-1:0] prod_ii = b_im_r * w_im_r;
    wire signed [FFT_W+TW_W-1:0] prod_ri = b_re_r * w_im_r;
    wire signed [FFT_W+TW_W-1:0] prod_ir = b_im_r * w_re_r;

    wire signed [FFT_W+TW_W:0] t_re_full = {prod_rr[FFT_W+TW_W-1], prod_rr} -
                                           {prod_ii[FFT_W+TW_W-1], prod_ii};
    wire signed [FFT_W+TW_W:0] t_im_full = {prod_ri[FFT_W+TW_W-1], prod_ri} +
                                           {prod_ir[FFT_W+TW_W-1], prod_ir};
    wire signed [FFT_W-1:0] t_re_calc = t_re_full >>> (TW_W - 1);
    wire signed [FFT_W-1:0] t_im_calc = t_im_full >>> (TW_W - 1);

    wire signed [FFT_W-1:0] sum_re = u_re_r + t_re_r;
    wire signed [FFT_W-1:0] sum_im = u_im_r + t_im_r;
    wire signed [FFT_W-1:0] dif_re = u_re_r - t_re_r;
    wire signed [FFT_W-1:0] dif_im = u_im_r - t_im_r;

    function [ADDR_W-1:0] bit_reverse;
        input [ADDR_W-1:0] value;
        integer b;
        begin
            for (b = 0; b < ADDR_W; b = b + 1) begin
                bit_reverse[b] = value[ADDR_W-1-b];
            end
        end
    endfunction

    function signed [FFT_W-1:0] sext_in;
        input [IN_W-1:0] value;
        begin
            sext_in = {{(FFT_W-IN_W){value[IN_W-1]}}, value};
        end
    endfunction

    function [MEM_W-1:0] pack_complex;
        input [FFT_W-1:0] re;
        input [FFT_W-1:0] im;
        begin
            pack_complex = {re, im};
        end
    endfunction

    integer i;
    real phase;
    real scale;
    initial begin
        scale = (2.0 ** (TW_W - 1)) - 1.0;
        for (i = 0; i < POINTS/2; i = i + 1) begin
            phase = 2.0 * 3.14159265358979323846 * i / POINTS;
            tw_re[i] = $rtoi(scale * $cos(phase));
            tw_im[i] = $rtoi(-scale * $sin(phase));
        end
    end

    always @(*) begin
        ram_a_we   = 1'b0;
        ram_b_we   = 1'b0;
        ram_a_addr = {ADDR_W{1'b0}};
        ram_b_addr = {ADDR_W{1'b0}};
        ram_a_din  = {MEM_W{1'b0}};
        ram_b_din  = {MEM_W{1'b0}};

        case (state)
            S_LOAD: begin
                ram_a_addr = bit_reverse(load_count);
                ram_a_din  = pack_complex(sext_in(in_re), sext_in(in_im));
                ram_a_we   = load_fire;
            end

            S_CALC_READ: begin
                ram_a_addr = idx_a;
                ram_b_addr = idx_b;
            end

            S_CALC_WRITE: begin
                ram_a_addr = idx_a;
                ram_b_addr = idx_b;
                ram_a_din  = pack_complex(sum_re, sum_im);
                ram_b_din  = pack_complex(dif_re, dif_im);
                ram_a_we   = 1'b1;
                ram_b_we   = 1'b1;
            end

            S_OUTPUT_READ: begin
                ram_a_addr = output_count;
            end

            default: begin
                ram_a_addr = {ADDR_W{1'b0}};
                ram_b_addr = {ADDR_W{1'b0}};
            end
        endcase
    end

    always @(posedge clk) begin
        if (ram_a_we) begin
            work_mem[ram_a_addr] <= ram_a_din;
        end
        ram_a_dout <= work_mem[ram_a_addr];
    end

    always @(posedge clk) begin
        if (ram_b_we) begin
            work_mem[ram_b_addr] <= ram_b_din;
        end
        ram_b_dout <= work_mem[ram_b_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            load_count   <= {ADDR_W{1'b0}};
            output_count <= {ADDR_W{1'b0}};
            stage        <= 4'd1;
            block_base   <= {ADDR_W{1'b0}};
            butterfly_j  <= {ADDR_W{1'b0}};
            busy         <= 1'b0;
            done         <= 1'b0;
            out_valid    <= 1'b0;
            out_index    <= {ADDR_W{1'b0}};
            out_re       <= {FFT_W{1'b0}};
            out_im       <= {FFT_W{1'b0}};
            out_last     <= 1'b0;
            u_re_r       <= {FFT_W{1'b0}};
            u_im_r       <= {FFT_W{1'b0}};
            b_re_r       <= {FFT_W{1'b0}};
            b_im_r       <= {FFT_W{1'b0}};
            t_re_r       <= {FFT_W{1'b0}};
            t_im_r       <= {FFT_W{1'b0}};
            w_re_r       <= {TW_W{1'b0}};
            w_im_r       <= {TW_W{1'b0}};
        end else begin
            done      <= 1'b0;
            out_valid <= 1'b0;
            out_last  <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        state        <= S_LOAD;
                        busy         <= 1'b1;
                        load_count   <= {ADDR_W{1'b0}};
                        output_count <= {ADDR_W{1'b0}};
                    end
                end

                S_LOAD: begin
                    busy <= 1'b1;
                    if (load_fire) begin
                        if ((load_count == POINTS-1) || in_last) begin
                            state       <= S_CALC_READ;
                            stage       <= 4'd1;
                            block_base  <= {ADDR_W{1'b0}};
                            butterfly_j <= {ADDR_W{1'b0}};
                        end else begin
                            load_count <= load_count + 1'b1;
                        end
                    end
                end

                S_CALC_READ: begin
                    busy  <= 1'b1;
                    state <= S_CALC_LATCH;
                end

                S_CALC_LATCH: begin
                    u_re_r <= ram_a_dout[MEM_W-1:FFT_W];
                    u_im_r <= ram_a_dout[FFT_W-1:0];
                    b_re_r <= ram_b_dout[MEM_W-1:FFT_W];
                    b_im_r <= ram_b_dout[FFT_W-1:0];
                    w_re_r <= tw_re[tw_idx];
                    w_im_r <= tw_im[tw_idx];
                    state  <= S_CALC_MUL;
                end

                S_CALC_MUL: begin
                    t_re_r <= t_re_calc;
                    t_im_r <= t_im_calc;
                    state  <= S_CALC_WRITE;
                end

                S_CALC_WRITE: begin
                    if (butterfly_j == (half_size[ADDR_W-1:0] - 1'b1)) begin
                        butterfly_j <= {ADDR_W{1'b0}};
                        if (next_block >= POINTS_EXT) begin
                            block_base <= {ADDR_W{1'b0}};
                            if (stage == NFFT) begin
                                state        <= S_OUTPUT_READ;
                                output_count <= {ADDR_W{1'b0}};
                            end else begin
                                stage <= stage + 1'b1;
                                state <= S_CALC_READ;
                            end
                        end else begin
                            block_base <= next_block[ADDR_W-1:0];
                            state      <= S_CALC_READ;
                        end
                    end else begin
                        butterfly_j <= butterfly_j + 1'b1;
                        state       <= S_CALC_READ;
                    end
                end

                S_OUTPUT_READ: begin
                    busy  <= 1'b1;
                    state <= S_OUTPUT_EMIT;
                end

                S_OUTPUT_EMIT: begin
                    busy      <= 1'b1;
                    out_valid <= 1'b1;
                    out_index <= output_count;
                    out_re    <= ram_a_dout[MEM_W-1:FFT_W];
                    out_im    <= ram_a_dout[FFT_W-1:0];

                    if (output_count == POINTS-1) begin
                        out_last     <= 1'b1;
                        done         <= 1'b1;
                        busy         <= 1'b0;
                        output_count <= {ADDR_W{1'b0}};
                        state        <= S_IDLE;
                    end else begin
                        output_count <= output_count + 1'b1;
                        state        <= S_OUTPUT_READ;
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
