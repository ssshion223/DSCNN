`timescale 1ns / 1ps
module mfcc10_mfcc_block (
    input  wire                         clk,
    input  wire                         rst_n,

    input  wire                         mel_valid,
    input  wire [7-1:0] mel_index,
    input  wire [84-1:0] mel_data,
    input  wire                         mel_frame_done,

    output reg                          coeff_valid,
    input  wire                         coeff_ready,
    output reg  [6-1:0] coeff_frame_index,
    output reg  [4-1:0]  coeff_index,
    output reg  signed [32-1:0] coeff_data,
    output reg                          block_done
);

    localparam NUM_MELS     = 128;
    localparam NUM_FRAMES   = 49;
    localparam NUM_MFCC     = 10;
    localparam BLOCK_STEP   = 10;
    localparam RING_FRAMES  = NUM_FRAMES + BLOCK_STEP;
    localparam LOG_DEPTH    = RING_FRAMES * NUM_MELS;
    localparam LOG_ADDR_W   = 13;
    localparam LOG_USER_W   = 6 + 7;
    localparam LOAD_CNT_W   = 7;

    localparam S_IDLE         = 4'd0;
    localparam S_FIND_ISSUE   = 4'd1;
    localparam S_FIND_DRAIN   = 4'd2;
    localparam S_PREP_DCT     = 4'd3;
    localparam S_DCT_FETCH    = 4'd4;
    localparam S_DCT_DB_MUL   = 4'd5;
    localparam S_DCT_DB_CLAMP = 4'd6;
    localparam S_DCT_MUL      = 4'd7;
    localparam S_DCT_ACC      = 4'd8;
    localparam S_SEND         = 4'd9;

    (* ram_style = "block" *) reg [32-1:0] log_ring_mem [0:LOG_DEPTH-1];

    reg  [3:0] state;

    wire                         log_valid;
    wire [32-1:0]     log_q16;
    wire [LOG_USER_W-1:0]        log_user;
    wire [6-1:0] log_frame_index;
    wire [7-1:0]   log_mel_index;

    reg [6-1:0] input_frame_idx;
    reg [LOAD_CNT_W-1:0]          loaded_frame_count;
    reg [6-1:0] frame_step_count;
    reg                           pending_block_valid;
    reg [6-1:0] pending_block_start;
    reg [6-1:0] block_start_frame;

    reg [6-1:0] scan_frame_idx;
    reg [7-1:0]   scan_mel_idx;
    reg [32-1:0]       log_mem_rd_data;
    reg                           find_first;
    reg [32-1:0]       max_log_q16;

    reg [6-1:0] dct_frame_idx;
    reg [4-1:0]  dct_cep_idx;
    reg [7-1:0]   dct_mel_idx;
    reg signed [18-1:0] dct_coeff_r;
    reg signed [64:0]             db_mul_r;
    reg signed [32-1:0] db_clamped_q16_r;
    reg signed [49:0]             dct_prod_r;
    reg signed [63:0]             dct_accum_r;

    wire [18-1:0]      dct_coeff_q17;
    wire signed [32-1:0] min_db_q16;
    wire signed [64:0]            db_mul_shifted;
    wire signed [32-1:0] dct_result_q16_16;
    wire signed [63:0]            dct_accum_sum_wire;
    wire [11-1:0] dct_addr_wire;
    wire                          log_frame_complete;
    wire                          new_block_due;
    wire                          start_block_wire;

    wire [6:0]  scan_frame_sum_wire;
    wire [6:0]  dct_frame_sum_wire;
    wire [6-1:0] scan_ring_frame_wire;
    wire [6-1:0] dct_ring_frame_wire;
    wire [LOG_ADDR_W-1:0]         scan_read_addr_wire;
    wire [LOG_ADDR_W-1:0]         dct_read_addr_wire;
    wire [LOG_ADDR_W-1:0]         log_write_addr_wire;
    wire [LOG_ADDR_W-1:0]         log_read_addr_wire;
    wire                          log_read_en_wire;
    reg [6:0]   block_start_calc;

    assign log_frame_index = log_user[LOG_USER_W-1:7];
    assign log_mel_index   = log_user[7-1:0];
    assign min_db_q16      = -$signed(32'd5242880);
    assign db_mul_shifted  = db_mul_r >>> 16;
    assign dct_accum_sum_wire = dct_accum_r + {{14{dct_prod_r[49]}}, dct_prod_r};
    assign dct_result_q16_16 = dct_accum_sum_wire >>> 17;

    assign scan_frame_sum_wire = {1'b0, block_start_frame} + {1'b0, scan_frame_idx};
    assign dct_frame_sum_wire  = {1'b0, block_start_frame} + {1'b0, dct_frame_idx};
    assign scan_ring_frame_wire = (scan_frame_sum_wire >= RING_FRAMES) ?
                                  (scan_frame_sum_wire - RING_FRAMES) :
                                  scan_frame_sum_wire[6-1:0];
    assign dct_ring_frame_wire = (dct_frame_sum_wire >= RING_FRAMES) ?
                                 (dct_frame_sum_wire - RING_FRAMES) :
                                 dct_frame_sum_wire[6-1:0];
    assign scan_read_addr_wire = (scan_ring_frame_wire * NUM_MELS) + scan_mel_idx;
    assign dct_read_addr_wire  = (dct_ring_frame_wire * NUM_MELS) + dct_mel_idx;
    assign dct_addr_wire       = (dct_cep_idx * NUM_MELS) + dct_mel_idx;
    assign log_write_addr_wire = (log_frame_index * NUM_MELS) + log_mel_index;
    assign log_read_en_wire    = (state == S_FIND_ISSUE) || (state == S_DCT_FETCH);
    assign log_read_addr_wire  = (state == S_FIND_ISSUE) ? scan_read_addr_wire : dct_read_addr_wire;
    assign log_frame_complete  = log_valid && (log_mel_index == (NUM_MELS - 1));
    assign new_block_due       = log_frame_complete &&
                                  (((loaded_frame_count < NUM_FRAMES) &&
                                    (loaded_frame_count == (NUM_FRAMES - 1))) ||
                                   ((loaded_frame_count >= NUM_FRAMES) &&
                                    (frame_step_count == (BLOCK_STEP - 1))));
    assign start_block_wire    = (state == S_IDLE) && pending_block_valid;

    mfcc10_log2_pipe #(
        .IN_W(84),
        .USER_W(LOG_USER_W)
    ) u_log2_pipe (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(mel_valid),
        .in_data(mel_data),
        .in_user({input_frame_idx, mel_index}),
        .out_valid(log_valid),
        .out_log_q16(log_q16),
        .out_user(log_user)
    );

    mfcc10_dct_rom u_dct_rom (
        .addr(dct_addr_wire),
        .coeff_q17(dct_coeff_q17)
    );

    always @(*) begin
        if (log_frame_index >= (NUM_FRAMES - 1)) begin
            block_start_calc = log_frame_index - (NUM_FRAMES - 1);
        end else begin
            block_start_calc = log_frame_index + RING_FRAMES - (NUM_FRAMES - 1);
        end
    end

    always @(posedge clk) begin
        if (log_valid) begin
            log_ring_mem[log_write_addr_wire] <= log_q16;
        end

        if (log_read_en_wire) begin
            log_mem_rd_data <= log_ring_mem[log_read_addr_wire];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_frame_idx      <= {6{1'b0}};
            loaded_frame_count   <= {LOAD_CNT_W{1'b0}};
            frame_step_count     <= {6{1'b0}};
            pending_block_valid  <= 1'b0;
            pending_block_start  <= {6{1'b0}};
        end else begin
            if (log_valid) begin
                if (log_mel_index == (NUM_MELS - 1)) begin
                    if (loaded_frame_count < NUM_FRAMES) begin
                        loaded_frame_count <= loaded_frame_count + 1'b1;
                        if (loaded_frame_count == (NUM_FRAMES - 1)) begin
                            pending_block_valid <= 1'b1;
                            pending_block_start <= block_start_calc[6-1:0];
                            frame_step_count    <= {6{1'b0}};
                        end
                    end else if (frame_step_count == (BLOCK_STEP - 1)) begin
                        pending_block_valid <= 1'b1;
                        pending_block_start <= block_start_calc[6-1:0];
                        frame_step_count    <= {6{1'b0}};
                    end else begin
                        frame_step_count <= frame_step_count + 1'b1;
                    end
                end
            end

            if (mel_frame_done) begin
                if (input_frame_idx == (RING_FRAMES - 1)) begin
                    input_frame_idx <= {6{1'b0}};
                end else begin
                    input_frame_idx <= input_frame_idx + 1'b1;
                end
            end

            if (start_block_wire && !new_block_due) begin
                pending_block_valid <= 1'b0;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            block_start_frame <= {6{1'b0}};
            scan_frame_idx    <= {6{1'b0}};
            scan_mel_idx      <= {7{1'b0}};
            find_first        <= 1'b1;
            max_log_q16       <= {32{1'b0}};
            dct_frame_idx     <= {6{1'b0}};
            dct_cep_idx       <= {4{1'b0}};
            dct_mel_idx       <= {7{1'b0}};
            dct_coeff_r       <= {18{1'b0}};
            db_mul_r          <= 65'sd0;
            db_clamped_q16_r  <= {32{1'b0}};
            dct_prod_r        <= 50'sd0;
            dct_accum_r       <= 64'sd0;
            coeff_valid       <= 1'b0;
            coeff_frame_index <= {6{1'b0}};
            coeff_index       <= {4{1'b0}};
            coeff_data        <= {32{1'b0}};
            block_done        <= 1'b0;
        end else begin
            block_done <= 1'b0;

            case (state)
                S_IDLE: begin
                    coeff_valid <= 1'b0;
                    if (pending_block_valid) begin
                        block_start_frame <= pending_block_start;
                        scan_frame_idx    <= {6{1'b0}};
                        scan_mel_idx      <= {7{1'b0}};
                        find_first        <= 1'b1;
                        max_log_q16       <= {32{1'b0}};
                        state             <= S_FIND_ISSUE;
                    end
                end

                S_FIND_ISSUE: begin
                    state <= S_FIND_DRAIN;
                end

                S_FIND_DRAIN: begin
                    if (find_first || (log_mem_rd_data > max_log_q16)) begin
                        max_log_q16 <= log_mem_rd_data;
                    end
                    find_first <= 1'b0;

                    if ((scan_frame_idx == (NUM_FRAMES - 1)) && (scan_mel_idx == (NUM_MELS - 1))) begin
                        state <= S_PREP_DCT;
                    end else begin
                        if (scan_mel_idx == (NUM_MELS - 1)) begin
                            scan_mel_idx   <= {7{1'b0}};
                            scan_frame_idx <= scan_frame_idx + 1'b1;
                        end else begin
                            scan_mel_idx <= scan_mel_idx + 1'b1;
                        end
                        state <= S_FIND_ISSUE;
                    end
                end

                S_PREP_DCT: begin
                    dct_frame_idx <= {6{1'b0}};
                    dct_cep_idx   <= {4{1'b0}};
                    dct_mel_idx   <= {7{1'b0}};
                    dct_accum_r   <= 64'sd0;
                    state         <= S_DCT_FETCH;
                end

                S_DCT_FETCH: begin
                    dct_coeff_r     <= dct_coeff_q17;
                    state           <= S_DCT_DB_MUL;
                end

                S_DCT_DB_MUL: begin
                    db_mul_r <= ($signed({1'b0, log_mem_rd_data}) - $signed({1'b0, max_log_q16})) *
                                $signed(32'd197281);
                    state    <= S_DCT_DB_CLAMP;
                end

                S_DCT_DB_CLAMP: begin
                    if (db_mul_shifted < min_db_q16) begin
                        db_clamped_q16_r <= min_db_q16;
                    end else if (db_mul_shifted > 0) begin
                        db_clamped_q16_r <= {32{1'b0}};
                    end else begin
                        db_clamped_q16_r <= db_mul_shifted[32-1:0];
                    end
                    state <= S_DCT_MUL;
                end

                S_DCT_MUL: begin
                    dct_prod_r <= db_clamped_q16_r * dct_coeff_r;
                    state      <= S_DCT_ACC;
                end

                S_DCT_ACC: begin
                    if (dct_mel_idx == (NUM_MELS - 1)) begin
                        coeff_valid       <= 1'b1;
                        coeff_frame_index <= dct_frame_idx;
                        coeff_index       <= dct_cep_idx;
                        coeff_data        <= dct_result_q16_16;
                        state             <= S_SEND;
                    end else begin
                        dct_accum_r <= dct_accum_sum_wire;
                        dct_mel_idx <= dct_mel_idx + 1'b1;
                        state       <= S_DCT_FETCH;
                    end
                end

                S_SEND: begin
                    if (coeff_ready) begin
                        coeff_valid <= 1'b0;

                        if ((dct_frame_idx == (NUM_FRAMES - 1)) && (dct_cep_idx == (NUM_MFCC - 1))) begin
                            block_done <= 1'b1;
                            state      <= S_IDLE;
                        end else begin
                            if (dct_cep_idx == (NUM_MFCC - 1)) begin
                                dct_cep_idx   <= {4{1'b0}};
                                dct_frame_idx <= dct_frame_idx + 1'b1;
                            end else begin
                                dct_cep_idx <= dct_cep_idx + 1'b1;
                            end
                            dct_mel_idx <= {7{1'b0}};
                            dct_accum_r <= 64'sd0;
                            state       <= S_DCT_FETCH;
                        end
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
