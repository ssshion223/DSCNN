`timescale 1ns / 1ps
`include "mfcc10_defs.vh"

module mfcc10_mfcc_block (
    input  wire                             clk,
    input  wire                             rst_n,

    input  wire                             mel_valid,
    input  wire [`MFCC10_MEL_IDX_W-1:0]     mel_index,
    input  wire [`MFCC10_MEL_ACC_W-1:0]     mel_data,
    input  wire                             mel_frame_done,

    output reg                              coeff_valid,
    input  wire                             coeff_ready,
    output reg [`MFCC10_FRAME_IDX_W-1:0]    coeff_frame_index,
    output reg [`MFCC10_MFCC_IDX_W-1:0]     coeff_index,
    output reg signed [`MFCC10_OUT_W-1:0]   coeff_data,
    output reg                              block_done
);

    localparam S_LOAD      = 4'd0;
    localparam S_PREP_DCT  = 4'd1;
    localparam S_DCT_FETCH = 4'd2;
    localparam S_DCT_CLAMP = 4'd3;
    localparam S_DCT_MUL   = 4'd4;
    localparam S_DCT_ACC   = 4'd5;
    localparam S_SEND      = 4'd6;

    localparam integer LOG_DEPTH = `MFCC10_FRAME_COUNT * `MFCC10_NUM_MELS;
    localparam integer LOG_USER_W = `MFCC10_FRAME_IDX_W + `MFCC10_MEL_IDX_W;

    reg [3:0] state;
    reg [`MFCC10_FRAME_IDX_W-1:0] load_frame_idx;
    reg [`MFCC10_FRAME_IDX_W-1:0] dct_frame_idx;
    reg [`MFCC10_MFCC_IDX_W-1:0] dct_cep_idx;
    reg [`MFCC10_MEL_IDX_W-1:0] dct_mel_idx;

    reg signed [`MFCC10_DB_W-1:0] max_db_q16;
    reg signed [`MFCC10_DB_W-1:0] min_db_q16;
    (* ram_style = "block" *) reg signed [`MFCC10_DB_W-1:0] db_mem [0:LOG_DEPTH-1];
    reg signed [`MFCC10_DB_W-1:0] db_mem_rd_data;
    reg signed [`MFCC10_DB_W-1:0] dct_db_r;
    reg signed [`MFCC10_DCT_W-1:0] dct_coeff_r;
    reg signed [(`MFCC10_DB_W+`MFCC10_DCT_W)-1:0] dct_prod_r;
    reg signed [63:0] dct_accum;
    reg signed [`MFCC10_OUT_W-1:0] dct_out_r;

    wire log_valid;
    wire [`MFCC10_LOG_W-1:0] log_q16;
    wire [LOG_USER_W-1:0] log_user;
    wire [`MFCC10_FRAME_IDX_W-1:0] log_frame_index;
    wire [`MFCC10_MEL_IDX_W-1:0] log_index;
    wire [`MFCC10_DCT_ADDR_W-1:0] dct_addr;
    wire signed [`MFCC10_DCT_W-1:0] dct_coeff_q17;
    wire [15:0] log_count_next;
    wire signed [`MFCC10_DB_W-1:0] max_db_next_wire;
    wire signed [`MFCC10_DB_W-1:0] db_read_clamped_wire;
    wire signed [63:0] dct_prod_ext;
    wire send_fire;

    reg [15:0] log_count;
    reg load_s0_valid;
    reg [`MFCC10_FRAME_IDX_W-1:0] load_s0_frame_idx;
    reg [`MFCC10_MEL_IDX_W-1:0] load_s0_mel_idx;
    reg signed [`MFCC10_DB_W-1:0] load_s0_log_input;
    reg load_s1_valid;
    reg [`MFCC10_FRAME_IDX_W-1:0] load_s1_frame_idx;
    reg [`MFCC10_MEL_IDX_W-1:0] load_s1_mel_idx;
    reg signed [63:0] load_s1_db_prod;
    reg load_s2_valid;
    reg [`MFCC10_FRAME_IDX_W-1:0] load_s2_frame_idx;
    reg [`MFCC10_MEL_IDX_W-1:0] load_s2_mel_idx;
    reg signed [`MFCC10_DB_W-1:0] load_s2_db_q16;

    assign log_count_next = log_count + 1'b1;
    assign dct_addr = (dct_cep_idx * `MFCC10_NUM_MELS) + dct_mel_idx;
    assign max_db_next_wire = ((log_count == 16'd0) || (load_s2_db_q16 > max_db_q16)) ? load_s2_db_q16 : max_db_q16;
    assign db_read_clamped_wire = (db_mem_rd_data < min_db_q16) ? min_db_q16 : db_mem_rd_data;
    assign dct_prod_ext = {{(64-(`MFCC10_DB_W+`MFCC10_DCT_W)){dct_prod_r[`MFCC10_DB_W+`MFCC10_DCT_W-1]}}, dct_prod_r};
    assign send_fire = coeff_valid && coeff_ready;
    assign log_frame_index = log_user[LOG_USER_W-1:`MFCC10_MEL_IDX_W];
    assign log_index = log_user[`MFCC10_MEL_IDX_W-1:0];

    mfcc10_log2_pipe #(
        .IN_W(`MFCC10_MEL_ACC_W),
        .USER_W(LOG_USER_W)
    ) u_log2 (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_valid    (mel_valid),
        .in_data     (mel_data),
        .in_user     ({load_frame_idx, mel_index}),
        .out_valid   (log_valid),
        .out_log_q16 (log_q16),
        .out_user    (log_user)
    );

    mfcc10_dct_rom u_dct_rom (
        .addr       (dct_addr),
        .coeff_q17  (dct_coeff_q17)
    );

    function signed [`MFCC10_OUT_W-1:0] sat_out;
        input signed [63:0] value;
        begin
            if (value > $signed({1'b0, {(`MFCC10_OUT_W-1){1'b1}}})) begin
                sat_out = {1'b0, {(`MFCC10_OUT_W-1){1'b1}}};
            end else if (value < $signed({1'b1, {(`MFCC10_OUT_W-1){1'b0}}})) begin
                sat_out = {1'b1, {(`MFCC10_OUT_W-1){1'b0}}};
            end else begin
                sat_out = value[`MFCC10_OUT_W-1:0];
            end
        end
    endfunction

    always @(posedge clk) begin
        if (load_s2_valid && (state == S_LOAD)) begin
            db_mem[(load_s2_frame_idx * `MFCC10_NUM_MELS) + load_s2_mel_idx] <= load_s2_db_q16;
        end

        if (state == S_DCT_FETCH) begin
            db_mem_rd_data <= db_mem[(dct_frame_idx * `MFCC10_NUM_MELS) + dct_mel_idx];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_LOAD;
            load_frame_idx    <= {`MFCC10_FRAME_IDX_W{1'b0}};
            dct_frame_idx     <= {`MFCC10_FRAME_IDX_W{1'b0}};
            dct_cep_idx       <= {`MFCC10_MFCC_IDX_W{1'b0}};
            dct_mel_idx       <= {`MFCC10_MEL_IDX_W{1'b0}};
            max_db_q16        <= 32'sh80000000;
            min_db_q16        <= {`MFCC10_DB_W{1'b0}};
            dct_db_r          <= {`MFCC10_DB_W{1'b0}};
            dct_coeff_r       <= {`MFCC10_DCT_W{1'b0}};
            dct_prod_r        <= {(`MFCC10_DB_W+`MFCC10_DCT_W){1'b0}};
            dct_accum         <= 64'sd0;
            dct_out_r         <= {`MFCC10_OUT_W{1'b0}};
            coeff_valid       <= 1'b0;
            coeff_frame_index <= {`MFCC10_FRAME_IDX_W{1'b0}};
            coeff_index       <= {`MFCC10_MFCC_IDX_W{1'b0}};
            coeff_data        <= {`MFCC10_OUT_W{1'b0}};
            block_done        <= 1'b0;
            log_count         <= 16'd0;
            load_s0_valid     <= 1'b0;
            load_s0_frame_idx <= {`MFCC10_FRAME_IDX_W{1'b0}};
            load_s0_mel_idx   <= {`MFCC10_MEL_IDX_W{1'b0}};
            load_s0_log_input <= {`MFCC10_DB_W{1'b0}};
            load_s1_valid     <= 1'b0;
            load_s1_frame_idx <= {`MFCC10_FRAME_IDX_W{1'b0}};
            load_s1_mel_idx   <= {`MFCC10_MEL_IDX_W{1'b0}};
            load_s1_db_prod   <= 64'sd0;
            load_s2_valid     <= 1'b0;
            load_s2_frame_idx <= {`MFCC10_FRAME_IDX_W{1'b0}};
            load_s2_mel_idx   <= {`MFCC10_MEL_IDX_W{1'b0}};
            load_s2_db_q16    <= {`MFCC10_DB_W{1'b0}};
        end else begin
            block_done <= 1'b0;

            load_s0_valid <= log_valid && (state == S_LOAD);
            if (log_valid && (state == S_LOAD)) begin
                load_s0_frame_idx <= log_frame_index;
                load_s0_mel_idx   <= log_index;
                load_s0_log_input <= $signed({1'b0, log_q16}) - $signed(`MFCC10_SAMPLE_LOG2_Q16);
            end

            load_s1_valid     <= load_s0_valid;
            load_s1_frame_idx <= load_s0_frame_idx;
            load_s1_mel_idx   <= load_s0_mel_idx;
            if (load_s0_valid) begin
                load_s1_db_prod <= load_s0_log_input * $signed(`MFCC10_DB_COEFF_Q16);
            end

            load_s2_valid     <= load_s1_valid;
            load_s2_frame_idx <= load_s1_frame_idx;
            load_s2_mel_idx   <= load_s1_mel_idx;
            if (load_s1_valid) begin
                load_s2_db_q16 <= load_s1_db_prod >>> `MFCC10_LOG_FRAC_W;
            end

            if (load_s2_valid && (state == S_LOAD)) begin
                max_db_q16 <= max_db_next_wire;
                log_count <= log_count_next;
                if (log_count_next == (`MFCC10_FRAME_COUNT * `MFCC10_NUM_MELS)) begin
                    state         <= S_PREP_DCT;
                    dct_frame_idx <= {`MFCC10_FRAME_IDX_W{1'b0}};
                    dct_cep_idx   <= {`MFCC10_MFCC_IDX_W{1'b0}};
                    dct_mel_idx   <= {`MFCC10_MEL_IDX_W{1'b0}};
                    dct_accum     <= 64'sd0;
                    min_db_q16    <= max_db_next_wire - $signed(`MFCC10_TOP_DB_Q16);
                end
            end

            if ((state == S_LOAD) && mel_frame_done) begin
                if (load_frame_idx != (`MFCC10_FRAME_COUNT - 1)) begin
                    load_frame_idx <= load_frame_idx + 1'b1;
                end
            end

            case (state)
                S_LOAD: begin
                    coeff_valid <= 1'b0;
                end

                S_PREP_DCT: begin
                    dct_mel_idx <= {`MFCC10_MEL_IDX_W{1'b0}};
                    dct_accum   <= 64'sd0;
                    state       <= S_DCT_FETCH;
                end

                S_DCT_FETCH: begin
                    dct_coeff_r  <= dct_coeff_q17;
                    state        <= S_DCT_CLAMP;
                end

                S_DCT_CLAMP: begin
                    dct_db_r <= db_read_clamped_wire;
                    state    <= S_DCT_MUL;
                end

                S_DCT_MUL: begin
                    dct_prod_r  <= dct_db_r * dct_coeff_r;
                    state       <= S_DCT_ACC;
                end

                S_DCT_ACC: begin
                    if (dct_mel_idx == (`MFCC10_NUM_MELS - 1)) begin
                        dct_out_r <= sat_out((dct_accum + dct_prod_ext + (64'sd1 << (`MFCC10_DCT_FRAC_W - 1))) >>> `MFCC10_DCT_FRAC_W);
                        state     <= S_SEND;
                    end else begin
                        dct_accum   <= dct_accum + dct_prod_ext;
                        dct_mel_idx <= dct_mel_idx + 1'b1;
                        state       <= S_DCT_FETCH;
                    end
                end

                S_SEND: begin
                    coeff_valid       <= 1'b1;
                    coeff_frame_index <= dct_frame_idx;
                    coeff_index       <= dct_cep_idx;
                    coeff_data        <= dct_out_r;

                    if (send_fire) begin
                        coeff_valid <= 1'b0;
                        if ((dct_frame_idx == (`MFCC10_FRAME_COUNT - 1)) &&
                            (dct_cep_idx == (`MFCC10_NUM_MFCC - 1))) begin
                            block_done     <= 1'b1;
                            load_frame_idx <= {`MFCC10_FRAME_IDX_W{1'b0}};
                            log_count      <= 16'd0;
                            max_db_q16     <= 32'sh80000000;
                            min_db_q16     <= {`MFCC10_DB_W{1'b0}};
                            state          <= S_LOAD;
                        end else if (dct_cep_idx == (`MFCC10_NUM_MFCC - 1)) begin
                            dct_frame_idx <= dct_frame_idx + 1'b1;
                            dct_cep_idx   <= {`MFCC10_MFCC_IDX_W{1'b0}};
                            state         <= S_PREP_DCT;
                        end else begin
                            dct_cep_idx <= dct_cep_idx + 1'b1;
                            state       <= S_PREP_DCT;
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

