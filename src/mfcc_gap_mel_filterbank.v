`include "mfcc_defs.vh"

module mfcc_gap_mel_filterbank (
    input  wire                           clk,
    input  wire                           rst_n,
    input  wire                           load_valid,
    input  wire [`MFCC_POWER_ADDR_W-1:0]  load_index,
    input  wire [`MFCC_FFT_W-1:0]         load_re,
    input  wire [`MFCC_FFT_W-1:0]         load_im,
    input  wire                           start,
    output reg                            busy,
    output reg                            done,
    input  wire [`MFCC_FILTER_IDX_W-1:0]  rd_index,
    output wire [31:0]                    rd_data
);

    localparam PHASE_FETCH = 3'd0;
    localparam PHASE_LOAD  = 3'd1;
    localparam PHASE_MUL   = 3'd2;
    localparam PHASE_ACC   = 3'd3;
    localparam PHASE_STORE = 3'd4;
    localparam SQRT_STAGES = 32;

    reg [31:0] mel_mem [0:`MFCC_NUM_FILTERS-1];

    reg                           power_wr_en;
    reg [`MFCC_POWER_ADDR_W-1:0]  power_wr_addr;
    reg [31:0]                    power_wr_data;
    reg                           power_rd_en;
    reg [`MFCC_POWER_ADDR_W-1:0]  power_rd_addr;
    wire [31:0]                   power_rd_data;

    reg [2:0]                     phase;
    reg [`MFCC_FILTER_IDX_W-1:0]  filter_idx;
    reg [`MFCC_POWER_ADDR_W-1:0]  item_idx;
    reg [47:0]                    accum;
    reg [31:0]                    curr_mag_r;
    reg [15:0]                    weight_q15_r;
    reg [47:0]                    contrib_r;
    reg [47:0]                    final_sum_r;

    reg                           load_s0_valid;
    reg [`MFCC_POWER_ADDR_W-1:0]  load_s0_addr;
    reg [`MFCC_FFT_W-1:0]         load_s0_re;
    reg [`MFCC_FFT_W-1:0]         load_s0_im;

    reg                           load_s1_valid;
    reg [`MFCC_POWER_ADDR_W-1:0]  load_s1_addr;
    reg [55:0]                    load_s1_re_sq;
    reg [55:0]                    load_s1_im_sq;

    reg                           load_s2_valid;
    reg [`MFCC_POWER_ADDR_W-1:0]  load_s2_addr;
    reg [63:0]                    load_s2_mag_sq;

    reg                           start_pending;
    reg [`MFCC_POWER_ADDR_W:0]    load_pending_count;

    reg [63:0]                    sqrt_value_pipe [0:SQRT_STAGES];
    reg [63:0]                    sqrt_rem_pipe   [0:SQRT_STAGES];
    reg [31:0]                    sqrt_root_pipe  [0:SQRT_STAGES];
    reg                           sqrt_valid_pipe [0:SQRT_STAGES];
    reg [`MFCC_POWER_ADDR_W-1:0]  sqrt_addr_pipe  [0:SQRT_STAGES];

    wire [`MFCC_POWER_ADDR_W-1:0] start_bin;
    wire [`MFCC_POWER_ADDR_W-1:0] items;
    wire [15:0]                   weight_q15;
    wire [55:0]                   re_sq_wire;
    wire [55:0]                   im_sq_wire;

    wire                          load_in_fire;
    wire                          power_wr_fire;
    wire                          load_pipe_busy;

    integer i;

    function [31:0] sat_u32;
        input [63:0] value;
        begin
            if (|value[63:32]) begin
                sat_u32 = 32'hFFFF_FFFF;
            end else begin
                sat_u32 = value[31:0];
            end
        end
    endfunction

    function [159:0] sqrt_step;
        input [63:0] value;
        input [63:0] rem;
        input [31:0] root;
        reg   [1:0]  pair;
        reg   [63:0] value_next;
        reg   [63:0] rem_shift;
        reg   [63:0] rem_next;
        reg   [31:0] root_shift;
        reg   [31:0] root_next;
        reg   [32:0] trial;
        reg   [63:0] trial_ext;
        begin
            pair       = value[63:62];
            value_next = {value[61:0], 2'b0};
            rem_shift  = {rem[61:0], pair};
            root_shift = {root[30:0], 1'b0};
            trial      = ({1'b0, root_shift} << 1) | 33'd1;
            trial_ext  = {{31{1'b0}}, trial};

            if (rem_shift >= trial_ext) begin
                rem_next  = rem_shift - trial_ext;
                root_next = root_shift | 32'd1;
            end else begin
                rem_next  = rem_shift;
                root_next = root_shift;
            end

            sqrt_step = {value_next, rem_next, root_next};
        end
    endfunction

    assign rd_data      = mel_mem[rd_index];
    assign load_in_fire = load_valid && (load_index < `MFCC_POWER_BINS);
    assign power_wr_fire = sqrt_valid_pipe[SQRT_STAGES];
    assign load_pipe_busy = (load_pending_count != {(`MFCC_POWER_ADDR_W+1){1'b0}}) || load_in_fire;

    mfcc_gap_mel_weight_rom u_weight_rom (
        .filter_idx(filter_idx),
        .item_idx  (item_idx),
        .start_bin (start_bin),
        .items     (items),
        .weight_q15(weight_q15)
    );

    mfcc_ram_sdp #(
        .DATA_W(32),
        .ADDR_W(`MFCC_POWER_ADDR_W),
        .DEPTH (`MFCC_POWER_BINS)
    ) u_mag_ram (
        .clk    (clk),
        .rst_n  (rst_n),
        .wr_en  (power_wr_en),
        .wr_addr(power_wr_addr),
        .wr_data(power_wr_data),
        .rd_en  (power_rd_en),
        .rd_addr(power_rd_addr),
        .rd_data(power_rd_data)
    );

    mfcc_tc_mul #(
        .A_W(`MFCC_FFT_W),
        .B_W(`MFCC_FFT_W),
        .P_W(56)
    ) u_re_sq (
        .a_tc(load_s0_re),
        .b_tc(load_s0_re),
        .p_tc(re_sq_wire)
    );

    mfcc_tc_mul #(
        .A_W(`MFCC_FFT_W),
        .B_W(`MFCC_FFT_W),
        .P_W(56)
    ) u_im_sq (
        .a_tc(load_s0_im),
        .b_tc(load_s0_im),
        .p_tc(im_sq_wire)
    );

    always @(*) begin
        power_wr_en   = power_wr_fire;
        power_wr_addr = sqrt_addr_pipe[SQRT_STAGES];
        power_wr_data = sqrt_root_pipe[SQRT_STAGES];
        power_rd_en   = 1'b0;
        power_rd_addr = {`MFCC_POWER_ADDR_W{1'b0}};

        if (busy && (phase == PHASE_FETCH) && (items != {`MFCC_POWER_ADDR_W{1'b0}})) begin
            power_rd_en   = 1'b1;
            power_rd_addr = start_bin + item_idx;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy               <= 1'b0;
            done               <= 1'b0;
            phase              <= PHASE_FETCH;
            filter_idx         <= {`MFCC_FILTER_IDX_W{1'b0}};
            item_idx           <= {`MFCC_POWER_ADDR_W{1'b0}};
            accum              <= 48'd0;
            curr_mag_r         <= 32'd0;
            weight_q15_r       <= 16'd0;
            contrib_r          <= 48'd0;
            final_sum_r        <= 48'd0;
            load_s0_valid      <= 1'b0;
            load_s0_addr       <= {`MFCC_POWER_ADDR_W{1'b0}};
            load_s0_re         <= {`MFCC_FFT_W{1'b0}};
            load_s0_im         <= {`MFCC_FFT_W{1'b0}};
            load_s1_valid      <= 1'b0;
            load_s1_addr       <= {`MFCC_POWER_ADDR_W{1'b0}};
            load_s1_re_sq      <= 56'd0;
            load_s1_im_sq      <= 56'd0;
            load_s2_valid      <= 1'b0;
            load_s2_addr       <= {`MFCC_POWER_ADDR_W{1'b0}};
            load_s2_mag_sq     <= 64'd0;
            start_pending      <= 1'b0;
            load_pending_count <= {(`MFCC_POWER_ADDR_W+1){1'b0}};

            for (i = 0; i <= SQRT_STAGES; i = i + 1) begin
                sqrt_value_pipe[i] <= 64'd0;
                sqrt_rem_pipe[i]   <= 64'd0;
                sqrt_root_pipe[i]  <= 32'd0;
                sqrt_valid_pipe[i] <= 1'b0;
                sqrt_addr_pipe[i]  <= {`MFCC_POWER_ADDR_W{1'b0}};
            end
        end else begin
            done <= 1'b0;

            load_s0_valid <= load_in_fire;
            if (load_in_fire) begin
                load_s0_addr <= load_index;
                load_s0_re   <= load_re;
                load_s0_im   <= load_im;
            end

            load_s1_valid <= load_s0_valid;
            if (load_s0_valid) begin
                load_s1_addr  <= load_s0_addr;
                load_s1_re_sq <= re_sq_wire;
                load_s1_im_sq <= im_sq_wire;
            end

            load_s2_valid <= load_s1_valid;
            if (load_s1_valid) begin
                load_s2_addr   <= load_s1_addr;
                load_s2_mag_sq <= {8'd0, load_s1_re_sq} + {8'd0, load_s1_im_sq};
            end

            sqrt_valid_pipe[0] <= load_s2_valid;
            sqrt_addr_pipe[0]  <= load_s2_addr;
            sqrt_value_pipe[0] <= load_s2_mag_sq;
            sqrt_rem_pipe[0]   <= 64'd0;
            sqrt_root_pipe[0]  <= 32'd0;

            for (i = 0; i < SQRT_STAGES; i = i + 1) begin
                sqrt_valid_pipe[i+1] <= sqrt_valid_pipe[i];
                sqrt_addr_pipe[i+1]  <= sqrt_addr_pipe[i];
                {sqrt_value_pipe[i+1], sqrt_rem_pipe[i+1], sqrt_root_pipe[i+1]} <=
                    sqrt_step(sqrt_value_pipe[i], sqrt_rem_pipe[i], sqrt_root_pipe[i]);
            end

            case ({load_in_fire, power_wr_fire})
                2'b10: load_pending_count <= load_pending_count + 1'b1;
                2'b01: load_pending_count <= load_pending_count - 1'b1;
                default: load_pending_count <= load_pending_count;
            endcase

            if (start && !busy) begin
                if (load_pipe_busy) begin
                    start_pending <= 1'b1;
                end else begin
                    start_pending <= 1'b0;
                    busy          <= 1'b1;
                    phase         <= PHASE_FETCH;
                    filter_idx    <= {`MFCC_FILTER_IDX_W{1'b0}};
                    item_idx      <= {`MFCC_POWER_ADDR_W{1'b0}};
                    accum         <= 48'd0;
                end
            end else if (start_pending && !busy &&
                         (load_pending_count == {(`MFCC_POWER_ADDR_W+1){1'b0}}) &&
                         !load_in_fire) begin
                start_pending <= 1'b0;
                busy          <= 1'b1;
                phase         <= PHASE_FETCH;
                filter_idx    <= {`MFCC_FILTER_IDX_W{1'b0}};
                item_idx      <= {`MFCC_POWER_ADDR_W{1'b0}};
                accum         <= 48'd0;
            end else if (busy) begin
                case (phase)
                    PHASE_FETCH: begin
                        if (items == {`MFCC_POWER_ADDR_W{1'b0}}) begin
                            final_sum_r <= 48'd0;
                            phase       <= PHASE_STORE;
                        end else begin
                            phase <= PHASE_LOAD;
                        end
                    end

                    PHASE_LOAD: begin
                        curr_mag_r   <= power_rd_data;
                        weight_q15_r <= weight_q15;
                        phase        <= PHASE_MUL;
                    end

                    PHASE_MUL: begin
                        contrib_r <= (curr_mag_r * weight_q15_r) >> 15;
                        phase     <= PHASE_ACC;
                    end

                    PHASE_ACC: begin
                        if (item_idx == (items - 1'b1)) begin
                            final_sum_r <= accum + contrib_r;
                            phase       <= PHASE_STORE;
                        end else begin
                            accum    <= accum + contrib_r;
                            item_idx <= item_idx + 1'b1;
                            phase    <= PHASE_FETCH;
                        end
                    end

                    PHASE_STORE: begin
                        mel_mem[filter_idx] <= sat_u32(final_sum_r);
                        accum               <= 48'd0;
                        item_idx            <= {`MFCC_POWER_ADDR_W{1'b0}};

                        if (filter_idx == (`MFCC_NUM_FILTERS - 1)) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            phase <= PHASE_FETCH;
                        end else begin
                            filter_idx <= filter_idx + 1'b1;
                            phase      <= PHASE_FETCH;
                        end
                    end

                    default: begin
                        phase <= PHASE_FETCH;
                    end
                endcase
            end
        end
    end

endmodule
