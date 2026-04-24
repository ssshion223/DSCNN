`timescale 1ns / 1ps

module tb_ram_frame_to_fifo_reader;

    // ------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------
    localparam integer PIXELS_PER_FRAME = 2;
    localparam integer SEGMENTS         = 1;
    localparam integer CHANNELS         = 64;
    localparam integer SEG_CHANS        = CHANNELS / SEGMENTS;
    localparam integer DATA_WIDTH       = 22 * SEG_CHANS;
    localparam integer OUT_W            = DATA_WIDTH / SEG_CHANS;
    localparam integer ADDR_WIDTH       = $clog2(PIXELS_PER_FRAME * SEGMENTS);
    localparam integer FIFO_DEPTH       = 8;
    localparam integer FIFO_AF_LEVEL    = 6;
    localparam integer TOTAL_WORDS      = PIXELS_PER_FRAME * SEGMENTS;
    localparam integer TOTAL_OUTS       = PIXELS_PER_FRAME * SEGMENTS * SEG_CHANS;
    localparam integer WAIT_MAX_CYCLES  = 200;
    localparam integer ROUND_MAX_CYCLES = WAIT_MAX_CYCLES * 8;
    localparam integer OUTW_SLICES      = DATA_WIDTH / OUT_W;
    localparam integer OUTW_UNUSED_BITS = DATA_WIDTH - OUTW_SLICES * OUT_W;
    localparam integer READ_ROUNDS      = 3;

    // ------------------------------------------------------------
    // DUT I/O
    // ------------------------------------------------------------
    reg                         clk;
    reg                         rst_n;
    reg                         start;
    wire                        busy;
    wire                        ram_re;
    wire [ADDR_WIDTH-1:0]       ram_raddr;
    wire [DATA_WIDTH-1:0]       ram_rdata;
    wire                        out_valid;
    reg                         out_ready;
    wire [OUT_W-1:0]            out_data;
    wire                        end_frame;
    wire                        end_all_frame;
    wire                        fifo_full;
    wire                        fifo_almost_full;
    wire                        fifo_empty;

    reg [DATA_WIDTH-1:0] ram_mem [0:TOTAL_WORDS-1];

    // ------------------------------------------------------------
    // Scoreboard
    // ------------------------------------------------------------
    integer word_idx;
    integer pixel_idx;
    integer seg_idx;
    integer lane_idx;
    integer errors;
    integer out_count;
    integer cyc;
    integer wait_guard;
    integer init_i;
    integer round_i;
    reg [OUT_W-1:0] expected_word;

    psum_ram #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DEPTH(TOTAL_WORDS)
    ) u_ram (
        .clk(clk),
        .re(ram_re),
        .raddr(ram_raddr),
        .rdata(ram_rdata),
        .we(1'b0),
        .waddr({ADDR_WIDTH{1'b0}}),
        .wdata({DATA_WIDTH{1'b0}})
    );

    ram_frame_to_fifo_reader #(
        .PIXELS_PER_FRAME(PIXELS_PER_FRAME),
        .SEGMENTS(SEGMENTS),
        .CHANNELS(CHANNELS),
        .RAM_DATA_W(DATA_WIDTH),
        .RAM_ADDR_W(ADDR_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_AF_LEVEL(FIFO_AF_LEVEL),
        .OUT_W(OUT_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .busy(busy),
        .ram_re(ram_re),
        .ram_raddr(ram_raddr),
        .ram_rdata(ram_rdata),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .end_frame(end_frame),
        .end_all_frame(end_all_frame),
        .fifo_full(fifo_full),
        .fifo_almost_full(fifo_almost_full),
        .fifo_empty(fifo_empty)
    );

    always #5 clk = ~clk;

    function [DATA_WIDTH-1:0] build_word;
        input integer p_idx;
        input integer s_idx;
        integer lane;
        reg [DATA_WIDTH-1:0] word;
        reg [21:0] lane_value;
    begin
        word = {DATA_WIDTH{1'b0}};
        for (lane = 0; lane < SEG_CHANS; lane = lane + 1) begin
            lane_value = p_idx * 1000 + s_idx * 100 + lane;
            word[(lane + 1) * 22 - 1 -: 22] = lane_value;
        end
        build_word = word;
    end
    endfunction

    task init_ram;
        integer p;
        integer s;
    begin
        for (p = 0; p < PIXELS_PER_FRAME; p = p + 1) begin
            for (s = 0; s < SEGMENTS; s = s + 1) begin
                ram_mem[p * SEGMENTS + s] = build_word(p, s);
            end
        end
    end
    endtask

    task print_ram_init_data;
        integer addr;
        integer slice_idx;
        reg [OUT_W-1:0] slice_data;
        reg signed [OUT_W-1:0] slice_data_s;
    begin
        $display("[TB][RAM_INIT] -------- Dump Start --------");
        $display("[TB][RAM_INIT] split by OUT_W=%0d, slices_per_word=%0d, unused_msb_bits=%0d",
                 OUT_W, OUTW_SLICES, OUTW_UNUSED_BITS);
        for (addr = 0; addr < TOTAL_WORDS; addr = addr + 1) begin
            $display("[TB][RAM_INIT] addr=%0d data=%h", addr, ram_mem[addr]);
            for (slice_idx = 0; slice_idx < OUTW_SLICES; slice_idx = slice_idx + 1) begin
                slice_data = ram_mem[addr][(slice_idx + 1) * OUT_W - 1 -: OUT_W];
                slice_data_s = slice_data;
                $display("[TB][RAM_INIT] addr=%0d slice[%0d]=%h signed=%0d", addr, slice_idx, slice_data, slice_data_s);
            end
        end
        $display("[TB][RAM_INIT] -------- Dump End   --------");
    end
    endtask

    task pulse_start;
    begin
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
    end
    endtask

    task wait_for_idle;
    begin
        wait_guard = 0;
        while (busy) begin
            @(posedge clk);
            wait_guard = wait_guard + 1;
            if (wait_guard > WAIT_MAX_CYCLES) begin
                $display("[TB][FAIL] timeout waiting for busy deassert");
                errors = errors + 1;
                disable wait_for_idle;
            end
        end
    end
    endtask

    task check_output;
        input [OUT_W-1:0] got_word;
        input integer word_no;
        integer p;
        integer s;
        integer lane;
        reg expected_end_frame;
        reg expected_end_all_frame;
        reg [DATA_WIDTH-1:0] full_word;
        reg signed [OUT_W-1:0] expected_word_s;
        reg signed [OUT_W-1:0] got_word_s;
    begin
        p = word_no % PIXELS_PER_FRAME;
        lane = (word_no / PIXELS_PER_FRAME) % SEG_CHANS;
        s = word_no / (PIXELS_PER_FRAME * SEG_CHANS);
        full_word = build_word(p, s);
        expected_word = full_word[(lane + 1) * OUT_W - 1 -: OUT_W];
        expected_word_s = expected_word;
        got_word_s = got_word;
        expected_end_frame = ((word_no % PIXELS_PER_FRAME) == (PIXELS_PER_FRAME - 1));
        expected_end_all_frame = (word_no == (TOTAL_OUTS - 1));
        if (got_word !== expected_word) begin
            $display("[TB][FAIL] word=%0d pixel=%0d seg=%0d lane=%0d expected=%h(%0d) got=%h(%0d)",
                     word_no, p, s, lane, expected_word, expected_word_s, got_word, got_word_s);
            errors = errors + 1;
        end else begin
            $display("[TB][PASS] word=%0d pixel=%0d seg=%0d lane=%0d data=%h(%0d)",
                     word_no, p, s, lane, got_word, got_word_s);
        end
        if (end_frame !== expected_end_frame) begin
            $display("[TB][FAIL] word=%0d expected end_frame=%0b got=%0b", word_no, expected_end_frame, end_frame);
            errors = errors + 1;
        end
        if (end_all_frame !== expected_end_all_frame) begin
            $display("[TB][FAIL] word=%0d expected end_all_frame=%0b got=%0b", word_no, expected_end_all_frame, end_all_frame);
            errors = errors + 1;
        end
    end
    endtask

    task run_one_round;
        input integer round_id;
    begin
        out_count = 0;

        pulse_start();

        wait_guard = 0;
        while (!busy) begin
            @(posedge clk);
            wait_guard = wait_guard + 1;
            if (wait_guard > 20) begin
                $display("[TB][FAIL] round=%0d busy did not assert after start", round_id);
                errors = errors + 1;
                disable run_one_round;
            end
        end

        cyc = 0;
        while (((out_count < TOTAL_OUTS) || busy) && (cyc < ROUND_MAX_CYCLES)) begin
            @(negedge clk);
            if (((cyc + round_id) % 7) == 4) begin
                out_ready = 1'b0;
            end else begin
                out_ready = 1'b1;
            end
            cyc = cyc + 1;
        end

        if (cyc >= ROUND_MAX_CYCLES) begin
            $display("[TB][FAIL] round=%0d timeout waiting round done, got=%0d expected=%0d busy=%0b", round_id, out_count, TOTAL_OUTS, busy);
            errors = errors + 1;
        end else if (out_count != TOTAL_OUTS) begin
            $display("[TB][FAIL] round=%0d output beat count mismatch got=%0d expected=%0d", round_id, out_count, TOTAL_OUTS);
            errors = errors + 1;
        end

        wait_for_idle();

        out_ready = 1'b1;
        repeat (2) @(posedge clk);
    end
    endtask

    initial begin : TEST_MAIN
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        out_ready = 1'b1;
        errors = 0;
        out_count = 0;

        init_ram();
        for (init_i = 0; init_i < TOTAL_WORDS; init_i = init_i + 1) begin
            u_ram.ram[init_i] = ram_mem[init_i];
        end
        print_ram_init_data();

        $display("\n===============================================");
        $display("[TB] tb_ram_frame_to_fifo_reader start");
        $display("[TB] PIXELS_PER_FRAME=%0d SEGMENTS=%0d CHANNELS=%0d", PIXELS_PER_FRAME, SEGMENTS, CHANNELS);
        $display("[TB] DATA_WIDTH=%0d ADDR_WIDTH=%0d TOTAL_WORDS=%0d TOTAL_OUTS=%0d", DATA_WIDTH, ADDR_WIDTH, TOTAL_WORDS, TOTAL_OUTS);
        $display("[TB] end_frame expected on every %0d outputs, end_all_frame expected on last output", PIXELS_PER_FRAME);
        $display("[TB] READ_ROUNDS=%0d", READ_ROUNDS);
        $display("===============================================\n");

        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        for (round_i = 0; round_i < READ_ROUNDS; round_i = round_i + 1) begin
            $display("[TB] ---- round %0d/%0d ----", round_i + 1, READ_ROUNDS);
            run_one_round(round_i);
        end

        if (errors == 0) begin
            $display("[TB][PASS] tb_ram_frame_to_fifo_reader PASSED");
        end else begin
            $display("[TB][FAIL] tb_ram_frame_to_fifo_reader FAILED, errors=%0d", errors);
        end

        #30;
        $stop;
    end

    always @(posedge clk) begin
        if (rst_n && out_valid) begin
            if (out_ready) begin
                check_output(out_data, out_count);
                out_count = out_count + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            if (^end_frame === 1'bx || ^end_all_frame === 1'bx) begin
                $display("[TB][FAIL] frame flags became unknown");
                errors = errors + 1;
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && out_valid && !out_ready) begin
            if (^out_data === 1'bx) begin
                $display("[TB][FAIL] out_data became unknown during stall");
                errors = errors + 1;
            end
        end
    end

endmodule