`include "mfcc_defs.vh"

`timescale 1ns / 1ps

// 单帧功能冒烟测试。
// 同时打印送入 DCT 的 ln 输出和最终 MFCC 输出。
module tb_mfcc_gap_top_oneframe;

    reg         clk;
    reg         rst_n;
    reg         sample_valid;
    reg         sample_first;
    reg  [15:0] sample_data;
    reg         coeff_ready;

    wire                           busy;
    wire                           coeff_valid;
    wire [`MFCC_CEP_IDX_W-1:0]     coeff_index;
    wire [`MFCC_DCT_OUT_W-1:0]     coeff_data;
    wire                           frame_done;

    reg [15:0] sample_mem [0:`MFCC_FRAME_LEN-1];
    integer i;

    function integer tc_out_to_int;
        input [`MFCC_DCT_OUT_W-1:0] value;
        begin
            if (value[`MFCC_DCT_OUT_W-1]) begin
                tc_out_to_int = value - (1 << `MFCC_DCT_OUT_W);
            end else begin
                tc_out_to_int = value;
            end
        end
    endfunction

    function integer tc16_to_int;
        input [15:0] value;
        begin
            if (value[15]) begin
                tc16_to_int = value - (1 << 16);
            end else begin
                tc16_to_int = value;
            end
        end
    endfunction

    mfcc_gap_top dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_valid(sample_valid),
        .sample_first(sample_first),
        .sample_data (sample_data),
        .coeff_ready (coeff_ready),
        .busy        (busy),
        .coeff_valid (coeff_valid),
        .coeff_index (coeff_index),
        .coeff_data  (coeff_data),
        .frame_done  (frame_done)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    initial begin
        $readmemh("gap_oneframe_input_hex.mem", sample_mem);
    end

    initial begin
        rst_n        = 1'b0;
        sample_valid = 1'b0;
        sample_first = 1'b0;
        sample_data  = 16'd0;
        coeff_ready  = 1'b1;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // 精确送入一帧输入，让 DUT 输出一整组 MFCC。
        for (i = 0; i < `MFCC_FRAME_LEN; i = i + 1) begin
            @(posedge clk);
            sample_valid <= 1'b1;
            sample_first <= (i == 0);
            sample_data  <= sample_mem[i];
        end

        @(posedge clk);
        sample_valid <= 1'b0;
        sample_first <= 1'b0;
        sample_data  <= 16'd0;

        wait (frame_done == 1'b1);
        repeat (40) @(posedge clk);
        $finish;
    end

    // 加一个很小的 #1 延时，让打印值对应上升沿后的寄存器状态。
    always @(posedge clk) begin
        #1;
        if (coeff_valid && coeff_ready) begin
            $display("mfcc_gap_hw[%0d] = %0d", coeff_index, tc_out_to_int(coeff_data));
        end
        if (dut.dct_load_valid) begin
            $display("ln_gap_hw[%0d] = %0d", dut.dct_load_index, tc16_to_int(dut.dct_load_data));
        end
    end

endmodule
