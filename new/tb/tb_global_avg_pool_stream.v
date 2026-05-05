`timescale 1ns/1ps

// GAP 单模块 testbench：
// 1. 从相对路径 data/gap_input.memh 读取 25*5*64 个输入；
// 2. 按通道优先顺序喂给 DUT；
// 3. 用 display 打印每个通道的输出，便于人工观察。
module tb_global_avg_pool_stream;

    localparam integer DATA_W        = 8;
    localparam integer ROW_LEN       = 25;
    localparam integer ROW_NUM       = 5;
    localparam integer CHANNELS      = 64;
    localparam integer SUM_W         = 16;
    localparam integer PIXELS_PER_CH = ROW_LEN * ROW_NUM;
    localparam integer TOTAL_SAMPLES = CHANNELS * PIXELS_PER_CH;
    localparam [1023:0] INPUT_FILE   = "../../../data/layer4_pw.memh";

    reg               clk;
    reg               rst_n;
    reg               in_valid;
    wire              in_ready;
    reg  [DATA_W-1:0] in_data;
    wire              out_valid;
    reg               out_ready;
    wire [DATA_W-1:0] out_data;
    wire              out_channel_last;
    wire              out_frame_last;

    reg  [DATA_W-1:0] gap_input_mem [0:TOTAL_SAMPLES-1];

    integer           ch_idx;
    integer           pix_idx;
    integer           mem_idx;
    integer           output_count;
    integer           output_channel;

    function integer tc8_to_int;
        input [DATA_W-1:0] value;
        reg   [DATA_W-1:0] mag;
        begin
            if (value[DATA_W-1] == 1'b1) begin
                mag = (~value) + {{(DATA_W-1){1'b0}}, 1'b1};
                tc8_to_int = -mag;
            end else begin
                tc8_to_int = value;
            end
        end
    endfunction

    task send_sample;
        input [DATA_W-1:0] sample_data;
        begin
            @(posedge clk);
            while (in_ready == 1'b0) begin
                @(posedge clk);
            end
            in_valid <= 1'b1;
            in_data  <= sample_data;
            @(posedge clk);
            in_valid <= 1'b0;
            in_data  <= {DATA_W{1'b0}};
        end
    endtask

    global_avg_pool_stream #(
        .DATA_W(DATA_W),
        .ROW_LEN(ROW_LEN),
        .ROW_NUM(ROW_NUM),
        .CHANNELS(CHANNELS),
        .SUM_W(SUM_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_data(in_data),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .out_channel_last(out_channel_last),
        .out_frame_last(out_frame_last)
    );

    always #5 clk = ~clk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            output_count   <= 0;
            output_channel <= 0;
        end else if (out_valid && out_ready) begin
            $display("[GAP_OUT] idx=%0d ch=%0d time=%0t data_hex=%02h data_signed=%0d frame_last=%0d",
                     output_count,
                     output_channel,
                     $time,
                     out_data,
                     tc8_to_int(out_data),
                     out_frame_last);
            output_count <= output_count + 1;
            if (out_frame_last) begin
                output_channel <= 0;
            end else begin
                output_channel <= output_channel + 1;
            end
        end
    end

    initial begin
        $readmemh(INPUT_FILE, gap_input_mem);
        $display("[GAP_TB] load input file: %0s", INPUT_FILE);
        $display("[GAP_TB] total samples=%0d", TOTAL_SAMPLES);

        clk       = 1'b0;
        rst_n     = 1'b0;
        in_valid  = 1'b0;
        in_data   = {DATA_W{1'b0}};
        out_ready = 1'b1;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        for (ch_idx = 0; ch_idx < CHANNELS; ch_idx = ch_idx + 1) begin
            for (pix_idx = 0; pix_idx < PIXELS_PER_CH; pix_idx = pix_idx + 1) begin
                mem_idx = ch_idx * PIXELS_PER_CH + pix_idx;
                send_sample(gap_input_mem[mem_idx]);
            end
        end

        wait (output_count == CHANNELS);
        repeat (20) @(posedge clk);
        $finish;
    end

endmodule
