`timescale 1ns/1ns

//==============================================================================
// Testbench: tb_network_top
// 参考 tb_dw_pw_pingpong 的发送风格，对 `network_top` 顶层进行输入驱动与输出基本检查。
// - 读取原始像素文件 data/pixel_input/input_pixels.memh
// - 使用组合驱动发送（conv-style），在握手时推进发送索引
//==============================================================================
module tb_network_top;

    // parameters (match network_top defaults)
    localparam integer DS_DATA_W       = 8;
    localparam integer DS_COL          = 10;
    localparam integer DS_ROW          = 49;
    localparam integer PIXELS_PER_FRAME = DS_COL * DS_ROW; // 490

    localparam INPUT_INIT_FILE = "D:/vivado/exp/DSCNN/data/pixel_input/input_pixels.memh";

    // DUT I/O
    reg                             clk;
    reg                             rst_n;
    reg                             start;

    reg                             in_valid;
    wire                            in_ready;
    reg signed [DS_DATA_W-1:0]      in_pixel;
    reg                             in_end_all_frame;

    wire                            out_valid;
    reg                             out_ready;
    wire [7:0]                      out_data; // default network_top OUT width is 8
    wire                            out_end_frame;
    wire                            out_end_all_frame;
    wire                            busy;

    // input memory
    localparam integer RUNS = 64*2;
    localparam integer TOTAL_IN_BEATS = PIXELS_PER_FRAME * RUNS; // 490
    reg [DS_DATA_W-1:0] input_mem [0:PIXELS_PER_FRAME-1];

    integer send_idx;
    reg send_done;
    integer out_count;
    integer err_cnt;
    integer i;

    // instantiate DUT
    network_top #(
        .DS_DATA_W(DS_DATA_W),
        .DS_COL(DS_COL),
        .DS_ROW(DS_ROW)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .busy(busy),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .out_valid(out_valid),
        .out_ready(out_ready),
        .out_data(out_data),
        .out_end_frame(out_end_frame),
        .out_end_all_frame(out_end_all_frame)
    );

    always #5 clk = ~clk;

    task pulse_start;
    begin
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;
    end
    endtask
    integer fd;
    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        in_valid = 1'b0;
        in_pixel = {DS_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;
        out_ready = 1'b1;

        err_cnt = 0;
        out_count = 0;
        send_idx = 0;
        send_done = 1'b0;


        // load input data
        $readmemh(INPUT_INIT_FILE, input_mem, 0, PIXELS_PER_FRAME-1);
        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // start network
        pulse_start();
        repeat (2) @(posedge clk);
        wait (busy == 1'b0);
        pulse_start();

        while (!send_done) @(posedge clk);

        // wait for DUT to finish processing
        wait (busy == 1'b0);
        // allow outputs to drain
        repeat (10) @(posedge clk);

        $display("[TB] finished: out_count=%0d errors=%0d", out_count, err_cnt);
        #20; $stop;
    end

    // combinational sender (conv-style)
    always @(*) begin
        in_valid = 1'b0;
        in_pixel = {DS_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;

        if (!send_done && (send_idx < TOTAL_IN_BEATS)) begin
            in_valid = 1'b1;
            in_pixel = input_mem[send_idx % PIXELS_PER_FRAME];
            in_end_all_frame = ((send_idx % PIXELS_PER_FRAME) == (PIXELS_PER_FRAME - 1)) ? 1'b1 : 1'b0;
        end
    end

    // advance send index on handshake
    always @(posedge clk) begin
        if (!rst_n) begin
            send_idx <= 0;
            send_done <= 1'b0;
        end else begin
            if (!send_done && in_valid && in_ready) begin
                send_idx <= send_idx + 1;
                if (send_idx + 1 >= TOTAL_IN_BEATS) send_done <= 1'b1;
            end
        end
    end

    // simple output monitor
    always @(posedge clk) begin
        if (rst_n && out_valid && out_ready) begin
            out_count <= out_count + 1;
            // basic display (can be commented)
            //$display("[TB] out[%0d]=%h ef=%b eaf=%b", out_count, out_data, out_end_frame, out_end_all_frame);
        end
    end

    reg[31:0] out_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_cnt <= 32'b0;
        end else begin
            if(out_valid && out_ready) begin
                $display("beat %d: PW_out_pixel(Hex): %h ,end_frame: %b,end_all_frame: %b", out_cnt, out_data, out_end_frame, out_end_all_frame);
                out_cnt <= out_cnt + 1;
            end
        end
    end

endmodule
