`timescale 1ns/1ps

// FC + argmax 单模块 testbench：
// 1. 从相对路径 data/fc_input.memh 读取 64 个 GAP 理论输出；
// 2. 顺序送入 FC 模块；
// 3. 用 display 打印最终类别和分数。
module tb_fc_argmax_stream;

    localparam integer DATA_W      = 8;
    localparam integer IN_CH       = 64;
    localparam integer OUT_CLASS   = 12;
    localparam integer ACC_W       = 40;
    localparam [1023:0] INPUT_FILE = "../../../data/avgpool.memh";

    reg               clk;
    reg               rst_n;
    reg               in_valid;
    wire              in_ready;
    reg  [DATA_W-1:0] in_data;
    wire              out_valid;
    wire [7:0]        out_class;
    wire [ACC_W-1:0]  out_score;

    reg  [DATA_W-1:0] fc_input_mem [0:IN_CH-1];
    integer           in_idx;

    function integer tc40_to_int;
        input [ACC_W-1:0] value;
        reg   [ACC_W-1:0] mag;
        begin
            if (value[ACC_W-1] == 1'b1) begin
                mag = (~value) + {{(ACC_W-1){1'b0}}, 1'b1};
                tc40_to_int = -mag[30:0];
            end else begin
                tc40_to_int = value[30:0];
            end
        end
    endfunction

    task send_input;
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

    fc_argmax_stream #(
        .DATA_W(DATA_W),
        .IN_CH(IN_CH),
        .OUT_CLASS(OUT_CLASS),
        .ACC_W(ACC_W),
        .WEIGHT_INIT_FILE("../data/DS-CNN_fc.memh"),
        .BIAS_INIT_FILE("../data/DS-CNN_fc_bias.memh")
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_data(in_data),
        .out_valid(out_valid),
        .out_class(out_class),
        .out_score(out_score)
    );

    always #5 clk = ~clk;

    initial begin
        $readmemh(INPUT_FILE, fc_input_mem);
        $display("[FC_TB] load input file: %0s", INPUT_FILE);
        $display("[FC_TB] total inputs=%0d", IN_CH);

        clk      = 1'b0;
        rst_n    = 1'b0;
        in_valid = 1'b0;
        in_data  = {DATA_W{1'b0}};

        repeat (4) @(posedge clk);
        rst_n = 1'b1;

        for (in_idx = 0; in_idx < IN_CH; in_idx = in_idx + 1) begin
            send_input(fc_input_mem[in_idx]);
        end

        wait (out_valid == 1'b1);
        $display("[FC_OUT] time=%0t class=%0d score_hex=%010h score_signed=%0d",
                 $time, out_class, out_score, tc40_to_int(out_score));

        repeat (10) @(posedge clk);
        $finish;
    end

endmodule
