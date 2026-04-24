`timescale 1ns/1ns

module tb_dw_pw_cac;
    localparam integer IN_DATA_W          = 8;
    localparam integer DW_COEFF_W         = 8;
    localparam integer DW_K_H             = 3;
    localparam integer DW_K_W             = 3;
    localparam integer DW_MUL_W           = IN_DATA_W + DW_COEFF_W;
    localparam integer DW_SUM_W           = DW_MUL_W + $clog2(DW_K_H*DW_K_W);
    localparam integer DW_COL             = 5;
    localparam integer DW_ROW             = 25;
    localparam integer DW_STRIDE          = 1;
    localparam integer DW_PAD_TOP         = (DW_K_H-1)/2;
    localparam integer DW_PAD_BOTTOM      = (DW_K_H)/2;
    localparam integer DW_PAD_LEFT        = (DW_K_W-1)/2;
    localparam integer DW_PAD_RIGHT       = (DW_K_W)/2;
    localparam integer DW_COEFF_GRP_NUM   = 64*4;
    localparam integer DW_FRAME_GRP_NUM   = 64;
    localparam integer DW_MAC_PIPELINE    = 1;
    localparam integer DW_OUT_WIDTH       = 8;
    localparam integer DW_SHIFT_VAL       = 16;
    localparam integer DW_BIAS_GROUP_SIZE = 4;
    localparam integer DW_BIAS_GROUP_BITS = $clog2(DW_BIAS_GROUP_SIZE);
    localparam integer DW_BIAS_CH_BITS    = 6;
    localparam integer DW_MULT_CNT           = 4;
    // 915,1246,406,828,461,652,442,412,623
    localparam signed [11:0] DW_MULT_FACTOR0 = 12'sd1246;
    localparam signed [11:0] DW_MULT_FACTOR1 = 12'sd828;
    localparam signed [11:0] DW_MULT_FACTOR2 = 12'sd652;
    localparam signed [11:0] DW_MULT_FACTOR3 = 12'sd412;
    localparam integer DW_FIFO_DEPTH      = 16;
    localparam integer DW_FIFO_AF_LEVEL   = 10;

    localparam integer PW_OUT_CH          = 64;
    localparam integer PW_COEFF_W         = 8;
    localparam integer PW_K_H             = 1;
    localparam integer PW_K_W             = 1;
    localparam integer PW_MUL_W           = DW_OUT_WIDTH + PW_COEFF_W;
    localparam integer PW_SUM_W           = PW_MUL_W + $clog2(PW_K_H*PW_K_W);
    localparam integer PW_COEFF_GRP_NUM   = 64*4;
    localparam integer PW_FRAME_GRP_NUM   = 64;
    localparam integer PW_MAC_PIPELINE    = 1;
    localparam integer PW_FIFO_DEPTH      = 4;
    localparam integer PW_FIFO_AF_LEVEL   = 2;

    localparam integer PW_BIAS_GROUP_SIZE = 4;
    localparam integer PW_BIAS_GROUP_BITS = $clog2(PW_BIAS_GROUP_SIZE);
    localparam integer PW_BIAS_CH_BITS    = 6;
    localparam integer PW_MULT_CNT        = 4;

    localparam signed [11:0] PW_MULT_FACTOR0 = 12'sd406;
    localparam signed [11:0] PW_MULT_FACTOR1 = 12'sd461;
    localparam signed [11:0] PW_MULT_FACTOR2 = 12'sd442;
    localparam signed [11:0] PW_MULT_FACTOR3 = 12'sd623;

    localparam integer RAM_DATA_W         = PW_SUM_W + $clog2(PW_OUT_CH);
    localparam integer SEGMENTS           = 4;
    localparam integer PIXEL_DEPTH        = ((DW_COL + DW_PAD_LEFT + DW_PAD_RIGHT - DW_K_W) / DW_STRIDE + 1)
                                           * ((DW_ROW + DW_PAD_TOP + DW_PAD_BOTTOM - DW_K_H) / DW_STRIDE + 1);
    localparam integer RAM_ADDR_WIDTH     = $clog2(PIXEL_DEPTH * SEGMENTS);
    localparam integer IN_FRAME_SIZE      = 64;

    localparam integer SEG_CHANS          = PW_OUT_CH / SEGMENTS;
    localparam integer RAM_DATA_WIDTH     = SEG_CHANS * RAM_DATA_W;
    localparam integer RAM_DEPTH          = PIXEL_DEPTH * SEGMENTS;

    localparam integer PIXELS_PER_FRAME     = DW_ROW * DW_COL;
    localparam integer ROUND            = 4; 
    localparam integer ROUND_IN_BEATS   = PIXELS_PER_FRAME * IN_FRAME_SIZE ;
    localparam integer TOTAL_IN_BEATS    = ROUND_IN_BEATS * ROUND;
    
    localparam integer WAIT_CYC_MAX       = 1200000;

    localparam INPUT_INIT_FILE      = "D:/vivado/exp/DSCNN/data/pixel_input/input.memh";
    localparam DW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh";
    localparam DW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw_pingpong_Fold_bias.hex";
    localparam PW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_pw.memh";
    localparam PW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_pw_pingpong_bias.hex";


    reg                               clk;
    reg                               rst_n;

    reg                               in_valid;
    wire                              in_ready;
    reg signed [IN_DATA_W-1:0]        in_pixel;
    reg                               in_end_all_frame;

    wire                              ram_re_pw;
    wire [RAM_ADDR_WIDTH-1:0]         ram_raddr_pw;
    wire [RAM_DATA_WIDTH-1:0]         ram_rdata;
    wire                              ram_we_pw;
    wire [RAM_ADDR_WIDTH-1:0]         ram_waddr_pw;
    wire [RAM_DATA_WIDTH-1:0]         ram_wdata_pw;

    wire                              pw_end_frame;
    wire                              pw_end_all_frame;

    reg                               reader_start;
    wire                              reader_busy;
    wire                              reader_ram_re;
    wire [RAM_ADDR_WIDTH-1:0]         reader_ram_raddr;
    wire [RAM_DATA_W-1:0]             reader_out_data;
    wire                              reader_out_valid;
    wire                              reader_out_ready;
    wire                              reader_end_frame;
    wire                              reader_end_all_frame;
    wire                              reader_fifo_full;
    wire                              reader_fifo_almost_full;
    wire                              reader_fifo_empty;

    wire [7:0]                        bias_out_data;
    wire                              bias_out_valid;
    wire                              bias_out_end_frame;
    wire                              bias_out_end_all_frame;
    wire                              bias_in_ready;
    reg                               bias_out_ready;

    wire                              ram_re_mux;
    wire [RAM_ADDR_WIDTH-1:0]         ram_raddr_mux;

    reg                               pw_stage_done;

    reg [IN_DATA_W-1:0]               input_mem [0:TOTAL_IN_BEATS-1];
    

    integer cyc;
    integer send_idx;
    reg send_done;
    reg reader_ram_stage;
    

    dw_pw_cac #(
        .IN_DATA_W(IN_DATA_W),
        .DW_COEFF_W(DW_COEFF_W),
        .DW_K_H(DW_K_H),
        .DW_K_W(DW_K_W),
        .DW_MUL_W(DW_MUL_W),
        .DW_SUM_W(DW_SUM_W),
        .DW_COL(DW_COL),
        .DW_ROW(DW_ROW),
        .DW_STRIDE(DW_STRIDE),
        .DW_PAD_TOP(DW_PAD_TOP),
        .DW_PAD_BOTTOM(DW_PAD_BOTTOM),
        .DW_PAD_LEFT(DW_PAD_LEFT),
        .DW_PAD_RIGHT(DW_PAD_RIGHT),
        .DW_COEFF_GRP_NUM(DW_COEFF_GRP_NUM),
        .DW_FRAME_GRP_NUM(DW_FRAME_GRP_NUM),
        .DW_MAC_PIPELINE(DW_MAC_PIPELINE),
        .DW_COEFF_INIT_FILE(DW_COEFF_INIT_FILE),
        .DW_BIAS_INIT_FILE(DW_BIAS_INIT_FILE),
        .DW_OUT_WIDTH(DW_OUT_WIDTH),
        .DW_SHIFT_VAL(DW_SHIFT_VAL),
        .DW_BIAS_GROUP_BITS(DW_BIAS_GROUP_BITS),
        .DW_BIAS_GROUP_SIZE(DW_BIAS_GROUP_SIZE),
        .DW_BIAS_CH_BITS(DW_BIAS_CH_BITS),
        .DW_MULT_CNT(DW_MULT_CNT),
        .DW_MULT_FACTOR0(DW_MULT_FACTOR0),
        .DW_MULT_FACTOR1(DW_MULT_FACTOR1),
        .DW_MULT_FACTOR2(DW_MULT_FACTOR2),
        .DW_MULT_FACTOR3(DW_MULT_FACTOR3),
        .DW_FIFO_DEPTH(DW_FIFO_DEPTH),
        .DW_FIFO_AF_LEVEL(DW_FIFO_AF_LEVEL),
        .PW_OUT_CH(PW_OUT_CH),
        .PW_COEFF_W(PW_COEFF_W),
        .PW_K_H(PW_K_H),
        .PW_K_W(PW_K_W),
        .PW_MUL_W(PW_MUL_W),
        .PW_SUM_W(PW_SUM_W),
        .PW_COEFF_GRP_NUM(PW_COEFF_GRP_NUM),
        .PW_FRAME_GRP_NUM(PW_FRAME_GRP_NUM),
        .PW_MAC_PIPELINE(PW_MAC_PIPELINE),
        .PW_COEFF_INIT_FILE(PW_COEFF_INIT_FILE),
        .PW_FIFO_DEPTH(PW_FIFO_DEPTH),
        .PW_FIFO_AF_LEVEL(PW_FIFO_AF_LEVEL),
        .RAM_DATA_W(RAM_DATA_W),
        .SEGMENTS(SEGMENTS),
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
        .IN_FRAME_SIZE(IN_FRAME_SIZE)
    ) u_dw_pw_cac (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .in_pixel(in_pixel),
        .in_end_all_frame(in_end_all_frame),
        .ram_re(ram_re_pw),
        .ram_raddr(ram_raddr_pw),
        .ram_rdata(ram_rdata),
        .ram_we(ram_we_pw),
        .ram_waddr(ram_waddr_pw),
        .ram_wdata(ram_wdata_pw),
        .end_frame(pw_end_frame),
        .end_all_frame(pw_end_all_frame)
    );

    assign ram_re_mux = pw_stage_done ? reader_ram_re : ram_re_pw;
    assign ram_raddr_mux = pw_stage_done ? reader_ram_raddr : ram_raddr_pw;

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_WIDTH),
        .ADDR_WIDTH(RAM_ADDR_WIDTH),
        .DEPTH(RAM_DEPTH)
    ) u_psum_ram (
        .clk(clk),
        .re(ram_re_mux),
        .raddr(ram_raddr_mux),
        .rdata(ram_rdata),
        .we(ram_we_pw),
        .waddr(ram_waddr_pw),
        .wdata(ram_wdata_pw)
    );

    assign reader_out_ready = bias_in_ready;

    ram_frame_to_fifo_reader #(
        .PIXELS_PER_FRAME(PIXEL_DEPTH),
        .SEGMENTS(SEGMENTS),
        .CHANNELS(PW_OUT_CH),
        .DATA_WIDTH(RAM_DATA_WIDTH),
        .ADDR_WIDTH(RAM_ADDR_WIDTH),
        .FIFO_DEPTH(16),
        .FIFO_AF_LEVEL(14),
        .OUT_W(RAM_DATA_W)
    ) u_ram_reader (
        .clk(clk),
        .rst_n(rst_n),
        .start(reader_start),
        .busy(reader_busy),
        .ram_re(reader_ram_re),
        .ram_raddr(reader_ram_raddr),
        .ram_rdata(ram_rdata),
        .out_valid(reader_out_valid),
        .out_ready(reader_out_ready),
        .out_data(reader_out_data),
        .fifo_full(reader_fifo_full),
        .fifo_almost_full(reader_fifo_almost_full),
        .fifo_empty(reader_fifo_empty),
        .end_frame(reader_end_frame),
        .end_all_frame(reader_end_all_frame)
    );

    bias_process_wrapper #(
        .IN_WIDTH(RAM_DATA_W),
        .BIAS_WIDTH(32),
        .OUT_WIDTH(8),
        .SHIFT_VAL(DW_SHIFT_VAL),
        .GROUP_BITS(PW_BIAS_GROUP_BITS),
        .GROUP_SIZE(PW_BIAS_GROUP_SIZE),
        .CH_BITS(PW_BIAS_CH_BITS),
        .FIFO_DEPTH(16),
        .FIFO_AF_LEVEL(14),
        .BIAS_INIT_FILE(PW_BIAS_INIT_FILE),
        .MULT_CNT(PW_MULT_CNT),
        .MULT_FACTOR0(PW_MULT_FACTOR0),
        .MULT_FACTOR1(PW_MULT_FACTOR1),
        .MULT_FACTOR2(PW_MULT_FACTOR2),
        .MULT_FACTOR3(PW_MULT_FACTOR3)
    ) u_bias_process (
        .clk(clk),
        .rst_n(rst_n),
        .in_pixel_data_bus(reader_out_data),
        .in_valid(reader_out_valid),
        .in_end_frame(reader_end_frame),
        .in_end_all_frame(reader_end_all_frame),
        .in_ready(bias_in_ready),
        .out_pixel_data(bias_out_data),
        .out_valid(bias_out_valid),
        .out_end_frame(bias_out_end_frame),
        .out_end_all_frame(bias_out_end_all_frame),
        .out_ready(bias_out_ready)
    );

    always #5 clk = ~clk;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pw_stage_done <= 1'b0;
        end else if (pw_end_all_frame) begin
            pw_stage_done <= 1'b1;
        end else if(!reader_ram_stage) begin
            pw_stage_done <= 1'b0;
        end
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        in_valid = 1'b0;
        in_pixel = {IN_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;
        reader_start = 1'b0;
        bias_out_ready = 1'b1;

        pw_stage_done = 1'b0;
        $readmemh(INPUT_INIT_FILE, input_mem, 0, TOTAL_IN_BEATS - 1);
        // expected output files removed (silent testbench)
        
        // start combinational-driven sender (see always @(*) / posedge block)
        send_idx = 0;
        send_done = 1'b0;
        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        while (!pw_stage_done) @(posedge clk);
        start_reader_once();
        while(reader_ram_stage) @(posedge clk);
        repeat (2) @(posedge clk);

        while (!pw_stage_done) @(posedge clk);
        start_reader_once();
        while(reader_ram_stage) @(posedge clk);
        repeat (2) @(posedge clk);

        while (!pw_stage_done) @(posedge clk);
        start_reader_once();
        while(reader_ram_stage) @(posedge clk);
        repeat (2) @(posedge clk);

        while (!pw_stage_done) @(posedge clk);
        start_reader_once();
        while(reader_ram_stage) @(posedge clk);

        repeat (PIXEL_DEPTH*2) @(posedge clk);

        #20; $stop;
    end

    reg send_group_frame;
    always @(*) begin
        in_valid = 1'b0;
        in_pixel = {IN_DATA_W{1'b0}};
        in_end_all_frame = 1'b0;

        if (!send_done && (send_idx < TOTAL_IN_BEATS)&& ! send_group_frame && ! reader_ram_stage) begin
            in_valid = 1'b1;
            in_pixel = input_mem[send_idx];
            in_end_all_frame = ((send_idx % ROUND_IN_BEATS) == (ROUND_IN_BEATS - 1)) ? 1'b1 : 1'b0;
        end
    end

    // combined posedge logic: advance send_idx on handshake and manage send_group_frame
    always @(posedge clk) begin
        if (!rst_n) begin
            send_idx <= 0;
            send_done <= 1'b0;
            send_group_frame <= 1'b0;
        end else begin
            // default: keep previous state unless events change it

            // advance index when a beat is accepted
            if (!send_done && in_valid && in_ready) begin
                // if current send_idx is the last beat of a round, set send_group_frame
                if ((send_idx % ROUND_IN_BEATS) == (ROUND_IN_BEATS - 1)) begin
                    send_group_frame <= 1'b1;
                end

                // advance index and mark completion when reaching total
                send_idx <= send_idx + 1;
                if (send_idx + 1 >= TOTAL_IN_BEATS) begin
                    send_done <= 1'b1;
                end
            end

            // clear send_group_frame when bias stage signals completed frame
            if (bias_out_end_all_frame && bias_out_valid && bias_out_ready) begin
                send_group_frame <= 1'b0;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            reader_ram_stage <= 1'b0;
        end else if (pw_end_all_frame) begin
            reader_ram_stage <= 1'b1;
        end else if (bias_out_end_all_frame && bias_out_valid && bias_out_ready) begin
            reader_ram_stage <= 1'b0;
        end
    end

    task start_reader_once;
    begin
        @(negedge clk);
        reader_start = 1'b1;
        @(negedge clk);
        reader_start = 1'b0;
    end
    endtask



    reg [31:0] bias_in_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bias_in_cnt <= 32'b0;
        end else begin
            // if(reader_out_valid && reader_out_ready) begin
            //     $display("beat %d: Reader_Out_Data: %d, End_Frame: %b, End_All_Frame: %b", bias_in_cnt, $signed(reader_out_data), reader_end_frame, reader_end_all_frame);
            //     bias_in_cnt <= bias_in_cnt + 1;
            // end
        end
    end
    reg [31:0] input_pixel_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            input_pixel_cnt <= 0;
        end else if (in_valid && in_ready) begin
            // $display("beat %d: Input_Pixel(hex): %h, End_All_Frame: %b", input_pixel_cnt, in_pixel, in_end_all_frame);
            // input_pixel_cnt <= input_pixel_cnt + 1;
        end
    end
    reg [31:0] out_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_cnt <= 32'b0;
        end else begin
            // if(bias_out_valid && bias_out_ready) begin
            //     $display("beat %d: PW_out_pixel(Hex): %h ,end_frame: %b,end_all_frame: %b", out_cnt, bias_out_data, bias_out_end_frame, bias_out_end_all_frame);
            //     out_cnt <=(out_cnt == (ROUND_IN_BEATS - 1)) ? 0 : out_cnt + 1;
            // end
        end
    end

endmodule
