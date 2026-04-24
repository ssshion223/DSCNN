module dw_pw_pingpong #(
    // ---------------- pingpong control ----------------
    parameter integer PINGPONG_ROUNDS      = 3,

    // ---------------- stream shape ----------------
    parameter integer PIXEL_DEPTH          = 125,
    parameter integer IN_FRAME_SIZE        = 64,//每轮输入帧数，等于输入通道数（depthwise输出的通道数）

    // ---------------- DW stage ----------------
	parameter integer IN_DATA_W            = 8,
	parameter integer DW_COEFF_W           = 8,
	parameter integer DW_K_H               = 3,
	parameter integer DW_K_W               = 3,
	parameter integer DW_MUL_W             = IN_DATA_W + DW_COEFF_W,
	parameter integer DW_SUM_W             = DW_MUL_W + $clog2(DW_K_H*DW_K_W),
	parameter integer DW_COL               = 5,
	parameter integer DW_ROW               = 25,
	parameter integer DW_STRIDE            = 1,
	parameter integer DW_PAD_TOP           = (DW_K_H-1)/2,
	parameter integer DW_PAD_BOTTOM        = (DW_K_H)/2,
	parameter integer DW_PAD_LEFT          = (DW_K_W-1)/2,
	parameter integer DW_PAD_RIGHT         = (DW_K_W)/2,
	parameter integer DW_COEFF_GRP_NUM     = IN_FRAME_SIZE * (PINGPONG_ROUNDS + 1),//一层计算64通道，复用4层系数
    parameter integer DW_FRAME_GRP_NUM     = IN_FRAME_SIZE,
	parameter integer DW_MAC_PIPELINE      = 1,
	parameter         DW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_dw.memh",
	parameter         DW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_dw_pingpong_bias.hex",
    parameter integer DW_OUT_WIDTH         = 8,
	parameter integer DW_SHIFT_VAL         = 16,
    parameter integer DW_BIAS_GROUP_SIZE   = (PINGPONG_ROUNDS + 1), //4层系数
    parameter integer DW_BIAS_GROUP_BITS   = $clog2(DW_BIAS_GROUP_SIZE),
    parameter integer DW_BIAS_CH_BITS      = $clog2(IN_FRAME_SIZE), //每个通道对应一个BIAS
    parameter         DW_MULT_CNT          = (PINGPONG_ROUNDS + 1),  // 量化乘数个数，每层一个乘数
    parameter signed [11:0] DW_MULT_FACTOR0 = 12'sd1246,  // layer1_dw
    parameter signed [11:0] DW_MULT_FACTOR1 = 12'sd828,  // layer2_dw
    parameter signed [11:0] DW_MULT_FACTOR2 = 12'sd652,  // layer3_dw
    parameter signed [11:0] DW_MULT_FACTOR3 = 12'sd412,  // layer4_dw
	parameter integer DW_FIFO_DEPTH        = 16,//输出数据缓存深度，2的幂次
	parameter integer DW_FIFO_AF_LEVEL     = 10,//注意该值不能太大，否则后端堵塞时容易丢失数据，参考值DW_FIFO_DEPTH- $clog2(DW_K_H*DW_K_W)- 2 

	// ---------------- PW matrix stage ----------------
	parameter integer PW_OUT_CH            = 64,
	parameter integer PW_COEFF_W           = 8,
	parameter integer PW_K_H               = 1,
	parameter integer PW_K_W               = 1,
	parameter integer PW_MUL_W             = DW_OUT_WIDTH + PW_COEFF_W,
	parameter integer PW_SUM_W             = PW_MUL_W + $clog2(PW_K_H*PW_K_W),
	parameter integer PW_COEFF_GRP_NUM     = PW_OUT_CH * (PINGPONG_ROUNDS + 1),//每个frame的64个通道对应一组系数
    parameter integer PW_FRAME_GRP_NUM     = PW_OUT_CH,
	parameter integer PW_MAC_PIPELINE      = 1,
	parameter         PW_COEFF_INIT_FILE   = "D:/vivado/exp/DSCNN/data/weights/DS-CNN_pingpong_pw.memh",
    parameter         PW_BIAS_INIT_FILE    = "D:/vivado/exp/DSCNN/data/bias/DS-CNN_pw_pingpong_bias.hex",
    parameter integer PW_BIAS_GROUP_SIZE   = (PINGPONG_ROUNDS + 1), //4层系数
    parameter integer PW_BIAS_GROUP_BITS   = $clog2(PW_BIAS_GROUP_SIZE),
    parameter integer PW_BIAS_CH_BITS      = $clog2(PW_OUT_CH), //每个通道对应一个BIAS
    parameter integer PW_MULT_CNT          = (PINGPONG_ROUNDS + 1),  // 量化乘数个数，每层一个乘数
    localparam signed [11:0] PW_MULT_FACTOR0 = 12'sd406, // layer1_pw
    localparam signed [11:0] PW_MULT_FACTOR1 = 12'sd461,// layer2_pw
    localparam signed [11:0] PW_MULT_FACTOR2 = 12'sd442,// layer3_pw
    localparam signed [11:0] PW_MULT_FACTOR3 = 12'sd623,// layer4_pw
    parameter integer PW_FIFO_DEPTH        = 4,
    parameter integer PW_FIFO_AF_LEVEL     = 2,

    // ---------------- accumulator / RAM ----------------  
    parameter integer RAM_SEGMENTS          = 4,
    parameter integer RAM_ONE_DATA_W        = PW_SUM_W + $clog2(PW_OUT_CH),
    parameter integer OUT_PIXEL_WIDTH       = 8
)(
    input  wire                                clk,
    input  wire                                rst_n,

    // 上游输入流（只在第0轮外部输入阶段有效）
    input  wire                                in_valid,
    output wire                                in_ready,
    input  wire  [IN_DATA_W-1:0]               in_pixel,
    input  wire                                in_end_all_frame,

    output wire                                out_valid,
    input  wire                                out_ready,
    output wire                                end_frame,
    output wire                                end_all_frame,
    output wire  [OUT_PIXEL_WIDTH-1:0]         out_data,

    input  wire                                 start,
    output wire                                 busy
);

    localparam integer SEG_CHANS        = PW_OUT_CH / RAM_SEGMENTS;
    localparam integer RAM_DATA_W       = SEG_CHANS * RAM_ONE_DATA_W;
    localparam integer RAM_DEPTH        = PIXEL_DEPTH * RAM_SEGMENTS;
    localparam integer RAM_ADDR_W       = (RAM_DEPTH <= 1) ? 1 : $clog2(RAM_DEPTH);
    localparam integer PASS_TOTAL       = PINGPONG_ROUNDS + 1;
    localparam integer PASS_TOTAL_W     = (PASS_TOTAL <= 1) ? 1 : $clog2(PASS_TOTAL);
    localparam integer SEG_CHANS_W      = (SEG_CHANS <= 1) ? 1 : $clog2(SEG_CHANS);

    localparam integer FRAME_PIXELS_W = $clog2(PIXEL_DEPTH);
    localparam integer FRAME_SIZE_W = $clog2(PW_OUT_CH);
    // ---------------- dual ram ports ----------------
    reg                             ram0_re;
    reg [RAM_ADDR_W-1:0]            ram0_raddr;
    wire [RAM_DATA_W-1:0]           ram0_rdata;
    reg                             ram0_we;
    reg [RAM_ADDR_W-1:0]            ram0_waddr;
    reg [RAM_DATA_W-1:0]            ram0_wdata;

    reg                             ram1_re;
    reg [RAM_ADDR_W-1:0]            ram1_raddr;
    wire [RAM_DATA_W-1:0]           ram1_rdata;
    reg                             ram1_we;
    reg [RAM_ADDR_W-1:0]            ram1_waddr;
    reg [RAM_DATA_W-1:0]            ram1_wdata;

    // ---------------- RAM to FIFO reader ----------------
    reg                                 ram_reader_start;
    wire                                ram_reader_busy;
    wire                                ram_reader_ram_re;
    wire [RAM_ADDR_W-1:0]               ram_reader_ram_raddr;
    reg [RAM_DATA_W-1:0]                ram_reader_ram_rdata;
    wire [RAM_ONE_DATA_W-1:0]           ram_reader_out_data;
    wire                                ram_reader_out_valid;
    wire                                ram_reader_end_frame;
    wire                                ram_reader_end_all_frame;
    reg                                 ram_reader_out_ready;

    // ---------------- bias relu wrapper ----------------
    reg                                 bias_process_in_valid;
    wire                                bias_process_in_ready;
    reg                                 bias_process_in_end_frame;
    reg                                 bias_process_in_end_all_frame;
    reg  [RAM_ONE_DATA_W-1:0]           bias_process_in_data;
    wire                                bias_process_out_valid;
    wire                                bias_process_out_end_frame;
    wire                                bias_process_out_end_all_frame;
    reg                                 bias_process_out_ready;
    wire [OUT_PIXEL_WIDTH-1:0]          bias_process_out_data;

    // ---------------- dw_pw_cac ----------------
    wire                                pw_conv_ram_re;
    wire [RAM_ADDR_W-1:0]               pw_conv_ram_raddr;
    reg [RAM_DATA_W-1:0]                pw_conv_ram_rdata;
    wire                                pw_conv_ram_we;
    wire                                pw_conv_in_ready;
    reg                                 pw_conv_in_valid;
    reg [IN_DATA_W-1:0]                 pw_conv_in_pixel;
    reg                                 pw_conv_in_end_all_frame;
    wire [RAM_ADDR_W-1:0]               pw_conv_ram_waddr;
    wire [RAM_DATA_W-1:0]               pw_conv_ram_wdata;
    wire                                pw_conv_end_frame;
    wire                                pw_conv_end_all_frame;


    localparam  IDLE             = 3'b000,
                EXTERNAL_INPUT   = 3'b001,
                RAM0_INPUT       = 3'b010,
                RAM1_INPUT       = 3'b011,
                OUT              = 3'b100;
    
    reg [2:0] state, next_state;
    reg [PASS_TOTAL_W-1:0] pass_cnt; // 记录当前是第几轮pingpong
    wire start_fire = start && !busy; // 仅在非忙状态下响应 start 信号
    wire pass_cnt_wrap = (pass_cnt == PASS_TOTAL-1);
    wire out_fire = out_valid && out_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pass_cnt <= {PASS_TOTAL_W{1'b0}};
        end else if (state==IDLE) begin
            pass_cnt <= {PASS_TOTAL_W{1'b0}};
        end else if (pw_conv_end_all_frame) begin
            pass_cnt <= (pass_cnt_wrap) ? {PASS_TOTAL_W{1'b0}} : pass_cnt + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(*) begin
        case(state)
            IDLE: begin
                if (start_fire) 
                    next_state = EXTERNAL_INPUT;
                else 
                    next_state = IDLE;
            end
            EXTERNAL_INPUT: begin
                if(pw_conv_end_all_frame && !ram_reader_busy)
                    next_state = RAM0_INPUT;
                else
                    next_state = EXTERNAL_INPUT;
            end
            RAM0_INPUT: begin
                if (pw_conv_end_all_frame && !ram_reader_busy) begin
                    if(pass_cnt_wrap)
                        next_state = OUT;
                    else
                        next_state = RAM1_INPUT;
                end else
                    next_state = RAM0_INPUT;
            end
            RAM1_INPUT: begin
                if (pw_conv_end_all_frame && !ram_reader_busy) begin
                    if(pass_cnt_wrap)
                        next_state = OUT;
                    else
                        next_state = RAM0_INPUT;
                end else
                    next_state = RAM1_INPUT;
            end
            OUT: begin
                if (bias_process_out_end_all_frame && out_fire)
                    next_state = IDLE ;
                else
                    next_state = OUT;
            end
        endcase
    end

    always @(*) begin
        case(state)
            EXTERNAL_INPUT: begin
                pw_conv_in_valid = in_valid;
                pw_conv_in_pixel = in_pixel;
                pw_conv_in_end_all_frame = in_end_all_frame;
                pw_conv_ram_rdata = ram0_rdata; // 外部输入阶段从RAM0读取数据
            end
            RAM0_INPUT: begin
                pw_conv_in_valid = bias_process_out_valid; // 读使能作为数据有效信号
                pw_conv_in_pixel = bias_process_out_data; // 取出最低位宽的数据输入
                pw_conv_in_end_all_frame = bias_process_out_end_all_frame;
                pw_conv_ram_rdata = ram1_rdata; // RAM0输入阶段从RAM0读取数据写入RAM1
            end
            RAM1_INPUT: begin
                pw_conv_in_valid = bias_process_out_valid; // 读使能作为数据有效信号
                pw_conv_in_pixel = bias_process_out_data; // 取出最低位宽的数据输入
                pw_conv_in_end_all_frame = bias_process_out_end_all_frame;
                pw_conv_ram_rdata = ram0_rdata; // RAM1输入阶段从RAM1读取数据写入RAM0
            end
            default: begin
                pw_conv_in_valid = 1'b0;
                pw_conv_in_pixel = {IN_DATA_W{1'b0}};
                pw_conv_in_end_all_frame = 1'b0;
                pw_conv_ram_rdata = {RAM_DATA_W{1'b0}};
            end
        endcase
    end

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
        .RAM_ONE_DATA_W(RAM_ONE_DATA_W),
        .SEGMENTS(RAM_SEGMENTS),
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .RAM_ADDR_W(RAM_ADDR_W),
        .IN_FRAME_SIZE(IN_FRAME_SIZE)
    ) u_dw_pw_cac (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(pw_conv_in_valid),
        .in_ready(pw_conv_in_ready),
        .in_pixel(pw_conv_in_pixel),
        .in_end_all_frame(pw_conv_in_end_all_frame),
        .ram_re(pw_conv_ram_re),
        .ram_raddr(pw_conv_ram_raddr),
        .ram_rdata(pw_conv_ram_rdata),
        .ram_we(pw_conv_ram_we),
        .ram_waddr(pw_conv_ram_waddr),
        .ram_wdata(pw_conv_ram_wdata),
        .end_frame(pw_conv_end_frame),
        .end_all_frame(pw_conv_end_all_frame)
    );

    localparam IS_EVEN_ROUND = (PINGPONG_ROUNDS % 2 == 0);

    always @(*) begin
        ram0_re    = 1'b0;
        ram0_raddr = {RAM_ADDR_W{1'b0}};
        ram0_we    = 1'b0;
        ram0_waddr = {RAM_ADDR_W{1'b0}};
        ram0_wdata = {RAM_DATA_W{1'b0}};

        ram1_re    = 1'b0;
        ram1_raddr = {RAM_ADDR_W{1'b0}};
        ram1_we    = 1'b0;
        ram1_waddr = {RAM_ADDR_W{1'b0}};
        ram1_wdata = {RAM_DATA_W{1'b0}};
        case(state)
            EXTERNAL_INPUT: begin
                ram0_re    = pw_conv_ram_re;
                ram0_raddr = pw_conv_ram_raddr;
                ram0_we    = pw_conv_ram_we;
                ram0_waddr = pw_conv_ram_waddr;
                ram0_wdata = pw_conv_ram_wdata;
            end
            RAM0_INPUT: begin
                ram0_re    = ram_reader_ram_re;
                ram0_raddr = ram_reader_ram_raddr;
                ram1_re    = pw_conv_ram_re;
                ram1_raddr = pw_conv_ram_raddr;
                ram1_we    = pw_conv_ram_we;
                ram1_waddr = pw_conv_ram_waddr;
                ram1_wdata = pw_conv_ram_wdata;
            end
            RAM1_INPUT: begin
                ram0_re    = pw_conv_ram_re;
                ram0_raddr = pw_conv_ram_raddr;
                ram0_we    = pw_conv_ram_we;
                ram0_waddr = pw_conv_ram_waddr;
                ram0_wdata = pw_conv_ram_wdata;
                ram1_re    = ram_reader_ram_re;
                ram1_raddr = ram_reader_ram_raddr;
            end
            OUT: begin
                if (IS_EVEN_ROUND) begin
                    ram0_re    = ram_reader_ram_re;
                    ram0_raddr = ram_reader_ram_raddr;
                end else begin
                    ram1_re    = ram_reader_ram_re;
                    ram1_raddr = ram_reader_ram_raddr;
                end
            end
            default: begin
                end
            endcase
    end

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_W),
        .ADDR_WIDTH(RAM_ADDR_W),
        .DEPTH(RAM_DEPTH)
    ) u_ram0 (
        .clk(clk),
        .re(ram0_re),
        .raddr(ram0_raddr),
        .rdata(ram0_rdata),
        .we(ram0_we),
        .waddr(ram0_waddr),
        .wdata(ram0_wdata)
    );

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_W),
        .ADDR_WIDTH(RAM_ADDR_W),
        .DEPTH(RAM_DEPTH)
    ) u_ram1 (
        .clk(clk),
        .re(ram1_re),
        .raddr(ram1_raddr),
        .rdata(ram1_rdata),
        .we(ram1_we),
        .waddr(ram1_waddr),
        .wdata(ram1_wdata)
    );

    always @(*) begin
        case(state)
            RAM0_INPUT: begin
                ram_reader_ram_rdata = ram0_rdata; // RAM0输入阶段从RAM0读取数据
                ram_reader_out_ready = bias_process_in_ready; // 将RAM读取模块的输出准备好信号连接到bias处理模块的输入准备好信号
            end
            RAM1_INPUT: begin
                ram_reader_ram_rdata = ram1_rdata; // RAM1输入阶段从RAM1读取数据
                ram_reader_out_ready = bias_process_in_ready; // 将RAM读取模块的输出准备好信号连接到bias处理模块的输入准备好信号
            end
            OUT: begin
                if(PINGPONG_ROUNDS % 2 == 0) // 如果不需要pingpong，直接从RAM0输出
                    ram_reader_ram_rdata = ram0_rdata;
                else
                    ram_reader_ram_rdata = ram1_rdata;
                ram_reader_out_ready = bias_process_in_ready;
            end
            default: begin
                ram_reader_ram_rdata = {RAM_DATA_W{1'b0}};
                ram_reader_out_ready = 1'b0;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ram_reader_start <= 1'b0;
        end else if (state != next_state ) begin
            if(state == EXTERNAL_INPUT || state == RAM0_INPUT || state == RAM1_INPUT) begin
                ram_reader_start <= 1'b1; // 在RAM输入阶段转换时启动RAM读取模块
            end else begin
                ram_reader_start <= 1'b0;
            end
        end else begin
            ram_reader_start <= 1'b0; // 其他时候保持不启动
        end
    end

    ram_frame_to_fifo_reader #(
        .PIXELS_PER_FRAME(PIXEL_DEPTH),
        .SEGMENTS(RAM_SEGMENTS),
        .CHANNELS(PW_OUT_CH),
        .RAM_DATA_W(RAM_DATA_W),
        .RAM_ADDR_W(RAM_ADDR_W),
        .FIFO_DEPTH(16),
        .FIFO_AF_LEVEL(10),
        .OUT_W(RAM_ONE_DATA_W)
    ) ram_reader_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(ram_reader_start), // TODO: 根据状态机控制何时开始读RAM
        .busy(ram_reader_busy),  // TODO: RAM读取模块的忙信号，用于状态机转移
        .ram_re(ram_reader_ram_re), // TODO: 根据状态机控制读哪个RAM
        .ram_raddr(ram_reader_ram_raddr), // TODO: 根据状态机生成读地址
        .ram_rdata(ram_reader_ram_rdata), // RAM数据输入
        .out_valid(ram_reader_out_valid), // TODO: RAM数据有效信号，连接到 downstream 处理模块
        .out_ready(ram_reader_out_ready), // TODO: downstream 处理模块的准备好信号，连接到RAM读取模块
        .out_data(ram_reader_out_data),   // TODO: RAM输出数据总线，连接到 downstream 处理模块
        .end_frame(ram_reader_end_frame),
        .end_all_frame(ram_reader_end_all_frame)
    );


     always @(*) begin
        case(state)
            RAM0_INPUT,RAM1_INPUT: begin
                bias_process_in_valid = ram_reader_out_valid;
                bias_process_in_data = ram_reader_out_data;
                bias_process_in_end_frame = ram_reader_end_frame;
                bias_process_in_end_all_frame = ram_reader_end_all_frame;
                bias_process_out_ready = pw_conv_in_ready; // 将bias处理模块的准备好信号连接到 dw_pw_cac 的输入准备好信号
            end
            OUT: begin
                bias_process_in_valid = ram_reader_out_valid;
                bias_process_in_data = ram_reader_out_data;
                bias_process_in_end_frame = ram_reader_end_frame;
                bias_process_in_end_all_frame = ram_reader_end_all_frame;
                bias_process_out_ready = out_ready;
            end
            default: begin
                bias_process_in_valid = 1'b0;
                bias_process_in_data = {{RAM_ONE_DATA_W{1'b0}}};
                bias_process_in_end_frame = 1'b0;
                bias_process_in_end_all_frame = 1'b0;
                bias_process_out_ready = 1'b0;
            end
        endcase
    end
    bias_process_wrapper #(
        .IN_WIDTH(RAM_ONE_DATA_W),
        .BIAS_WIDTH(32),
        .OUT_WIDTH(OUT_PIXEL_WIDTH),
        .SHIFT_VAL(DW_SHIFT_VAL),
        .BIAS_INIT_FILE(PW_BIAS_INIT_FILE),
        .BIAS_GROUP_BITS(PW_BIAS_GROUP_BITS),
        .BIAS_GROUP_SIZE(PW_BIAS_GROUP_SIZE),
        .BIAS_CH_BITS(PW_BIAS_CH_BITS),
        .MULT_CNT(PW_MULT_CNT),
        .MULT_FACTOR0(PW_MULT_FACTOR0),
        .MULT_FACTOR1(PW_MULT_FACTOR1),
        .MULT_FACTOR2(PW_MULT_FACTOR2),
        .MULT_FACTOR3(PW_MULT_FACTOR3)
    ) bias_process_inst (
        .clk(clk),
        .rst_n(rst_n),
        .in_pixel_data_bus(bias_process_in_data), // RAM读取模块输出的数据总线
        .in_valid(bias_process_in_valid), // RAM读取模块的数据有效信号
        .in_end_frame(bias_process_in_end_frame),
        .in_end_all_frame(bias_process_in_end_all_frame),
        .in_ready(bias_process_in_ready), // RAM读取模块的准备好信号
        .out_pixel_data(bias_process_out_data),
        .out_valid(bias_process_out_valid),
        .out_end_frame(bias_process_out_end_frame),
        .out_end_all_frame(bias_process_out_end_all_frame),
        .out_ready(bias_process_out_ready)
    );

    assign in_ready = (state == EXTERNAL_INPUT) ? pw_conv_in_ready : 1'b0;
    assign out_valid = (state == OUT) ? bias_process_out_valid : 1'b0;
    assign out_data = (state == OUT) ? bias_process_out_data : {OUT_PIXEL_WIDTH{1'b0}};
    assign end_frame = (state == OUT) ? bias_process_out_end_frame : 1'b0;
    assign end_all_frame = (state == OUT) ? bias_process_out_end_all_frame : 1'b0;
    assign busy = (state != IDLE);

    //test
    reg [31:0] out_cnt ;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_cnt <= 32'b0;
        end else begin
            // if(bias_process_out_valid && bias_process_out_ready) begin
            //     $display("beat %d: PW_out_pixel(Hex): %h ,end_frame: %b,end_all_frame: %b", out_cnt, bias_process_out_data, bias_process_out_end_frame, bias_process_out_end_all_frame);
            //     out_cnt <=(out_cnt == 125*64 -1) ? 0 : out_cnt + 1;
            // end
        end
    end


endmodule