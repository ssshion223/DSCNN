//==============================================================================
// 模块名：conv_mac_tree
// 功能说明：
//   对单个输入窗口与单个卷积核执行乘累加，输出单个求和结果。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_user/window_bus/coeff_bus
// 输出接口：
//   mac_sum/out_valid（无缓冲，若有效时未读出则消失）/out_user（可自定义的信号数据）
// 时序特性：
//   归约树可通过 PIPELINE 参数选择组合或流水实现（0组合/1流水），强烈建议使用流水线实现。
//==============================================================================
module conv_mac_tree #(
    parameter integer DATA_W  = 8,                      // 输入数据位宽
    parameter integer COEFF_W = 8,                      // 系数位宽
    parameter integer K_H     = 3,                      // 卷积核高度
    parameter integer K_W     = 3,                      // 卷积核宽度
    parameter integer MUL_W   = DATA_W + COEFF_W,       // 乘法结果位宽
    parameter integer SUM_W   = MUL_W + $clog2(K_H*K_W),// 累加输出位宽
    parameter integer USER_W  = 1,                      // 用户侧带位宽
    parameter integer PIPELINE = 1                      // 加法树模式：0组合/1流水
)(
    input  wire                          clk,        // 时钟信号
    input  wire                          rst_n,      // 低有效复位
    input  wire                          in_valid,   // 输入有效
    input  wire [USER_W-1:0]             in_user,    // 输入侧带信息
    input  wire signed [K_H*K_W*DATA_W-1:0]   window_bus, // 输入窗口数据
    input  wire signed [K_H*K_W*COEFF_W-1:0]  coeff_bus,  // 输入系数数据
    output wire signed [SUM_W-1:0]        mac_sum,   // MAC 累加结果
    output wire                          out_valid,  // 输出有效
    output wire [USER_W-1:0]             out_user    // 输出侧带信息
    
);
    wire signed [K_H*K_W*MUL_W-1:0] product_bus;

    conv_mult_products #(
        .DATA_W (DATA_W),
        .COEFF_W(COEFF_W),
        .OUT_W  (MUL_W), // 乘积位宽等于数据位宽加权重位宽以防止溢出
        .K_H    (K_H),
        .K_W    (K_W)
    ) u_mults (
        .window_bus (window_bus),
        .coeff_bus  (coeff_bus),
        .product_bus(product_bus)
    );

    conv_adder_tree #(
        .IN_W (MUL_W), // 乘积位宽
        .OUT_W(SUM_W), // 乘积累加后的位宽
        .K_H  (K_H),
        .K_W  (K_W),
        .PIPELINE(PIPELINE),
        .USER_W(USER_W)
    ) u_adder_tree (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .product_bus(product_bus),
        .sum_out(mac_sum),
        .out_valid(out_valid),
        .in_user(in_user),
        .out_user(out_user)
    );

endmodule

//==============================================================================
// 模块名：conv_mult_products
// 功能说明：
//   对 window_bus 与 coeff_bus 做逐元素有符号乘法。
// 时钟与复位：
//   纯组合模块（无时钟/复位）
// 输入接口：
//   打包的窗口向量与系数向量
// 输出接口：
//   打包的乘法结果向量
// 时序特性：
//   纯组合数据通路。
//==============================================================================
module conv_mult_products #(
    parameter integer DATA_W  = 8,                // 输入数据位宽
    parameter integer COEFF_W = 8,                // 系数位宽
    parameter integer OUT_W   = DATA_W + COEFF_W, // 乘法结果位宽
    parameter integer K_H     = 3,                // 卷积核高度
    parameter integer K_W     = 3                 // 卷积核宽度
)(
    input  wire signed [K_H*K_W*DATA_W-1:0]  window_bus,  // 输入窗口展开总线
    input  wire signed [K_H*K_W*COEFF_W-1:0] coeff_bus,   // 输入系数展开总线
    output wire signed [K_H*K_W*OUT_W-1:0]   product_bus  // 点乘结果展开总线
);
    localparam integer WIN_SIZE = K_H * K_W;

    wire signed [DATA_W-1:0]  win_vec   [0:WIN_SIZE-1];
    wire signed [COEFF_W-1:0] coeff_vec [0:WIN_SIZE-1];

    genvar gi;

    generate
        for (gi = 0; gi < WIN_SIZE; gi = gi + 1) begin : gen_unpack_and_mul
            assign win_vec[gi]   = window_bus[gi*DATA_W +: DATA_W];
            assign coeff_vec[gi] = coeff_bus[gi*COEFF_W +: COEFF_W];
            assign product_bus[gi*OUT_W +: OUT_W] = $signed(win_vec[gi]) * $signed(coeff_vec[gi]);
        end
    endgenerate
endmodule

//==============================================================================
// 模块名：conv_adder_tree
// 功能说明：
//   乘积归约加法树封装，可切换组合或流水模式。
// 时钟与复位：
//   clk：上升沿时钟（流水模式使用）
//   rst_n：低有效异步复位（流水模式使用）
// 输入接口：
//   in_valid/in_user/product_bus
// 输出接口：
//   sum_out/out_valid/out_user
// 时序特性：
//   由 PIPELINE 控制：0 为组合树，1 为寄存流水树。
//==============================================================================
module conv_adder_tree #(
    parameter integer IN_W = 16,                      // 输入乘积位宽
    parameter integer K_H   = 3,                      // 卷积核高度
    parameter integer K_W   = 3,                      // 卷积核宽度
    parameter integer OUT_W = (IN_W + $clog2(K_H*K_W)), // 输出累加位宽
    parameter integer USER_W = 1,                     // 用户侧带位宽
    parameter integer PIPELINE = 1                    // 加法树模式：0组合/1流水
) (
    input  wire                          clk,        // 时钟信号
    input  wire                          rst_n,      // 低有效复位
    input  wire                          in_valid,   // 输入有效
    input  wire [USER_W-1:0]             in_user,    // 输入侧带
    input  wire signed [K_H*K_W*IN_W-1:0]  product_bus, // 输入乘积总线
    output reg  signed [OUT_W-1:0]      sum_out,    // 累加输出
    output reg                           out_valid,  // 输出有效
    output reg [USER_W-1:0]             out_user    // 输出侧带
);
    localparam integer WIN_SIZE = K_H * K_W;
    localparam integer LEVEL = $clog2(WIN_SIZE);
    localparam integer TREE_OUT_W = IN_W + LEVEL;

    wire signed [TREE_OUT_W-1:0] comb_sum_out;
    wire signed [TREE_OUT_W-1:0] pipe_sum_out;
    wire                         pipe_out_valid;
    wire [USER_W-1:0]            pipe_out_user;

    generate
        if (PIPELINE == 0) begin : gen_comb_tree
            conv_adder_tree_comb #(
                .IN_W (IN_W),
                .N    (WIN_SIZE),
                .OUT_W(TREE_OUT_W)
            ) u_comb_tree (
                .product_bus(product_bus),
                .sum_out    (comb_sum_out)
            );

            // prevent unused port warnings when PIPELINE == 0
            // reference clk/rst_n so synthesis recognizes ports as used
            wire _unused_clk = clk;
            wire _unused_rst_n = rst_n;

            always @(*) begin
                if (OUT_W >= TREE_OUT_W) begin
                    sum_out = {{(OUT_W-TREE_OUT_W){comb_sum_out[TREE_OUT_W-1]}}, comb_sum_out};
                end else begin
                    sum_out = comb_sum_out[OUT_W-1:0];
                end
                out_valid = in_valid;
                out_user = in_user;
            end
        end else begin : gen_pipe_tree
            conv_adder_tree_pipe #(
                .IN_W (IN_W),
                .N    (WIN_SIZE),
                .OUT_W(TREE_OUT_W),
                .USER_W(USER_W)
            ) u_pipe_tree (
                .clk       (clk),
                .rst_n     (rst_n),
                .in_valid  (in_valid),
                .in_user   (in_user),
                .product_bus(product_bus),
                .sum_out   (pipe_sum_out),
                .out_valid (pipe_out_valid),
                .out_user  (pipe_out_user)
            );

            always @(*) begin
                if (OUT_W >= TREE_OUT_W) begin
                    sum_out = {{(OUT_W-TREE_OUT_W){pipe_sum_out[TREE_OUT_W-1]}}, pipe_sum_out};
                end else begin
                    sum_out = pipe_sum_out[OUT_W-1:0];
                end
                out_valid = pipe_out_valid;
                out_user = pipe_out_user;
            end
        end
    endgenerate

endmodule



//==============================================================================
// 模块名：adder_comb
// 功能说明：
//   组合归约单级：对输入做两两符号扩展相加。
// 时钟与复位：
//   纯组合模块
// 输入接口：
//   product_bus
// 输出接口：
//   sum_out
// 时序特性：
//   单级纯组合逻辑。
//==============================================================================
module adder_comb #(
    parameter integer IN_W = 16,       // 输入元素位宽
    parameter integer OUT_W = IN_W + 1,// 输出元素位宽
    parameter integer IN_NUM = 9,       // 输入元素个数
    parameter integer OUT_NUM = 5       // 输出元素个数
)(
    input  wire signed [IN_NUM*IN_W-1:0]   product_bus, // 输入向量
    output wire signed [OUT_NUM*OUT_W-1:0] sum_out      // 两两相加后的输出向量
);
    generate
    genvar gi;
    for (gi = 0; gi < OUT_NUM; gi = gi + 1) begin
        if(gi < IN_NUM/2)  begin
            assign sum_out[gi*OUT_W +: OUT_W] =
                $signed({{(OUT_W-IN_W){product_bus[(2*gi)*IN_W + IN_W - 1]}}, product_bus[(2*gi)*IN_W +: IN_W]}) +
                $signed({{(OUT_W-IN_W){product_bus[(2*gi+1)*IN_W + IN_W - 1]}}, product_bus[(2*gi+1)*IN_W +: IN_W]});
        end else if(gi == IN_NUM/2 && (IN_NUM % 2) == 1) begin
            assign sum_out[gi*OUT_W +: OUT_W] =
                $signed({{(OUT_W-IN_W){product_bus[(IN_NUM-1)*IN_W + IN_W - 1]}}, product_bus[(IN_NUM-1)*IN_W +: IN_W]});
        end else begin
            assign sum_out[gi*OUT_W +: OUT_W] = {OUT_W{1'b0}};
        end
    end
    endgenerate  
endmodule

//==============================================================================
// 模块名：conv_adder_tree_comb
// 功能说明：
//   全组合多级加法树归约。
// 时钟与复位：
//   纯组合模块
// 输入接口：
//   product_bus
// 输出接口：
//   sum_out
// 时序特性：
//   全组合归约到最终和。
//==============================================================================
module conv_adder_tree_comb #(
    parameter integer IN_W = 16,            // 输入元素位宽
    parameter integer N     = 9,            // 输入元素总数
    parameter integer OUT_W = IN_W + $clog2(N) // 输出位宽
)(
    input  wire signed [N*IN_W-1:0] product_bus, // 输入向量
    output wire signed [OUT_W-1:0]  sum_out      // 组合树求和结果
);
    localparam integer LEVEL = $clog2(N);
    generate
        if (LEVEL == 0) begin : gen_single
            if (OUT_W >= IN_W) begin : gen_single_ext
                assign sum_out = {{(OUT_W-IN_W){product_bus[IN_W-1]}}, product_bus[0 +: IN_W]};
            end else begin : gen_single_trunc
                assign sum_out = product_bus[0 +: OUT_W];
            end
        end else begin : gen_pipe_levels
            genvar gi;
            for(gi=0;gi<LEVEL; gi=gi+1) begin : gen_levels
                localparam integer IN_NUM = (N + (1 << gi) - 1) >> gi;
                localparam integer OUT_NUM = (IN_NUM + 1) >> 1;
                localparam integer STAGE_IN_W = IN_W + gi;
                localparam integer STAGE_OUT_W = STAGE_IN_W + 1;
                wire signed [IN_NUM*STAGE_IN_W-1:0] level_bus;
                wire signed [OUT_NUM*STAGE_OUT_W-1:0] next_bus;
                if(gi == 0) begin
                    assign level_bus = product_bus;
                end else begin
                    assign level_bus = gen_levels[gi-1].next_bus;
                end
                adder_comb #(
                    .IN_W(STAGE_IN_W),
                    .OUT_W(STAGE_OUT_W),
                    .IN_NUM(IN_NUM),
                    .OUT_NUM(OUT_NUM)
                ) u_pipe (
                    .product_bus(level_bus),
                    .sum_out(next_bus)
                );
                if(gi == LEVEL-1) begin
                    if (OUT_W >= STAGE_OUT_W) begin : gen_last_ext
                        assign sum_out = {{(OUT_W-STAGE_OUT_W){next_bus[STAGE_OUT_W-1]}}, next_bus[0 +: STAGE_OUT_W]};
                    end else begin : gen_last_trunc
                        assign sum_out = next_bus[0 +: OUT_W];
                    end
                end
            end
        end
    endgenerate
endmodule


//==============================================================================
// 模块名：adder_pipe
// 功能说明：
//   单级寄存归约，携带 valid/user 侧带传递。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_user/product_bus
// 输出接口：
//   sum_out/out_valid/out_user
// 时序特性：
//   单级流水寄存。
//==============================================================================
module adder_pipe #(
    parameter integer IN_W = 16,        // 输入元素位宽
    parameter integer OUT_W = IN_W + 1, // 输出元素位宽
    parameter integer IN_NUM = 9,        // 输入元素个数
    parameter integer OUT_NUM = 5,       // 输出元素个数
    parameter integer USER_W = 1         // 用户侧带位宽
)(
    input  wire                             clk,      // 时钟信号
    input  wire                             rst_n,    // 低有效复位
    input  wire                             in_valid, // 输入有效
    input  wire [USER_W-1:0]                in_user,  // 输入侧带
    input  wire signed [IN_NUM*IN_W-1:0]    product_bus, // 输入向量
    output reg signed [OUT_NUM*OUT_W-1:0]   sum_out,  // 级联求和输出
    output reg                              out_valid,// 输出有效
    output reg [USER_W-1:0]                 out_user  // 输出侧带
);
    generate
    genvar gi;
    for (gi = 0; gi < OUT_NUM; gi = gi + 1) begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                sum_out[gi*OUT_W +: OUT_W] <= {OUT_W{1'b0}};
            end else if(gi < IN_NUM/2)  begin
                sum_out[gi*OUT_W +: OUT_W] <=
                $signed({{(OUT_W-IN_W){product_bus[(2*gi)*IN_W + IN_W - 1]}}, product_bus[(2*gi)*IN_W +: IN_W]}) +
                $signed({{(OUT_W-IN_W){product_bus[(2*gi+1)*IN_W + IN_W - 1]}}, product_bus[(2*gi+1)*IN_W +: IN_W]});
            end else if(gi == IN_NUM/2 && (IN_NUM % 2) == 1) begin
                sum_out[gi*OUT_W +: OUT_W] <=
                $signed({{(OUT_W-IN_W){product_bus[(IN_NUM-1)*IN_W + IN_W - 1]}}, product_bus[(IN_NUM-1)*IN_W +: IN_W]});
            end else begin
                sum_out[gi*OUT_W +: OUT_W] <= {OUT_W{1'b0}};
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            out_valid <= 1'b0;
            out_user <= {USER_W{1'b0}};
        end else begin
            out_valid <= in_valid;
            out_user <= in_user;
        end
    end
    endgenerate
   
endmodule

//==============================================================================
// 模块名：conv_adder_tree_pipe
// 功能说明：
//   由 adder_pipe 级联构成的多级寄存加法树。
// 时钟与复位：
//   clk：上升沿时钟
//   rst_n：低有效异步复位
// 输入接口：
//   in_valid/in_user/product_bus
// 输出接口：
//   sum_out/out_valid/out_user
// 时序特性：
//   延迟等于归约层数（$clog2(N)）。
//==============================================================================
module conv_adder_tree_pipe #(
    parameter integer IN_W = 16,              // 输入元素位宽
    parameter integer N     = 9,              // 输入元素总数
    parameter integer OUT_W = IN_W + $clog2(N), // 输出位宽
    parameter integer USER_W = 1              // 用户侧带位宽
)(
    input  wire                        clk,       // 时钟信号
    input  wire                        rst_n,     // 低有效复位
    input  wire                        in_valid,  // 输入有效
    input  wire [USER_W-1:0]           in_user,   // 输入侧带
    input  wire signed [N*IN_W-1:0]   product_bus,// 输入向量
    output wire signed [OUT_W-1:0]    sum_out,    // 流水树求和输出
    output wire                       out_valid,   // 输出有效
    output wire [USER_W-1:0]          out_user     // 输出侧带
);
    localparam integer LEVEL = $clog2(N);
    generate
        if (LEVEL == 0) begin : gen_single
            if (OUT_W >= IN_W) begin : gen_single_ext
                assign sum_out = {{(OUT_W-IN_W){product_bus[IN_W-1]}}, product_bus[0 +: IN_W]};
            end else begin : gen_single_trunc
                assign sum_out = product_bus[0 +: OUT_W];
            end
            assign out_valid = in_valid;
            assign out_user = in_user;
        end else begin : gen_pipe_levels
            genvar gi;
            for(gi=0;gi<LEVEL; gi=gi+1) begin : gen_levels
                localparam integer IN_NUM = (N + (1 << gi) - 1) >> gi;
                localparam integer OUT_NUM = (IN_NUM + 1) >> 1;
                localparam integer STAGE_IN_W = IN_W + gi;
                localparam integer STAGE_OUT_W = STAGE_IN_W + 1;
                wire signed [IN_NUM*STAGE_IN_W-1:0] level_bus;
                wire signed [OUT_NUM*STAGE_OUT_W-1:0] next_bus;
                wire level_valid,next_valid;
                wire [USER_W-1:0] level_user,next_user;
                if(gi == 0) begin
                    assign level_bus = product_bus;
                    assign level_valid = in_valid;
                    assign level_user = in_user;
                end else begin
                    assign level_bus = gen_levels[gi-1].next_bus;
                    assign level_valid = gen_levels[gi-1].next_valid;
                    assign level_user = gen_levels[gi-1].next_user;
                end
                adder_pipe #(
                    .IN_W(STAGE_IN_W),
                    .OUT_W(STAGE_OUT_W),
                    .IN_NUM(IN_NUM),
                    .OUT_NUM(OUT_NUM),
                    .USER_W(USER_W)
                ) u_pipe (
                    .clk(clk),
                    .rst_n(rst_n),
                    .in_valid(level_valid),
                    .in_user(level_user),
                    .product_bus(level_bus),
                    .sum_out(next_bus),
                    .out_valid(next_valid),
                    .out_user(next_user)
                );
                if(gi == LEVEL-1) begin
                    if (OUT_W >= STAGE_OUT_W) begin : gen_last_ext
                        assign sum_out = {{(OUT_W-STAGE_OUT_W){next_bus[STAGE_OUT_W-1]}}, next_bus[0 +: STAGE_OUT_W]};
                    end else begin : gen_last_trunc
                        assign sum_out = next_bus[0 +: OUT_W];
                    end
                    assign out_valid = next_valid;
                    assign out_user = next_user;
                end
            end
        end
    endgenerate
endmodule
