module conv_mac_tree #(
    parameter integer DATA_W  = 8,
    parameter integer COEFF_W = 8,
    parameter integer ACC_W   = 16,
    parameter integer K_H     = 3,
    parameter integer K_W     = 3,
    parameter integer USER_W  = 1,
    parameter integer PIPELINE = 1 // 0: no adder pipeline, 1: full-registered pipeline
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          in_valid,
    input  wire [USER_W-1:0]             in_user,
    input  wire signed [K_H*K_W*DATA_W-1:0]   window_bus,
    input  wire signed [K_H*K_W*COEFF_W-1:0]  coeff_bus,
    output wire signed [ACC_W-1:0]        mac_sum,
    output wire                          out_valid,
    output wire [USER_W-1:0]             out_user
    
);
    wire signed [K_H*K_W*ACC_W-1:0] product_bus;

    conv_mult_products #(
        .DATA_W (DATA_W),
        .COEFF_W(COEFF_W),
        .ACC_W  (ACC_W),
        .K_H    (K_H),
        .K_W    (K_W)
    ) u_mults (
        .window_bus (window_bus),
        .coeff_bus  (coeff_bus),
        .product_bus(product_bus)
    );

    conv_adder_tree #(
        .ACC_W(ACC_W),
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

module conv_mult_products #(
    parameter integer DATA_W  = 8,
    parameter integer COEFF_W = 8,
    parameter integer ACC_W   = 16,
    parameter integer K_H     = 3,
    parameter integer K_W     = 3
)(
    input  wire signed [K_H*K_W*DATA_W-1:0]  window_bus,
    input  wire signed [K_H*K_W*COEFF_W-1:0] coeff_bus,
    output wire signed [K_H*K_W*ACC_W-1:0]   product_bus
);
    localparam integer WIN_SIZE = K_H * K_W;

    wire signed [DATA_W-1:0]  win_vec   [0:WIN_SIZE-1];
    wire signed [COEFF_W-1:0] coeff_vec [0:WIN_SIZE-1];

    genvar gi;

    generate
        for (gi = 0; gi < WIN_SIZE; gi = gi + 1) begin : gen_unpack_and_mul
            assign win_vec[gi]   = window_bus[gi*DATA_W +: DATA_W];
            assign coeff_vec[gi] = coeff_bus[gi*COEFF_W +: COEFF_W];
            assign product_bus[gi*ACC_W +: ACC_W] = $signed(win_vec[gi]) * $signed(coeff_vec[gi]);
        end
    endgenerate
endmodule

module conv_adder_tree #(
    parameter integer ACC_W = 16,
    parameter integer K_H   = 3,
    parameter integer K_W   = 3,
    parameter integer USER_W = 1,
    parameter integer PIPELINE = 1 // 0: combinational tree, 1: irregular full-registered pipeline
) (
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          in_valid,
    input  wire [USER_W-1:0]             in_user,
    input  wire signed [K_H*K_W*ACC_W-1:0]  product_bus,
    output reg  signed [ACC_W-1:0]      sum_out,
    output reg                           out_valid,
    output reg [USER_W-1:0]             out_user
);
    localparam integer WIN_SIZE = K_H * K_W;

    wire signed [ACC_W-1:0] comb_sum_out;
    wire signed [ACC_W-1:0] pipe_sum_out;
    wire                    pipe_out_valid;
    wire [USER_W-1:0]       pipe_out_user;

    generate
        if (PIPELINE == 0) begin : gen_comb_tree
            conv_adder_tree_comb #(
                .ACC_W(ACC_W),
                .N    (WIN_SIZE)
            ) u_comb_tree (
                .product_bus(product_bus),
                .sum_out    (comb_sum_out)
            );

            // prevent unused port warnings when PIPELINE == 0
            // reference clk/rst_n so synthesis recognizes ports as used
            wire _unused_clk = clk;
            wire _unused_rst_n = rst_n;

            always @(*) begin
                sum_out = comb_sum_out;
                out_valid = in_valid;
                out_user = in_user;
            end
        end else begin : gen_pipe_tree
            conv_adder_tree_pipe #(
                .ACC_W(ACC_W),
                .N    (WIN_SIZE),
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
                sum_out = pipe_sum_out;
                out_valid = pipe_out_valid;
                out_user = pipe_out_user;
            end
        end
    endgenerate

endmodule



module adder_comb #(
    parameter integer ACC_W = 16,
    parameter integer IN_NUM = 9,
    parameter integer OUT_NUM = 5
)(
    input  wire signed [IN_NUM*ACC_W-1:0]   product_bus,
    output wire signed [OUT_NUM*ACC_W-1:0]   sum_out
);
    generate
    genvar gi;
    for (gi = 0; gi < OUT_NUM; gi = gi + 1) begin
        if(gi < IN_NUM/2)  begin
            assign sum_out[gi*ACC_W +: ACC_W] =
                product_bus[(2*gi)*ACC_W +: ACC_W] +
                product_bus[(2*gi+1)*ACC_W +: ACC_W];
        end else if(gi == IN_NUM/2 && (IN_NUM % 2) == 1) begin
            assign sum_out[gi*ACC_W +: ACC_W] = product_bus[(IN_NUM-1)*ACC_W +: ACC_W];
        end else begin
            assign sum_out[gi*ACC_W +: ACC_W] = {ACC_W{1'b0}};
        end
    end
    endgenerate  
endmodule

module conv_adder_tree_comb #(
    parameter integer ACC_W = 16,
    parameter integer N     = 9
)(
    input  wire signed [N*ACC_W-1:0] product_bus,
    output wire signed [ACC_W-1:0]   sum_out
);
    localparam integer LEVEL = $clog2(N);
    generate
        if (LEVEL == 0) begin : gen_single
            assign sum_out = product_bus[0 +: ACC_W];
        end else begin : gen_pipe_levels
            genvar gi;
            for(gi=0;gi<LEVEL; gi=gi+1) begin : gen_levels
                localparam integer IN_NUM = (N + (1 << gi) - 1) >> gi;
                localparam integer OUT_NUM = (IN_NUM + 1) >> 1;
                wire signed [IN_NUM*ACC_W-1:0] level_bus;
                wire signed [OUT_NUM*ACC_W-1:0] next_bus;
                if(gi == 0) begin
                    assign level_bus = product_bus;
                end else begin
                    assign level_bus = gen_levels[gi-1].next_bus;
                end
                adder_comb #(
                    .ACC_W(ACC_W),
                    .IN_NUM(IN_NUM),
                    .OUT_NUM(OUT_NUM)
                ) u_pipe (
                    .product_bus(level_bus),
                    .sum_out(next_bus)
                );
                if(gi == LEVEL-1) begin
                    assign sum_out = next_bus[0 +: ACC_W];
                end
            end
        end
    endgenerate
endmodule


module adder_pipe #(
    parameter integer ACC_W = 16,
    parameter integer IN_NUM = 9,
    parameter integer OUT_NUM = 5,
    parameter integer USER_W = 1
)(
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             in_valid,
    input  wire [USER_W-1:0]                in_user,
    input  wire signed [IN_NUM*ACC_W-1:0]   product_bus,
    output reg signed [OUT_NUM*ACC_W-1:0]   sum_out,
    output reg                              out_valid,
    output reg [USER_W-1:0]                 out_user
);
    generate
    genvar gi;
    for (gi = 0; gi < OUT_NUM; gi = gi + 1) begin
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin
                sum_out[gi*ACC_W +: ACC_W] <= {ACC_W{1'b0}};
            end else if(gi < IN_NUM/2)  begin
                sum_out[gi*ACC_W +: ACC_W] <=
                product_bus[(2*gi)*ACC_W +: ACC_W] +
                product_bus[(2*gi+1)*ACC_W +: ACC_W];
            end else if(gi == IN_NUM/2 && (IN_NUM % 2) == 1) begin
                sum_out[gi*ACC_W +: ACC_W] <= product_bus[(IN_NUM-1)*ACC_W +: ACC_W];
            end else begin
                sum_out[gi*ACC_W +: ACC_W] <= {ACC_W{1'b0}};
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

module conv_adder_tree_pipe #(
    parameter integer ACC_W = 16,
    parameter integer N     = 9,
    parameter integer USER_W = 1
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        in_valid,
    input  wire [USER_W-1:0]           in_user,
    input  wire signed [N*ACC_W-1:0]  product_bus,
    output wire signed [ACC_W-1:0]    sum_out,
    output wire                       out_valid,
    output wire [USER_W-1:0]          out_user
);
    localparam integer LEVEL = $clog2(N);
    generate
        if (LEVEL == 0) begin : gen_single
            assign sum_out = product_bus[0 +: ACC_W];
            assign out_valid = in_valid;
            assign out_user = in_user;
        end else begin : gen_pipe_levels
            genvar gi;
            for(gi=0;gi<LEVEL; gi=gi+1) begin : gen_levels
                localparam integer IN_NUM = (N + (1 << gi) - 1) >> gi;
                localparam integer OUT_NUM = (IN_NUM + 1) >> 1;
                wire signed [IN_NUM*ACC_W-1:0] level_bus;
                wire signed [OUT_NUM*ACC_W-1:0] next_bus;
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
                    .ACC_W(ACC_W),
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
                    assign sum_out = next_bus[0 +: ACC_W];
                    assign out_valid = next_valid;
                    assign out_user = next_user;
                end
            end
        end
    endgenerate
endmodule
