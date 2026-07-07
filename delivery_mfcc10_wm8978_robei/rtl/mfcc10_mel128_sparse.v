`timescale 1ns / 1ps
module mfcc10_mel128_sparse #(
    parameter ROM_FILE = "rom/sparse_mel128_16000_512_q17.mem"
) (
    input  wire                               clk,
    input  wire                               rst_n,

    input  wire                               power_valid,
    output wire                               power_ready,
    input  wire [9-1:0]    power_index,
    input  wire [74-1:0]         power_data,
    input  wire                               power_last,

    output reg                                mel_valid,
    output reg  [7-1:0]       mel_index,
    output reg  [84-1:0]       mel_data,
    output reg                                frame_done
);

    localparam S_CLEAR       = 4'd0;
    localparam S_WAIT_POWER  = 4'd1;
    localparam S_ROM_WAIT0   = 4'd2;
    localparam S_ROM_WAIT1   = 4'd3;
    localparam S_ROM_WAIT2   = 4'd4;
    localparam S_MUL         = 4'd5;
    localparam S_ACC_READ    = 4'd6;
    localparam S_ACC_WRITE   = 4'd7;
    localparam S_NEXT_BIN    = 4'd8;
    localparam S_OUT_READ    = 4'd9;
    localparam S_OUT_SEND    = 4'd10;

    reg [3:0] state;

    (* ram_style = "block" *) reg [84-1:0] acc_a_ram [0:128-1];
    (* ram_style = "block" *) reg [84-1:0] acc_b_ram [0:128-1];

    reg [9-1:0] rom_addr;
    reg [74-1:0]      bin_power_r;
    reg                            bin_last_r;

    reg [7-1:0]    clear_idx;
    reg [7-1:0]    out_idx;
    reg [7-1:0]    mel_a_r;
    reg [18-1:0] weight_a_r;
    reg [7-1:0]    mel_b_r;
    reg [18-1:0] weight_b_r;
    reg [74-1:0]      contrib_a_r;
    reg [74-1:0]      contrib_b_r;
    reg [84-1:0]    acc_a_rd_data;
    reg [84-1:0]    acc_b_rd_data;

    wire [7-1:0]    mel_a;
    wire [18-1:0] weight_a_q17;
    wire [7-1:0]    mel_b;
    wire [18-1:0] weight_b_q17;

    wire [(74+18)-1:0] prod_a_wire;
    wire [(74+18)-1:0] prod_b_wire;
    wire [74-1:0] contrib_a_wire;
    wire [74-1:0] contrib_b_wire;
    wire power_fire;
    reg                             acc_a_rd_en;
    reg                             acc_b_rd_en;
    reg                             acc_a_wr_en;
    reg                             acc_b_wr_en;
    reg [7-1:0]     acc_a_rd_addr;
    reg [7-1:0]     acc_b_rd_addr;
    reg [7-1:0]     acc_a_wr_addr;
    reg [7-1:0]     acc_b_wr_addr;
    reg [84-1:0]     acc_a_wr_data;
    reg [84-1:0]     acc_b_wr_data;

    mfcc10_sparse_mel_rom #(
        .ROM_FILE(ROM_FILE)
    ) u_sparse_mel_rom (
        .clk           (clk),
        .addr          (rom_addr),
        .mel_a         (mel_a),
        .weight_a_q17  (weight_a_q17),
        .mel_b         (mel_b),
        .weight_b_q17  (weight_b_q17)
    );

    assign prod_a_wire = bin_power_r * weight_a_r;
    assign prod_b_wire = bin_power_r * weight_b_r;
    assign contrib_a_wire = (prod_a_wire + ({{(74){1'b0}}, 1'b1} << (17 - 1)))
                            >> 17;
    assign contrib_b_wire = (prod_b_wire + ({{(74){1'b0}}, 1'b1} << (17 - 1)))
                            >> 17;
    assign power_ready = (state == S_WAIT_POWER);
    assign power_fire  = power_valid && power_ready;

    always @(*) begin
        acc_a_rd_en   = 1'b0;
        acc_b_rd_en   = 1'b0;
        acc_a_wr_en   = 1'b0;
        acc_b_wr_en   = 1'b0;
        acc_a_rd_addr = {7{1'b0}};
        acc_b_rd_addr = {7{1'b0}};
        acc_a_wr_addr = {7{1'b0}};
        acc_b_wr_addr = {7{1'b0}};
        acc_a_wr_data = {84{1'b0}};
        acc_b_wr_data = {84{1'b0}};

        case (state)
            S_CLEAR: begin
                acc_a_wr_en   = 1'b1;
                acc_b_wr_en   = 1'b1;
                acc_a_wr_addr = clear_idx;
                acc_b_wr_addr = clear_idx;
            end

            S_ACC_READ: begin
                acc_a_rd_en   = 1'b1;
                acc_b_rd_en   = 1'b1;
                acc_a_rd_addr = mel_a_r;
                acc_b_rd_addr = mel_b_r;
            end

            S_ACC_WRITE: begin
                if (weight_a_r != {18{1'b0}}) begin
                    acc_a_wr_en   = 1'b1;
                    acc_a_wr_addr = mel_a_r;
                    acc_a_wr_data = acc_a_rd_data +
                                    {{(84-74){1'b0}}, contrib_a_r};
                end

                if (weight_b_r != {18{1'b0}}) begin
                    acc_b_wr_en   = 1'b1;
                    acc_b_wr_addr = mel_b_r;
                    acc_b_wr_data = acc_b_rd_data +
                                    {{(84-74){1'b0}}, contrib_b_r};
                end
            end

            S_OUT_READ: begin
                acc_a_rd_en   = 1'b1;
                acc_b_rd_en   = 1'b1;
                acc_a_rd_addr = out_idx;
                acc_b_rd_addr = out_idx;
            end
        endcase
    end

    always @(posedge clk) begin
        if (acc_a_wr_en) begin
            acc_a_ram[acc_a_wr_addr] <= acc_a_wr_data;
        end

        if (acc_a_rd_en) begin
            acc_a_rd_data <= acc_a_ram[acc_a_rd_addr];
        end
    end

    always @(posedge clk) begin
        if (acc_b_wr_en) begin
            acc_b_ram[acc_b_wr_addr] <= acc_b_wr_data;
        end

        if (acc_b_rd_en) begin
            acc_b_rd_data <= acc_b_ram[acc_b_rd_addr];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_CLEAR;
            rom_addr         <= {9{1'b0}};
            bin_power_r      <= {74{1'b0}};
            bin_last_r       <= 1'b0;
            clear_idx        <= {7{1'b0}};
            out_idx          <= {7{1'b0}};
            mel_a_r          <= {7{1'b0}};
            weight_a_r       <= {18{1'b0}};
            mel_b_r          <= {7{1'b0}};
            weight_b_r       <= {18{1'b0}};
            contrib_a_r      <= {74{1'b0}};
            contrib_b_r      <= {74{1'b0}};
            mel_valid        <= 1'b0;
            mel_index        <= {7{1'b0}};
            mel_data         <= {84{1'b0}};
            frame_done       <= 1'b0;
        end else begin
            mel_valid  <= 1'b0;
            frame_done <= 1'b0;

            case (state)
                S_CLEAR: begin
                    if (clear_idx == (128 - 1)) begin
                        clear_idx <= {7{1'b0}};
                        state     <= S_WAIT_POWER;
                    end else begin
                        clear_idx <= clear_idx + 1'b1;
                    end
                end

                S_WAIT_POWER: begin
                    if (power_fire) begin
                        rom_addr    <= power_index;
                        bin_power_r <= power_data;
                        bin_last_r  <= power_last;
                        state       <= S_ROM_WAIT0;
                    end
                end

                S_ROM_WAIT0: begin
                    state <= S_ROM_WAIT1;
                end

                S_ROM_WAIT1: begin
                    state <= S_ROM_WAIT2;
                end

                S_ROM_WAIT2: begin
                    mel_a_r     <= mel_a;
                    weight_a_r  <= weight_a_q17;
                    mel_b_r     <= mel_b;
                    weight_b_r  <= weight_b_q17;
                    state       <= S_MUL;
                end

                S_MUL: begin
                    contrib_a_r <= contrib_a_wire;
                    contrib_b_r <= contrib_b_wire;
                    state       <= S_ACC_READ;
                end

                S_ACC_READ: begin
                    state <= S_ACC_WRITE;
                end

                S_ACC_WRITE: begin
                    state <= S_NEXT_BIN;
                end

                S_NEXT_BIN: begin
                    if (bin_last_r) begin
                        out_idx <= {7{1'b0}};
                        state   <= S_OUT_READ;
                    end else begin
                        state <= S_WAIT_POWER;
                    end
                end

                S_OUT_READ: begin
                    state <= S_OUT_SEND;
                end

                S_OUT_SEND: begin
                    mel_valid <= 1'b1;
                    mel_index <= out_idx;
                    mel_data  <= acc_a_rd_data + acc_b_rd_data;

                    if (out_idx == (128 - 1)) begin
                        frame_done <= 1'b1;
                        out_idx    <= {7{1'b0}};
                        clear_idx  <= {7{1'b0}};
                        state      <= S_CLEAR;
                    end else begin
                        out_idx <= out_idx + 1'b1;
                        state   <= S_OUT_READ;
                    end
                end

                default: begin
                    state <= S_CLEAR;
                end
            endcase
        end
    end

endmodule




