`timescale 1ns / 1ps

//==============================================================================
// Module: dw_pw_pingpong_ctrl
// Description:
//   Control-only module for dw_pw_pingpong.
//   Keeps state machine, ping-pong round counter and signal routing logic.
//==============================================================================
module dw_pw_pingpong_ctrl #(
    parameter PINGPONG_ROUNDS      = 3,
    parameter IN_DATA_W            = 8,
    parameter RAM_DATA_W           = 2048,
    parameter RAM_ADDR_W           = 8,
    parameter RAM_ONE_DATA_W       = 24,
    parameter OUT_PIXEL_WIDTH      = 8
)(
    input  wire                                clk,
    input  wire                                rst_n,

    input  wire                                start,
    input  wire                                in_valid,
    input  wire  [IN_DATA_W-1:0]               in_pixel,
    input  wire                                in_end_all_frame,
    input  wire                                out_ready,

    output wire                                busy,
    output wire                                in_ready,
    output wire                                out_valid,
    output wire  [OUT_PIXEL_WIDTH-1:0]         out_data,
    output wire                                out_end_frame,
    output wire                                out_end_all_frame,

    input  wire                                pw_conv_in_ready,
    input  wire                                pw_conv_end_frame,
    input  wire                                pw_conv_end_all_frame,
    input  wire                                pw_conv_ram_re,
    input  wire                                pw_conv_ram_we,
    input  wire  [RAM_ADDR_W-1:0]              pw_conv_ram_raddr,
    input  wire  [RAM_ADDR_W-1:0]              pw_conv_ram_waddr,
    input  wire  [RAM_DATA_W-1:0]              pw_conv_ram_wdata,

    input  wire  [RAM_DATA_W-1:0]              ram0_rdata,
    input  wire  [RAM_DATA_W-1:0]              ram1_rdata,

    input  wire                                ram_reader_busy,
    input  wire                                ram_reader_ram_re,
    input  wire  [RAM_ADDR_W-1:0]              ram_reader_ram_raddr,
    input  wire                                ram_reader_out_valid,
    input  wire  [RAM_ONE_DATA_W-1:0]          ram_reader_out_data,
    input  wire                                ram_reader_end_frame,
    input  wire                                ram_reader_end_all_frame,

    input  wire                                bias_process_in_ready,
    input  wire                                bias_process_out_valid,
    input  wire  [OUT_PIXEL_WIDTH-1:0]         bias_process_out_data,
    input  wire                                bias_process_out_end_frame,
    input  wire                                bias_process_out_end_all_frame,

    output reg                                 pw_conv_in_valid,
    output reg  [IN_DATA_W-1:0]               pw_conv_in_pixel,
    output reg                                 pw_conv_in_end_all_frame,
    output reg  [RAM_DATA_W-1:0]              pw_conv_ram_rdata,

    output reg                                 ram0_re,
    output reg  [RAM_ADDR_W-1:0]              ram0_raddr,
    output reg                                 ram0_we,
    output reg  [RAM_ADDR_W-1:0]              ram0_waddr,
    output reg  [RAM_DATA_W-1:0]              ram0_wdata,

    output reg                                 ram1_re,
    output reg  [RAM_ADDR_W-1:0]              ram1_raddr,
    output reg                                 ram1_we,
    output reg  [RAM_ADDR_W-1:0]              ram1_waddr,
    output reg  [RAM_DATA_W-1:0]              ram1_wdata,

    output reg                                 ram_reader_start,
    output reg  [RAM_DATA_W-1:0]              ram_reader_ram_rdata,
    output reg                                 ram_reader_out_ready,

    output reg                                 bias_process_in_valid,
    output reg  [RAM_ONE_DATA_W-1:0]          bias_process_in_data,
    output reg                                 bias_process_in_end_frame,
    output reg                                 bias_process_in_end_all_frame,
    output reg                                 bias_process_out_ready
);

    localparam integer PASS_TOTAL       = PINGPONG_ROUNDS + 1;
    localparam integer PASS_TOTAL_W     = (PASS_TOTAL <= 1) ? 1 : $clog2(PASS_TOTAL);
    localparam integer IS_EVEN_ROUND    = (PINGPONG_ROUNDS % 2 == 0);

    localparam  IDLE           = 3'b000,
                EXTERNAL_INPUT = 3'b001,
                RAM0_INPUT     = 3'b010,
                RAM1_INPUT     = 3'b011,
                OUT            = 3'b100;

    reg [2:0] state, next_state;
    reg [PASS_TOTAL_W-1:0] pass_cnt;
    reg dw_pw_cac_done;

    wire busy_int = (state != IDLE);
    wire start_fire = start && !busy_int;
    wire out_fire = out_valid && out_ready;
    wire switch_next_state = state != next_state;
    wire next_cac_start = dw_pw_cac_done && !ram_reader_busy;
    wire pass_cnt_wrap = (pass_cnt == PASS_TOTAL-1);

    assign busy = busy_int;
    assign in_ready = (state == EXTERNAL_INPUT) ? pw_conv_in_ready : 1'b0;
    assign out_valid = (state == OUT) ? bias_process_out_valid : 1'b0;
    assign out_data = (state == OUT) ? bias_process_out_data : {OUT_PIXEL_WIDTH{1'b0}};
    assign out_end_frame = (state == OUT) ? bias_process_out_end_frame : 1'b0;
    assign out_end_all_frame = (state == OUT) ? bias_process_out_end_all_frame : 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dw_pw_cac_done <= 1'b0;
        end else if (pw_conv_end_all_frame) begin
            dw_pw_cac_done <= 1'b1;
        end else if (switch_next_state) begin
            dw_pw_cac_done <= 1'b0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pass_cnt <= {PASS_TOTAL_W{1'b0}};
        end else if (state == IDLE) begin
            pass_cnt <= {PASS_TOTAL_W{1'b0}};
        end else if (next_cac_start) begin
            pass_cnt <= (pass_cnt_wrap) ? {PASS_TOTAL_W{1'b0}} : pass_cnt + 1'b1;
        end
    end

    always @(*) begin
        case (state)
            IDLE: begin
                if (start_fire)
                    next_state = EXTERNAL_INPUT;
                else
                    next_state = IDLE;
            end
            EXTERNAL_INPUT: begin
                if (next_cac_start)
                    next_state = RAM0_INPUT;
                else
                    next_state = EXTERNAL_INPUT;
            end
            RAM0_INPUT: begin
                if (next_cac_start) begin
                    if (pass_cnt_wrap)
                        next_state = OUT;
                    else
                        next_state = RAM1_INPUT;
                end else begin
                    next_state = RAM0_INPUT;
                end
            end
            RAM1_INPUT: begin
                if (next_cac_start) begin
                    if (pass_cnt_wrap)
                        next_state = OUT;
                    else
                        next_state = RAM0_INPUT;
                end else begin
                    next_state = RAM1_INPUT;
                end
            end
            OUT: begin
                if (bias_process_out_end_all_frame && out_fire)
                    next_state = IDLE;
                else
                    next_state = OUT;
            end
            default: next_state = IDLE;
        endcase
    end

    always @(*) begin
        case (state)
            EXTERNAL_INPUT: begin
                pw_conv_in_valid = in_valid;
                pw_conv_in_pixel = in_pixel;
                pw_conv_in_end_all_frame = in_end_all_frame;
                pw_conv_ram_rdata = ram0_rdata;
            end
            RAM0_INPUT: begin
                pw_conv_in_valid = bias_process_out_valid;
                pw_conv_in_pixel = bias_process_out_data;
                pw_conv_in_end_all_frame = bias_process_out_end_all_frame;
                pw_conv_ram_rdata = ram1_rdata;
            end
            RAM1_INPUT: begin
                pw_conv_in_valid = bias_process_out_valid;
                pw_conv_in_pixel = bias_process_out_data;
                pw_conv_in_end_all_frame = bias_process_out_end_all_frame;
                pw_conv_ram_rdata = ram0_rdata;
            end
            default: begin
                pw_conv_in_valid = 1'b0;
                pw_conv_in_pixel = {IN_DATA_W{1'b0}};
                pw_conv_in_end_all_frame = 1'b0;
                pw_conv_ram_rdata = {RAM_DATA_W{1'b0}};
            end
        endcase
    end

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

        case (state)
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

    always @(*) begin
        case (state)
            RAM0_INPUT: begin
                ram_reader_ram_rdata = ram0_rdata;
                ram_reader_out_ready = bias_process_in_ready;
            end
            RAM1_INPUT: begin
                ram_reader_ram_rdata = ram1_rdata;
                ram_reader_out_ready = bias_process_in_ready;
            end
            OUT: begin
                if (IS_EVEN_ROUND)
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
        if (!rst_n) begin
            ram_reader_start <= 1'b0;
        end else if (next_cac_start) begin
            ram_reader_start <= 1'b1;
        end else begin
            ram_reader_start <= 1'b0;
        end
    end

    always @(*) begin
        case (state)
            RAM0_INPUT, RAM1_INPUT: begin
                bias_process_in_valid = ram_reader_out_valid;
                bias_process_in_data = ram_reader_out_data;
                bias_process_in_end_frame = ram_reader_end_frame;
                bias_process_in_end_all_frame = ram_reader_end_all_frame;
                bias_process_out_ready = pw_conv_in_ready;
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
                bias_process_in_data = {RAM_ONE_DATA_W{1'b0}};
                bias_process_in_end_frame = 1'b0;
                bias_process_in_end_all_frame = 1'b0;
                bias_process_out_ready = 1'b0;
            end
        endcase
    end

endmodule
