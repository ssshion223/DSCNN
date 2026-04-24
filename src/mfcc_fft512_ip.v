`include "mfcc_defs.vh"

// Vivado 512 点 FFT IP 的薄封装。
// 负责把工程内部的任务式控制转换成 IP 使用的 AXI-Stream 接口。
module mfcc_fft512_ip #(
    parameter IN_W      = `MFCC_SAMPLE_W,
    parameter FFT_W     = `MFCC_FFT_W,
    parameter POINTS    = `MFCC_FFT_LEN,
    parameter PAD_OUT_W = 32
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      start,
    input  wire                      in_valid,
    input  wire [IN_W-1:0]           in_re,
    input  wire [IN_W-1:0]           in_im,
    input  wire                      in_last,
    output wire                      in_ready,
    output reg                       busy,
    output reg                       done,
    output wire                      out_valid,
    output wire [`MFCC_POWER_ADDR_W-1:0] out_index,
    output wire [FFT_W-1:0]          out_re,
    output wire [FFT_W-1:0]          out_im,
    output wire                      out_last
);

    // Unscaled FFT 输出位宽会随 log2(N) 增长，并保留符号裕量。
    localparam integer LOG2_N      = 9;
    localparam integer IP_OUT_W    = IN_W + LOG2_N + 1;
    localparam integer PAD_IN_W    = ((IN_W + 7) / 8) * 8;
    localparam integer AXIS_IN_W   = 2 * PAD_IN_W;
    localparam integer AXIS_OUT_W  = 2 * PAD_OUT_W;
    localparam [7:0]   FFT_CFG_FWD = 8'b0000_0001;

    // S_CFG 负责发送配置字。
    // S_GAP 在配置后额外空几拍，再开始送流数据。
    localparam S_IDLE   = 2'd0;
    localparam S_CFG    = 2'd1;
    localparam S_GAP    = 2'd2;
    localparam S_STREAM = 2'd3;

    reg [1:0]                  state;
    reg [2:0]                  cfg_gap_cnt;
    reg [`MFCC_POWER_ADDR_W-1:0] input_count;
    reg [`MFCC_POWER_ADDR_W-1:0] output_count;
    reg                        cfg_valid;

    wire                  s_axis_config_tready;
    wire                  s_axis_data_tready;
    wire                  m_axis_data_tvalid;
    wire                  m_axis_data_tlast;
    wire [AXIS_OUT_W-1:0] m_axis_data_tdata;

    wire                  cfg_hs;
    wire                  send_hs;
    wire                  recv_hs;

    wire [PAD_IN_W-1:0]  send_re_pad;
    wire [PAD_IN_W-1:0]  send_im_pad;
    wire [AXIS_IN_W-1:0] s_axis_data_tdata;
    wire [PAD_OUT_W-1:0] recv_re_pad;
    wire [PAD_OUT_W-1:0] recv_im_pad;
    wire [IP_OUT_W-1:0]  out_re_raw;
    wire [IP_OUT_W-1:0]  out_im_raw;

    // 先把输入样本符号扩展到按字节对齐的 AXI 数据宽度。
    assign send_re_pad       = {{(PAD_IN_W-IN_W){in_re[IN_W-1]}}, in_re};
    assign send_im_pad       = {{(PAD_IN_W-IN_W){in_im[IN_W-1]}}, in_im};
    assign s_axis_data_tdata = {send_im_pad, send_re_pad};

    assign cfg_hs   = cfg_valid && s_axis_config_tready;
    assign send_hs  = (state == S_STREAM) && in_valid && s_axis_data_tready;
    assign recv_hs  = m_axis_data_tvalid;
    assign in_ready = (state == S_STREAM) && s_axis_data_tready;

    assign recv_re_pad = m_axis_data_tdata[PAD_OUT_W-1:0];
    assign recv_im_pad = m_axis_data_tdata[AXIS_OUT_W-1:PAD_OUT_W];
    assign out_re_raw  = recv_re_pad[IP_OUT_W-1:0];
    assign out_im_raw  = recv_im_pad[IP_OUT_W-1:0];

    // 输出侧恒定 ready，因此只看 valid 即可。
    assign out_valid = m_axis_data_tvalid;
    assign out_last  = m_axis_data_tlast;
    assign out_index = output_count;
    assign out_re    = {{(FFT_W-IP_OUT_W){out_re_raw[IP_OUT_W-1]}}, out_re_raw};
    assign out_im    = {{(FFT_W-IP_OUT_W){out_im_raw[IP_OUT_W-1]}}, out_im_raw};

    xfft_gap_512 u_xfft_gap_512 (
        .aclk                (clk),
        .aresetn             (rst_n),
        .s_axis_config_tdata (FFT_CFG_FWD),
        .s_axis_config_tvalid(cfg_valid),
        .s_axis_config_tready(s_axis_config_tready),
        .s_axis_data_tdata   (s_axis_data_tdata),
        .s_axis_data_tvalid  ((state == S_STREAM) && in_valid),
        .s_axis_data_tready  (s_axis_data_tready),
        .s_axis_data_tlast   (in_last),
        .m_axis_data_tdata   (m_axis_data_tdata),
        .m_axis_data_tvalid  (m_axis_data_tvalid),
        .m_axis_data_tready  (1'b1),
        .m_axis_data_tlast   (m_axis_data_tlast)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            cfg_gap_cnt  <= 3'd0;
            input_count  <= {`MFCC_POWER_ADDR_W{1'b0}};
            output_count <= {`MFCC_POWER_ADDR_W{1'b0}};
            cfg_valid    <= 1'b0;
            busy         <= 1'b0;
            done         <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    // 启动一次新的 FFT 任务。
                    if (start) begin
                        state        <= S_CFG;
                        cfg_valid    <= 1'b1;
                        cfg_gap_cnt  <= 3'd0;
                        input_count  <= {`MFCC_POWER_ADDR_W{1'b0}};
                        output_count <= {`MFCC_POWER_ADDR_W{1'b0}};
                        busy         <= 1'b1;
                    end
                end

                S_CFG: begin
                    // 等待配置字真正被 FFT IP 接收。
                    if (cfg_hs) begin
                        cfg_valid   <= 1'b0;
                        cfg_gap_cnt <= 3'd4;
                        state       <= S_GAP;
                    end
                end

                S_GAP: begin
                    // 在第一点输入前预留几拍保护间隔。
                    if (cfg_gap_cnt == 3'd1) begin
                        cfg_gap_cnt <= 3'd0;
                        state       <= S_STREAM;
                    end else begin
                        cfg_gap_cnt <= cfg_gap_cnt - 1'b1;
                    end
                end

                S_STREAM: begin
                    // 输入接收计数和输出发射计数分别维护。
                    if (send_hs) begin
                        input_count <= input_count + 1'b1;
                    end

                    if (recv_hs) begin
                        // m_axis_data_tlast 表示这一帧 FFT 输出结束。
                        if (m_axis_data_tlast) begin
                            state        <= S_IDLE;
                            busy         <= 1'b0;
                            done         <= 1'b1;
                            output_count <= {`MFCC_POWER_ADDR_W{1'b0}};
                        end else begin
                            output_count <= output_count + 1'b1;
                        end
                    end
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
