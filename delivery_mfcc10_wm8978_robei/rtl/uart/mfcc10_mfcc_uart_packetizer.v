`timescale 1ns / 1ps
module mfcc10_mfcc_uart_packetizer #(
    parameter BLOCK_ID_W = 16
) (
    input  wire                              clk,
    input  wire                              rst_n,

    input  wire                              coeff_valid,
    output wire                              coeff_ready,
    input  wire [6-1:0]    coeff_frame_index,
    input  wire [4-1:0]     coeff_index,
    input  wire signed [32-1:0]   coeff_data,
    input  wire                              block_done,

    output reg                               packet_valid,
    input  wire                              packet_ready,
    output reg  [7:0]                        packet_data,
    output wire                              packet_busy
);

    localparam S_COLLECT = 2'd0;
    localparam S_SEND    = 2'd1;

    localparam COEFF_COUNT  = 49 * 10;
    localparam HEADER_BYTES = 12;
    localparam PACKET_BYTES = HEADER_BYTES + (COEFF_COUNT * 4);
    localparam [15:0] FRAME_COUNT_U16 = 49;
    localparam [15:0] NUM_MFCC_U16    = 10;
    localparam [15:0] COEFF_COUNT_U16 = COEFF_COUNT;

    reg [1:0] state;
    reg [15:0] block_id;
    reg [11:0] send_index;
    reg signed [32-1:0] coeff_mem [0:COEFF_COUNT-1];

    wire coeff_fire;
    wire send_fire;
    wire [15:0] coeff_store_addr;
    wire [15:0] coeff_send_byte_index;
    wire [15:0] coeff_send_index;
    wire [1:0]  coeff_send_byte_sel;

    assign coeff_fire = coeff_valid && coeff_ready;
    assign send_fire  = packet_valid && packet_ready;
    assign coeff_ready = (state == S_COLLECT);
    assign packet_busy = (state == S_SEND);

    assign coeff_store_addr = (coeff_frame_index * 10) + coeff_index;
    assign coeff_send_byte_index = send_index - HEADER_BYTES;
    assign coeff_send_index = coeff_send_byte_index[15:2];
    assign coeff_send_byte_sel = coeff_send_byte_index[1:0];

    always @(*) begin
        packet_valid = (state == S_SEND);
        packet_data  = 8'd0;

        case (send_index)
            12'd0:  packet_data = 8'ha5;
            12'd1:  packet_data = 8'h10;
            12'd2:  packet_data = 8'h4d;
            12'd3:  packet_data = 8'h46;
            12'd4:  packet_data = block_id[7:0];
            12'd5:  packet_data = block_id[15:8];
            12'd6:  packet_data = FRAME_COUNT_U16[7:0];
            12'd7:  packet_data = FRAME_COUNT_U16[15:8];
            12'd8:  packet_data = NUM_MFCC_U16[7:0];
            12'd9:  packet_data = NUM_MFCC_U16[15:8];
            12'd10: packet_data = COEFF_COUNT_U16[7:0];
            12'd11: packet_data = COEFF_COUNT_U16[15:8];
            default: begin
                if (coeff_send_index < COEFF_COUNT) begin
                    case (coeff_send_byte_sel)
                        2'd0: packet_data = coeff_mem[coeff_send_index][7:0];
                        2'd1: packet_data = coeff_mem[coeff_send_index][15:8];
                        2'd2: packet_data = coeff_mem[coeff_send_index][23:16];
                        default: packet_data = coeff_mem[coeff_send_index][31:24];
                    endcase
                end
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_COLLECT;
            block_id   <= {BLOCK_ID_W{1'b0}};
            send_index <= 12'd0;
        end else begin
            case (state)
                S_COLLECT: begin
                    send_index <= 12'd0;

                    if (coeff_fire && (coeff_store_addr < COEFF_COUNT)) begin
                        coeff_mem[coeff_store_addr] <= coeff_data;
                    end

                    if (block_done) begin
                        state      <= S_SEND;
                        send_index <= 12'd0;
                    end
                end

                S_SEND: begin
                    if (send_fire) begin
                        if (send_index == (PACKET_BYTES - 1)) begin
                            send_index <= 12'd0;
                            block_id   <= block_id + 1'b1;
                            state      <= S_COLLECT;
                        end else begin
                            send_index <= send_index + 1'b1;
                        end
                    end
                end

                default: begin
                    state      <= S_COLLECT;
                    send_index <= 12'd0;
                end
            endcase
        end
    end

endmodule
