`timescale 1ns / 1ps

module mfcc10_wm8978_i2c_write #(
    parameter integer CLK_HZ = 50000000,
    parameter integer I2C_HZ = 10000
) (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       start,
    input  wire [6:0] reg_addr,
    input  wire [8:0] reg_data,
    output reg        busy,
    output reg        done,
    output reg        ack_error,
    output reg  [1:0] ack_error_byte,

    output wire       iic_scl,
    inout  wire       iic_sda
);

    localparam integer HALF_DIV = CLK_HZ / (I2C_HZ * 2);
    localparam integer DIV_W = 16;

    localparam [3:0] S_IDLE      = 4'd0;
    localparam [3:0] S_START_A   = 4'd1;
    localparam [3:0] S_START_B   = 4'd2;
    localparam [3:0] S_BIT_LOW   = 4'd3;
    localparam [3:0] S_BIT_HIGH  = 4'd4;
    localparam [3:0] S_ACK_LOW   = 4'd5;
    localparam [3:0] S_ACK_HIGH  = 4'd6;
    localparam [3:0] S_STOP_A    = 4'd7;
    localparam [3:0] S_STOP_B    = 4'd8;
    localparam [3:0] S_DONE      = 4'd9;

    reg [3:0] state;
    reg [DIV_W-1:0] div_count;
    reg [1:0] byte_idx;
    reg [2:0] bit_idx;
    reg [7:0] tx_byte;
    reg scl_r;
    reg sda_drive_low;
    reg [6:0] reg_addr_latched;
    reg [8:0] reg_data_latched;

    wire tick;

    assign tick = (div_count == (HALF_DIV - 1));
    assign iic_scl = scl_r;
    assign iic_sda = sda_drive_low ? 1'b0 : 1'bz;

    always @(*) begin
        case (byte_idx)
            2'd0: tx_byte = 8'h34;
            2'd1: tx_byte = {reg_addr_latched, reg_data_latched[8]};
            default: tx_byte = reg_data_latched[7:0];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_count <= {DIV_W{1'b0}};
        end else if (state == S_IDLE) begin
            div_count <= {DIV_W{1'b0}};
        end else if (tick) begin
            div_count <= {DIV_W{1'b0}};
        end else begin
            div_count <= div_count + 1'b1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            byte_idx         <= 2'd0;
            bit_idx          <= 3'd7;
            scl_r            <= 1'b1;
            sda_drive_low    <= 1'b0;
            busy             <= 1'b0;
            done             <= 1'b0;
            ack_error        <= 1'b0;
            ack_error_byte   <= 2'd3;
            reg_addr_latched <= 7'd0;
            reg_data_latched <= 9'd0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    scl_r         <= 1'b1;
                    sda_drive_low <= 1'b0;
                    busy          <= 1'b0;
                    if (start) begin
                        busy             <= 1'b1;
                        ack_error        <= 1'b0;
                        ack_error_byte   <= 2'd3;
                        reg_addr_latched <= reg_addr;
                        reg_data_latched <= reg_data;
                        byte_idx         <= 2'd0;
                        bit_idx          <= 3'd7;
                        state            <= S_START_A;
                    end
                end

                S_START_A: begin
                    if (tick) begin
                        scl_r         <= 1'b1;
                        sda_drive_low <= 1'b1;
                        state         <= S_START_B;
                    end
                end

                S_START_B: begin
                    if (tick) begin
                        scl_r <= 1'b0;
                        state <= S_BIT_LOW;
                    end
                end

                S_BIT_LOW: begin
                    if (tick) begin
                        scl_r         <= 1'b0;
                        sda_drive_low <= ~tx_byte[bit_idx];
                        state         <= S_BIT_HIGH;
                    end
                end

                S_BIT_HIGH: begin
                    if (tick) begin
                        scl_r <= 1'b1;
                        if (bit_idx == 3'd0) begin
                            state <= S_ACK_LOW;
                        end else begin
                            bit_idx <= bit_idx - 1'b1;
                            state   <= S_BIT_LOW;
                        end
                    end
                end

                S_ACK_LOW: begin
                    if (tick) begin
                        scl_r         <= 1'b0;
                        sda_drive_low <= 1'b0;
                        state         <= S_ACK_HIGH;
                    end
                end

                S_ACK_HIGH: begin
                    if (tick) begin
                        scl_r <= 1'b1;
                        if (iic_sda && !ack_error) begin
                            ack_error <= 1'b1;
                            ack_error_byte <= byte_idx;
                        end

                        if (byte_idx == 2'd2) begin
                            state <= S_STOP_A;
                        end else begin
                            byte_idx <= byte_idx + 1'b1;
                            bit_idx  <= 3'd7;
                            state    <= S_BIT_LOW;
                        end
                    end
                end

                S_STOP_A: begin
                    if (tick) begin
                        scl_r         <= 1'b0;
                        sda_drive_low <= 1'b1;
                        state         <= S_STOP_B;
                    end
                end

                S_STOP_B: begin
                    if (tick) begin
                        scl_r         <= 1'b1;
                        sda_drive_low <= 1'b0;
                        state         <= S_DONE;
                    end
                end

                S_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
