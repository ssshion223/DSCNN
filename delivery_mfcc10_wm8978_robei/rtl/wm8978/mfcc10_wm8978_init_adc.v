`timescale 1ns / 1ps

module mfcc10_wm8978_init_adc #(
    parameter CLK_HZ = 50000000,
    parameter I2C_HZ = 10000
) (
    input  wire clk,
    input  wire rst_n,

    output wire iic_scl,
    inout  wire iic_sda,

    output reg  init_done,
    output reg  init_busy,
    output reg  init_error,
    output reg [4:0] fail_init_idx,
    output reg [1:0] fail_ack_byte
);

    localparam INIT_COUNT = 15;
    localparam DELAY_CYCLES = CLK_HZ / 100;

    localparam [2:0] S_DELAY      = 3'd0;
    localparam [2:0] S_LOAD       = 3'd1;
    localparam [2:0] S_WAIT_BUSY  = 3'd2;
    localparam [2:0] S_WAIT_DONE  = 3'd3;
    localparam [2:0] S_NEXT       = 3'd4;
    localparam [2:0] S_DONE       = 3'd5;

    reg [2:0] state;
    reg [23:0] delay_count;
    reg [4:0] init_idx;
    reg       write_start;
    reg [6:0] write_reg_addr;
    reg [8:0] write_reg_data;

    wire write_busy;
    wire write_done;
    wire write_ack_error;
    wire [1:0] write_ack_error_byte;

    always @(*) begin
        write_reg_addr = 7'd0;
        write_reg_data = 9'd0;

        case (init_idx)
            5'd0:  begin write_reg_addr = 7'd0;  write_reg_data = 9'h000; end
            5'd1:  begin write_reg_addr = 7'd1;  write_reg_data = 9'h01B; end
            5'd2:  begin write_reg_addr = 7'd2;  write_reg_data = 9'h1BF; end
            5'd3:  begin write_reg_addr = 7'd3;  write_reg_data = 9'h000; end
            5'd4:  begin write_reg_addr = 7'd4;  write_reg_data = 9'h050; end
            5'd5:  begin write_reg_addr = 7'd6;  write_reg_data = 9'h000; end
            5'd6:  begin write_reg_addr = 7'd10; write_reg_data = 9'h008; end
            5'd7:  begin write_reg_addr = 7'd14; write_reg_data = 9'h008; end
            5'd8:  begin write_reg_addr = 7'd44; write_reg_data = 9'h033; end
            5'd9:  begin write_reg_addr = 7'd45; write_reg_data = 9'h02E; end
            5'd10: begin write_reg_addr = 7'd46; write_reg_data = 9'h12E; end
            5'd11: begin write_reg_addr = 7'd47; write_reg_data = 9'h100; end
            5'd12: begin write_reg_addr = 7'd48; write_reg_data = 9'h100; end
            5'd13: begin write_reg_addr = 7'd49; write_reg_data = 9'h002; end
            5'd14: begin write_reg_addr = 7'd50; write_reg_data = 9'h000; end
            default: begin write_reg_addr = 7'd0; write_reg_data = 9'h000; end
        endcase
    end

    mfcc10_wm8978_i2c_write #(
        .CLK_HZ (CLK_HZ),
        .I2C_HZ (I2C_HZ)
    ) u_i2c_write (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (write_start),
        .reg_addr  (write_reg_addr),
        .reg_data  (write_reg_data),
        .busy      (write_busy),
        .done      (write_done),
        .ack_error (write_ack_error),
        .ack_error_byte (write_ack_error_byte),
        .iic_scl   (iic_scl),
        .iic_sda   (iic_sda)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_DELAY;
            delay_count   <= 24'd0;
            init_idx      <= 5'd0;
            write_start   <= 1'b0;
            init_done     <= 1'b0;
            init_busy     <= 1'b1;
            init_error    <= 1'b0;
            fail_init_idx <= 5'd31;
            fail_ack_byte <= 2'd3;
        end else begin
            write_start <= 1'b0;

            case (state)
                S_DELAY: begin
                    init_busy <= 1'b1;
                    if (delay_count == DELAY_CYCLES[23:0]) begin
                        delay_count <= 24'd0;
                        state       <= S_LOAD;
                    end else begin
                        delay_count <= delay_count + 1'b1;
                    end
                end

                S_LOAD: begin
                    write_start <= 1'b1;
                    state       <= S_WAIT_BUSY;
                end

                S_WAIT_BUSY: begin
                    if (write_busy) begin
                        state <= S_WAIT_DONE;
                    end
                end

                S_WAIT_DONE: begin
                    if (write_done) begin
                        if (write_ack_error) begin
                            init_error <= 1'b1;
                            if (!init_error) begin
                                fail_init_idx <= init_idx;
                                fail_ack_byte <= write_ack_error_byte;
                            end
                        end
                        state <= S_NEXT;
                    end
                end

                S_NEXT: begin
                    if (init_idx == (INIT_COUNT - 1)) begin
                        state <= S_DONE;
                    end else begin
                        init_idx <= init_idx + 1'b1;
                        state    <= S_LOAD;
                    end
                end

                S_DONE: begin
                    init_done <= 1'b1;
                    init_busy <= 1'b0;
                    state     <= S_DONE;
                end

                default: begin
                    state <= S_DELAY;
                end
            endcase
        end
    end

endmodule
