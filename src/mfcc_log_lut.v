module mfcc_log_lut (
    input  wire [3:0]  addr,
    output reg  [15:0] y0_q15,
    output reg  [15:0] dy_q15
);

    reg [15:0] y0_rom [0:15];
    reg [15:0] dy_rom [0:15];

    initial begin
        y0_rom[0] = 16'h8000;  dy_rom[0] = 16'h0B32;
        y0_rom[1] = 16'h8B32;  dy_rom[1] = 16'h0A8E;
        y0_rom[2] = 16'h95C0;  dy_rom[2] = 16'h09FC;
        y0_rom[3] = 16'h9FBC;  dy_rom[3] = 16'h0979;
        y0_rom[4] = 16'hA935;  dy_rom[4] = 16'h0903;
        y0_rom[5] = 16'hB237;  dy_rom[5] = 16'h0897;
        y0_rom[6] = 16'hBACF;  dy_rom[6] = 16'h0835;
        y0_rom[7] = 16'hC304;  dy_rom[7] = 16'h07DC;
        y0_rom[8] = 16'hCAE0;  dy_rom[8] = 16'h078A;
        y0_rom[9] = 16'hD26A;  dy_rom[9] = 16'h073E;
        y0_rom[10] = 16'hD9A8;  dy_rom[10] = 16'h06F8;
        y0_rom[11] = 16'hE0A0;  dy_rom[11] = 16'h06B7;
        y0_rom[12] = 16'hE757;  dy_rom[12] = 16'h067B;
        y0_rom[13] = 16'hEDD2;  dy_rom[13] = 16'h0643;
        y0_rom[14] = 16'hF415;  dy_rom[14] = 16'h060E;
        y0_rom[15] = 16'hFA23;  dy_rom[15] = 16'h05DD;
    end

    always @(*) begin
        y0_q15 = y0_rom[addr];
        dy_q15 = dy_rom[addr];
    end

endmodule

