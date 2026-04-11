module conv_coeff_store #(
    parameter integer COEFF_W      = 8,
    parameter integer K            = 3,
    parameter integer OUT_CH       = 1,
    parameter         INIT_FILE  = "",

    parameter integer COEFF_CH_W   = ((OUT_CH <= 1) ? 1 : $clog2(OUT_CH))
)(
    input  wire                                  clk,
    input  wire [COEFF_CH_W-1:0]                 coeff_rd_ch,
    output reg  signed [K*K*COEFF_W-1:0]         coeff_bus
);
    localparam integer WIN_SIZE      = K * K;
    localparam integer COEFF_TOTAL   = OUT_CH * WIN_SIZE;
    localparam integer COEFF_TOTAL_W = (COEFF_TOTAL <= 1) ? 1 : $clog2(COEFF_TOTAL);

    (*rom_style = "distributed" *)reg signed [COEFF_W-1:0] coeff_mem [0:COEFF_TOTAL-1];

    wire [31:0] coeff_rd_addr_full;
    wire [COEFF_TOTAL_W-1:0] coeff_rd_addr;

    integer i;

    assign coeff_rd_addr_full = coeff_rd_ch * WIN_SIZE;
    assign coeff_rd_addr      = coeff_rd_addr_full[COEFF_TOTAL_W-1:0];

  
    initial begin
		$readmemh(INIT_FILE, coeff_mem);
    end

    
    always @(posedge clk) begin
        for (i = 0; i < WIN_SIZE; i = i + 1) begin
            coeff_bus[(i+1)*COEFF_W-1 -: COEFF_W] <= coeff_mem[coeff_rd_addr + i];
        end
    end

 
 
endmodule
