module psum_ram_cg #(  parameter DATA_WIDTH = 32,
  parameter ADDR_WIDTH = 7,
  parameter DEPTH      = 125
)
(
input wire  [ADDR_WIDTH-1:0] raddr,
input wire  [ADDR_WIDTH-1:0] waddr,
input wire  [DATA_WIDTH-1:0] wdata,
input wire clk,
input wire re,
input wire we,
output reg [DATA_WIDTH-1:0] rdata
);

  //----Code starts here: integrated by Robei-----
  reg [DATA_WIDTH-1:0] ram [0:DEPTH-1];
  
      integer i;
      initial begin
          for (i = 0; i < DEPTH; i = i + 1) begin
              ram[i] = {DATA_WIDTH{1'b0}}; 
          end
      end
  
      
      
      
      
      always @(posedge clk) begin
          if (re) begin
              rdata <= ram[raddr];
          end
      end
  
      
      
      
      always @(posedge clk) begin
          if (we) begin
              ram[waddr] <= wdata;
          end
      end
  
  
endmodule    //psum_ram_cg
