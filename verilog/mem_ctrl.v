module mem_ctrl(
    input clk,
    input rst,
    input start,
	input data_valid,
    input [7:0] data_in,
    
    output data_ready,
    output data_out
);
	reg [7:0] memA [0:1023];
    reg [7:0] memB [0:1023];
    reg sel_mem;
    reg [9:0] wr_addr;
    
    always @ (posedge clk) begin 
    		if (rst) begin
            wr_addr <= 0;
       	end         
        	else if (data_valid && data_ready) begin
            wr_addr <= wr_addr + 1;
            if 
        	end
    end
    
    