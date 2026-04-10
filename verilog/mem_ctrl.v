module mem_ctrl(
    input   clk,
    input   rst,

    //data description
    input   [3:0]   data_width,
    input   [5:0]   data_height,
    input   [5:0]   data_channel,
    input   [7:0]   data_single_in,

    //in-out interface
    input   data_valid_in,
    input   data_ready_out,             //output channel handshaking signal
    output  data_ready_in,              //input channel handshaking signal
    output  data_valid_out
);
	reg [7:0]   memA    [0:1023];       //buffer A
    reg [7:0]   memB    [0:1023];       //buffer B
    reg [7:0]   weight  [0:7999];
    reg [9:0]   wr_addr;                //write address
    reg [9:0]   rd_addr;
    reg [12:0]  rd_length;
    wire [12:0]  data_length;            //maximum input length 25x5x64=8000
    reg signed [31:0] bias [0:63];
    reg wr_sel_mem;                      //select buffer A0/B1
    reg rd_sel_mem;                      //select buffer A0/B1
    reg memA_full;                      
    reg memB_full;

    assign data_length = data_height * data_width * data_channel;
    assign data_ready_in = ~ (memA_full & memB_full);

    always @ (posedge clk or posedge rst) begin        //write data to one buffer
    		if (rst) begin
                wr_addr <= 0;
                done <= 0;
                wr_sel_mem <= 0;
                memA_full <= 0;
                memB_full <= 0;
       	    end else if (data_valid_in && data_ready_in) begin
                if (wr_sel_mem == 0 && !memA_full)
                    memA[wr_addr] <= data_single_in;
                else if (wr_sel_mem == 1 && !memB_full)
                    memB[wr_addr] <= data_single_in;

                if (wr_addr == data_length - 1) begin
                    wr_sel_mem <= ~wr_sel_mem;
                    wr_addr <= 0;
                    rd_length <= data_length;
                    if (wr_sel_mem == 0) begin
                        memA_full <= 1;
                    end else if (wr_sel_mem == 1) begin
                        memB_full <= 1;
                    end
                end else
                    wr_addr <= wr_addr + 1;
        	end
    end

    always @ (posedge clk or posedge rst) begin
        if (rst) begin
            data_valid_out <= 0;
            rd_sel_mem <= 0;
        end
        if (!memA_full && !memB_full)
            data_valid_out <= 0;
        else if (memA_full && !memB_full) begin
            data_valid_out <= 1;
            rd_sel_mem <= 0;
        end
        else if (!memA_full && memB_full) begin
            rd_sel_mem <= 1;

        end

        else if (memA_full && memB_full) begin
            data_valid_out <= 1;
        end
    end

    always @ (posedge clk or posedge rst) begin        //read data from buffer
        if (rst) begin
            rd_addr <= 0;
        end else if (memA_full == 1 && data_ready_out && data_valid_out && rd_sel_mem == 0) begin
            data_out <= memA[rd_addr];
            rd_addr <= rd_addr + 1;
        end else if (memB_full == 1 && data_ready_out && data_valid_out && rd_sel_mem == 1) begin
            data_out <= memB[rd_addr];
            rd_addr <= rd_addr + 1;
        end

        if (rd_addr == rd_length - 1) begin
            if (rd_sel_mem == 0) begin
                memA_full <= 0;
            end else if (rd_sel_mem == 1) begin
                memB_full <= 0;
            end
    end
endmodule
11111111111111