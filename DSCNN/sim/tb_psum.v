`timescale 1ns / 1ps

module tb_psum;

    // ==================== 参数配置 ====================
    localparam integer MULT_WIDTH      = 16;
    localparam integer PSUM_WIDTH      = 22;
    localparam integer CHANNELS        = 16;
    localparam integer SEGMENTS        = 1;
    localparam integer PIXEL_DEPTH     = 4;
    localparam integer IN_FRAME_SIZE   = 4;
    localparam integer RAM_ADDR_WIDTH = 4;

    localparam integer SEG_CHANS       = CHANNELS / SEGMENTS;
    localparam integer RAM_DATA_WIDTH = SEG_CHANS * PSUM_WIDTH;
    localparam integer RAM_DEPTH       = PIXEL_DEPTH * SEGMENTS;

    // ==================== 接口信号 ====================
    reg clk;
    reg rst_n;
    reg in_valid;
    wire in_ready;
    reg [CHANNELS*MULT_WIDTH-1:0] mult_in;
    
    wire ram_re;
    wire [RAM_ADDR_WIDTH-1:0] ram_raddr;
    wire [RAM_DATA_WIDTH-1:0] ram_rdata;
    reg end_frame;
    reg end_all_frame;
    wire ram_we;
    wire [RAM_ADDR_WIDTH-1:0] ram_waddr;
    wire [RAM_DATA_WIDTH-1:0] ram_wdata;

    reg [RAM_DATA_WIDTH-1:0] expected_ram [0:RAM_DEPTH-1];
    integer errors;
    integer i, frame_idx, pixel_idx;

    // ==================== 模块例化 ====================
    pw_sum #(
        .MULT_WIDTH(MULT_WIDTH),
        .PSUM_WIDTH(PSUM_WIDTH),
        .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
        .CHANNELS(CHANNELS),
        .SEGMENTS(SEGMENTS),
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .IN_FRAME_SIZE(IN_FRAME_SIZE)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .in_valid(in_valid),
        .in_ready(in_ready),
        .mult_in(mult_in),
        .ram_re(ram_re),
        .ram_raddr(ram_raddr),
        .ram_rdata(ram_rdata),
        .ram_we(ram_we),
        .ram_waddr(ram_waddr),
        .ram_wdata(ram_wdata)
    );

    psum_ram #(
        .DATA_WIDTH(RAM_DATA_WIDTH),
        .ADDR_WIDTH(RAM_ADDR_WIDTH),
        .DEPTH(RAM_DEPTH)
    ) u_psum_ram (
        .clk(clk),
        .re(ram_re),
        .raddr(ram_raddr),
        .rdata(ram_rdata),
        .we(ram_we),
        .waddr(ram_waddr),
        .wdata(ram_wdata)
    );

    // ==================== 时钟与系统控制 ====================
    always #5 clk = ~clk;

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        in_valid = 1'b0;
        mult_in = {CHANNELS*MULT_WIDTH{1'b0}};
        end_frame = 1'b0;
        end_all_frame = 1'b0;
        errors = 0;

        // 初始化计分板
        for (i = 0; i < RAM_DEPTH; i = i + 1) begin
            expected_ram[i] = {RAM_DATA_WIDTH{1'b0}};
        end

        // // 初始化外接 RAM，避免首轮时序观测读到未初始化 X。
        // for (i = 0; i < RAM_DEPTH; i = i + 1) begin
        //     u_psum_ram.ram[i] = {RAM_DATA_WIDTH{1'b0}};
        // end

        $display("\n===============================================");
        $display("[TB] Scoreboard Testbench Start");
        $display("===============================================\n");

        // 复位
        repeat (6) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // 主循环：喂入各帧数据
        for (frame_idx = 0; frame_idx < IN_FRAME_SIZE; frame_idx = frame_idx + 1) begin
            send_frame(frame_idx);
            repeat (2) @(posedge clk);
        end

        // 留出充分的时间让流水线排空，将所有段完全写回 RAM
        repeat (20) @(posedge clk); 

        // 最终查账
        readback_final_ram();

        // 结束控制
        @(negedge clk);
        end_all_frame = 1'b1;
        @(negedge clk);
        end_all_frame = 1'b0;

        if (errors == 0) begin
            $display("[TB][PASS] ALL MATCH! tb_psum PASSED beautifully!");
        end else begin
            $display("[TB][FAIL] tb_psum FAILED, errors=%0d", errors);
        end

        #20;
        $stop;
    end

    // ==================== 激励任务 ====================
    task send_frame;
        input integer frame_index;
        integer wait_guard;
    begin
        for (pixel_idx = 0; pixel_idx < PIXEL_DEPTH; pixel_idx = pixel_idx + 1) begin
            send_pixel(frame_index, pixel_idx);
        end

        // 等待当前帧最后一个像素的 4 段处理完全结束，再打 end_frame。
        wait_guard = 0;
        while (dut.processing === 1'b1) begin
            @(posedge clk);
            wait_guard = wait_guard + 1;
            if (wait_guard > 64) begin
                $display("[TB][FAIL] frame=%0d wait processing done timeout", frame_index);
                errors = errors + 1;
                disable send_frame;
            end
        end

        @(negedge clk);
        end_frame = 1'b1;
        @(negedge clk);
        end_frame = 1'b0;
    end
    endtask

    task send_pixel;
        input integer frame_index;
        input integer pixel_index;
        integer ch;
        integer wait_guard;
        reg [CHANNELS*MULT_WIDTH-1:0] pixel_bus;
    begin
        // 1. 生成当前像素的总线数据
        pixel_bus = {CHANNELS*MULT_WIDTH{1'b0}};
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin
            if ((frame_index % 2) == 0) begin
                pixel_bus[ch*MULT_WIDTH +: MULT_WIDTH] = $signed(frame_index * 64 + pixel_index * 8 + ch + 1);
            end else begin
                pixel_bus[ch*MULT_WIDTH +: MULT_WIDTH] = -$signed(frame_index * 48 + pixel_index * 6 + ch + 1);
            end
        end

        @(negedge clk);
        mult_in <= pixel_bus;
        in_valid <= 1'b1;

        // 3. 保持数据不变，直到模块真正 ready 并完成握手。
        wait_guard = 0;
        while (in_ready !== 1'b1) begin
            @(posedge clk);
            wait_guard = wait_guard + 1;
            if (wait_guard > 64) begin
                $display("[TB][FAIL] frame=%0d pixel=%0d wait in_ready timeout", frame_index, pixel_index);
                errors = errors + 1;
                disable send_pixel;
            end
        end

        @(posedge clk);
        in_valid <= 1'b0;
        update_expected_ram(frame_index, pixel_index, pixel_bus); // 调用计分板计算
    end
    endtask

    // ==================== 计分板核心：基于输入的纯软件累加推算 ====================
    task update_expected_ram;
        input integer f_idx;
        input integer p_idx;
        input [CHANNELS*MULT_WIDTH-1:0] p_bus;
        
        integer seg, ch;
        reg [RAM_DATA_WIDTH-1:0] old_data, new_data;
        reg signed [MULT_WIDTH-1:0] m_val;
        reg signed [PSUM_WIDTH-1:0] p_val;
    begin
        for (seg = 0; seg < SEGMENTS; seg = seg + 1) begin
            // 查首帧状态：如果第一帧，旧数据为0；否则从计分板中取历史数据
            old_data = (f_idx == 0) ? {RAM_DATA_WIDTH{1'b0}} : expected_ram[p_idx * SEGMENTS + seg];
            new_data = {RAM_DATA_WIDTH{1'b0}};
            
            // 按通道累加
            for (ch = 0; ch < SEG_CHANS; ch = ch + 1) begin
                m_val = p_bus[(seg*SEG_CHANS + ch)*MULT_WIDTH +: MULT_WIDTH];
                p_val = old_data[ch*PSUM_WIDTH +: PSUM_WIDTH];
                new_data[ch*PSUM_WIDTH +: PSUM_WIDTH] = $signed(m_val) + $signed(p_val);
            end
            
            // 更新计分板
            expected_ram[p_idx * SEGMENTS + seg] = new_data;
        end
    end
    endtask

    // ==================== 终局对账 ====================
    task readback_final_ram;
        integer addr;
        reg [RAM_DATA_WIDTH-1:0] exp_val;
        reg [RAM_DATA_WIDTH-1:0] got_val;
    begin
        $display("[TB][READBACK] --------- Final RAM Check Start ---------");
        for (addr = 0; addr < RAM_DEPTH; addr = addr + 1) begin
            exp_val = expected_ram[addr];
            got_val = u_psum_ram.ram[addr];
            
            if (got_val !== exp_val) begin
                $display("[TB][FAIL] Mismatch! addr=%0d | expected=%h | got=%h", addr, exp_val, got_val);
                errors = errors + 1;
            end else begin
                // 可选打印，如果你想看每个地址算出来对不对
                $display("[TB][PASS] addr=%0d | data=%h", addr, got_val);
            end
        end
        $display("[TB][READBACK] --------- Final RAM Check Done  ---------");
    end
    endtask

endmodule