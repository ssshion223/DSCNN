**matrix_conv2d_stream — 详细说明**

**简介**: 本模块为顶层流式 KxK 卷积处理单元，将像素流转换为滑动窗口、对每个窗口按所有输出通道做卷积（MAC），并输出带有行/帧结束指示的结果。

**文件**: [matrix_conv2d_stream.v](matrix_conv2d_stream.v)

**主要参数（summary）**
- `DATA_W`：像素位宽
- `COEFF_W`：卷积系数量化位宽
- `ACC_W`：累加器位宽（输出位宽）
- `COL, ROW`：输入帧尺寸
- `K`：卷积核尺寸
- `OUT_CH`：输出通道数
- `MAC_PIPELINE`：MAC树是否进行流水线操作（1表示进行，0表示不进行，默认进行）

**端口概览**
- `clk, rst_n`：时钟与异步复位
- `in_valid, in_ready, in_pixel`：输入像素流（握手）
- `out_valid, out_ready, out_pixel_data`：输出结果流（握手）
- `out_pixel_end_line, out_pixel_end_frame`：输出像素的行结束 / 帧结束指示

**模块功能详述**
- 输入：当in_ready有效时，允许输入像素流入，`in_pixel` 是单个像素数据，输入顺序要求按照行优先，即从左到右、从上到下。
- 输出：当out_valid有效时，`out_pixel_data` 携带卷积结果，输出顺序同样按照行优先，对于该串行通道计算，有多通道时，每一个滑窗会依次处理，输出结果是该滑窗按照输出通道索引依次输出，`out_pixel_end_line` 和 `out_pixel_end_frame` 分别指示当前输出像素是否为行末或帧末。

**子模块与代码位置（关键实例）**
- 滑动窗口：`conv_sliding_window_kxk` — 在 [matrix_conv2d_stream.v](matrix_conv2d_stream.v) 中实例化为 `u_conv_sliding_window_kxk`，负责生成 `window_bus`。
- 系数存储：`conv_coeff_store` — 实例化为 `u_conv_coeff_store`，按 `mac_ch_idx` 读出 `coeff_bus`。
- MAC 计算：`conv_mac_tree_kxk` — 实例化为 `u_conv_mac_tree_kxk`，接收 `window_bus_hold` 与 `coeff_bus`，输出 `mac_sum`, `mac_valid`, `user_out`（携带行/帧结束信号）。
- 输出缓冲：`fwft_fifo_reg` — 实例化为 `u_fwft_fifo_reg`，将 `{user_out, mac_sum}` 写入 FIFO，输出到模块外部。

（请在仓库中打开 [matrix_conv2d_stream.v](matrix_conv2d_stream.v) 搜索上述实例名以定位具体行）

**关键寄存器与信号**
- `window_bus_hold`：在 `window_out_ready` 时锁存 `window_bus`，供 MAC 使用，避免数据竞态。
- `cac_trigger`：当 `window_out_valid` 为高时置位，触发一轮通道计算；当所有通道计算完成（`cac_all_ch`）时清零。
- `mac_ch_idx` / `mac_ch_idx_align`：遍历输出通道索引，用于从 `conv_coeff_store` 读系数并控制 `window_out_ready`。
- `col_counter` / `row_counter`：输出像素坐标计数器；`col_counter_wrap` / `row_counter_wrap` 用于产生行/帧结束标志。
- `fifo_almost_full`：来自 FIFO 的回压信号，`in_ready = window_in_ready && !fifo_almost_full` 实现上游回压。

**数据流与时序要点（文字版）**
1. 输入像素流经 `conv_sliding_window_kxk` 生成滑动窗口总线 `window_bus`，当 `window_out_valid` 发出时表示有有效窗口。
2. 第一次有效窗口到来将把 `cac_trigger` 置位；在计算期间，`mac_ch_idx` 从 0 开始循环到 `OUT_CH-1`，每个值对应一个输出通道的系数集合，`conv_coeff_store` 根据 `mac_ch_idx` 输出 `coeff_bus`。
3. 当 `mac_ch_idx==0` 且 `window_out_valid` 时模块允许锁存 `window_bus`（`window_out_ready`），把窗口数据写入 `window_bus_hold`，保证后续多通道 MAC 使用同一窗口数据。
4. `conv_mac_tree_kxk` 使用 `window_bus_hold` 与 `coeff_bus` 进行乘加，输出 `mac_sum` 与 `mac_valid`；`user_out` 携带 `pixel_end_line` / `pixel_end_frame` 标志以标记输出位置。
5. `fwft_fifo_reg` 将 `{user_out, mac_sum}` 入队；当 FIFO 几乎满时通过 `fifo_almost_full` 回压输入，防止上游溢出。

