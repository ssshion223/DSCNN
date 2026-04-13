# DSCNN Module Call Relationship Summary

## 1. Top-Level Data Path

- `depthwiseConv2D`
  - Instantiates `matrix_conv2d_stream_parallel` as convolution front-end.
  - Instantiates `bias_process_wrapper` as post-processing back-end.
  - Connects stream handshakes (`valid/ready`) between both stages.

Dataflow:

`in_pixel` -> `matrix_conv2d_stream_parallel` -> `conv_out_pixel_data_bus` -> `bias_process_wrapper` -> `out_pixel`

## 2. Convolution Front-End Hierarchy

- `matrix_conv2d_stream_parallel`
  - Instantiates `conv_sliding_padding`
    - Instantiates `linebuf_bank_ram` (generate loop, multiple banks)
  - Instantiates `conv_coeff_store`
  - Instantiates `conv_mac_tree` in parallel for each output channel (`OUT_CH` lanes)
    - `conv_mac_tree` instantiates:
      - `conv_mult_products`
      - `conv_adder_tree`
        - PIPELINE=0 path: `conv_adder_tree_comb` -> `adder_comb`
        - PIPELINE=1 path: `conv_adder_tree_pipe` -> `adder_pipe`
  - Instantiates `fwft_fifo_reg` for output buffering and backpressure decoupling.

## 3. Post-Processing Hierarchy

- `bias_process_wrapper`
  - Instantiates `bias_memory` for per-group/per-channel bias fetch.
  - Instantiates `bias_quant_relu` for add + quantization + ReLU.
  - Instantiates `fwft_fifo_reg` for output stream buffering.

## 4. Testbench Coverage

- `tb_matrix_conv2d_stream_parallel`
  - DUT: `matrix_conv2d_stream_parallel`
  - Verifies convolution sums and frame flags against software-generated golden values.

- `tb_depthwiseConv2D`
  - DUT: `depthwiseConv2D`
  - Verifies full pipeline output (conv + bias + quant + ReLU) and frame flags.

## 5. Interface Convention

Across the design, modules mostly follow stream handshake semantics:

- Upstream sends data when `in_valid=1` and `in_ready=1`.
- Downstream consumes data when `out_valid=1` and `out_ready=1`.
- Frame boundary metadata is carried by sideband signals (`*_end_frame`, `*_end_all_frame`).
