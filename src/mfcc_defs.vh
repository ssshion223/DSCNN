`ifndef MFCC_DEFS_VH
`define MFCC_DEFS_VH

`define MFCC_SAMPLE_RATE_HZ 16000
`define MFCC_FRAME_LEN      400
`define MFCC_FRAME_SHIFT    160
`define MFCC_INPUT_SAMPLES  16000
`define MFCC_FRAME_COUNT    98

`define MFCC_FIFO_W         17
`define MFCC_FIFO_DEPTH     1024
`define MFCC_FIFO_COUNT_W   10

`define MFCC_FFT_LEN        512
`define MFCC_FFT_HALF       256
`define MFCC_POWER_BINS     257
`define MFCC_POWER_ADDR_W   9

`define MFCC_NUM_FILTERS    40
`define MFCC_NUM_CEPS       40
`define MFCC_OUT_CEPS       40
`define MFCC_FILTER_IDX_W   6
`define MFCC_CEP_IDX_W      6
`define MFCC_DCT_ROM_ADDR_W 11

`define MFCC_SAMPLE_W       16
`define MFCC_FFT_W          28
`define MFCC_POWER_W        32
`define MFCC_LOG_W          16
`define MFCC_MEL_W          32
`define MFCC_DCT_OUT_W      24
`define MFCC_DCT_Q          14
`define MFCC_LN2_Q15        22713

`endif
