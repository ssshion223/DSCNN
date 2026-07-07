`ifndef MFCC10_DEFS_VH
`define MFCC10_DEFS_VH

`define MFCC10_SR               16000
`define MFCC10_DURATION_MS      1568
`define MFCC10_INPUT_SAMPLES    25088
`define MFCC10_FRAME_LEN        512
`define MFCC10_FRAME_SHIFT      512
`define MFCC10_FRAME_COUNT      49
`define MFCC10_FFT_LEN          512
`define MFCC10_POWER_BINS       257

`define MFCC10_SAMPLE_W         24
`define MFCC10_HANN_W           18
`define MFCC10_HANN_FRAC_W      17
`define MFCC10_FFT_IN_W         24
`define MFCC10_FFT_CORE_IN_W    24
`define MFCC10_FFT_IP_IN_W      16
`define MFCC10_FFT_OUT_W        36
`define MFCC10_FFT_TW_W         18
`define MFCC10_FFT_TW_FRAC_W    17
`define MFCC10_POWER_W          74
`define MFCC10_NUM_MELS         128
`define MFCC10_NUM_MFCC         10
`define MFCC10_MEL_IDX_W        7
`define MFCC10_MFCC_IDX_W       4
`define MFCC10_FRAME_IDX_W      6
`define MFCC10_MEL_WEIGHT_W     18
`define MFCC10_MEL_WEIGHT_FRAC_W 17
`define MFCC10_MEL_ACC_W        84
`define MFCC10_MEL_ROM_W        50
`define MFCC10_DCT_W            18
`define MFCC10_DCT_FRAC_W       17
`define MFCC10_DCT_ADDR_W       11
`define MFCC10_LOG_W            32
`define MFCC10_LOG_FRAC_W       16
`define MFCC10_DB_W             32
`define MFCC10_DB_FRAC_W        16
`define MFCC10_OUT_W            32

`define MFCC10_DB_COEFF_Q16     32'd197281
`define MFCC10_SAMPLE_LOG2_Q16  32'd3014656
`define MFCC10_TOP_DB_Q16       32'd5242880

`define MFCC10_FRAME_ADDR_W     9
`define MFCC10_FFT_ADDR_W       9
`define MFCC10_POWER_ADDR_W     9

`endif
