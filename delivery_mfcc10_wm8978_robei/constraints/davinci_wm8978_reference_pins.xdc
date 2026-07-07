# MFCC10 + WM8978 Robei transfer reference pins.
# This file is a pin reference for the WM8978 capture side only.
# The MFCC coefficient outputs are intended to be connected to downstream
# Robei modules, not directly constrained as board pins.

set_property -dict {PACKAGE_PIN R4 IOSTANDARD LVCMOS33} [get_ports sys_clk]
create_clock -period 20.000 -name sys_clk [get_ports sys_clk]

set_property -dict {PACKAGE_PIN U2 IOSTANDARD LVCMOS33} [get_ports sys_rst_n]

set_property -dict {PACKAGE_PIN J17 IOSTANDARD LVCMOS33} [get_ports wm8978_sdb]
set_property -dict {PACKAGE_PIN K17 IOSTANDARD LVCMOS33} [get_ports wm8978_mclka]
set_property -dict {PACKAGE_PIN L15 IOSTANDARD LVCMOS33} [get_ports wm8978_scka]
set_property -dict {PACKAGE_PIN L14 IOSTANDARD LVCMOS33} [get_ports wm8978_fsa]
set_property -dict {PACKAGE_PIN M16 IOSTANDARD LVCMOS33 PULLUP true} [get_ports wm8978_iic_sda]
set_property -dict {PACKAGE_PIN M15 IOSTANDARD LVCMOS33 PULLUP true} [get_ports wm8978_iic_scl]

set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks -of_objects [get_pins u_audio_pll/u_audio_clk_buf/O]]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
