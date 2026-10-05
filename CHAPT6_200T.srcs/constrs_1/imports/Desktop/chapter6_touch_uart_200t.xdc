# GX-BIDT Chapter 6 touch + external DB9 RS-232 UART
# Device: XC7A200T-FBG484-2

# W19 is 100 MHz on the tested board
set_property -dict {PACKAGE_PIN W19 IOSTANDARD LVCMOS33} [get_ports sys_clk]
create_clock -name sys_clk -period 10.000 [get_ports sys_clk]

# Touch interface; keep BSW_CTRL1 7/8/9 in the verified positions
set_property -dict {PACKAGE_PIN R16 IOSTANDARD LVCMOS33} [get_ports TOUCH_SCL]
set_property -dict {PACKAGE_PIN E16 IOSTANDARD LVCMOS33} [get_ports TOUCH_SDA]
set_property -dict {PACKAGE_PIN F15 IOSTANDARD LVCMOS33} [get_ports TOUCH_INT]
set_property -dict {PACKAGE_PIN K16 IOSTANDARD LVCMOS33} [get_ports TOUCH_RST]

# Core-board LEDs
set_property -dict {PACKAGE_PIN J16 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN E22 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN E19 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN D21 IOSTANDARD LVCMOS33} [get_ports {led[4]}]

# External DB9 RS-232 TX path:
# R17 -> F_B14_L24_N / UART2_TX -> JP1 pins 1-2 -> MAX3232 -> DB9
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports UART_TXD]
