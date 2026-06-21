## Clock (12 MHz on-board oscillator)
set_property -dict {PACKAGE_PIN L17 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 83.333 -name sys_clk [get_ports clk]

## Button (active-high, directly on board)
set_property -dict {PACKAGE_PIN A18 IOSTANDARD LVCMOS33} [get_ports btn]

## LED
set_property -dict {PACKAGE_PIN A17 IOSTANDARD LVCMOS33} [get_ports led]

## Configuration
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
