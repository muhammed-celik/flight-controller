create_clock -name clk_12mhz -period 83.333 [get_ports clk_12mhz]

# ext_reset is an asynchronous assertion source. Reset release is synchronized
# by reset_sync and analyzed in the generated 100 MHz clock domain.
set_false_path -from [get_ports ext_reset]
