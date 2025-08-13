# Apple-1 Timing Constraints for DE10-Lite

# Main 50MHz clock constraint (DE10-Lite standard name)
create_clock -name "clk_50" -period 20.000ns [get_ports {MAX10_CLK1_50}]

# If you have a generated CPU clock signal in your design
# create_generated_clock -name "cpu_clk" -source [get_ports {MAX10_CLK1_50}] -divide_by 50 [get_nets {cpu_clk_signal}]

# Relax timing for user I/O (actual DE10-Lite port names)
set_false_path -from [get_ports {KEY[*]}]
set_false_path -to [get_ports {LEDR[*]}]

# If you have SW switches
# set_false_path -from [get_ports {SW[*]}]

# If you have 7-segment displays
# set_false_path -to [get_ports {HEX*}]