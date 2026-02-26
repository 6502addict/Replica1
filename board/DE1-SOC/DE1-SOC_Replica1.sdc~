# Replica1-sdram Timing Constraints for DE1-SOC

# Main 50MHz clock constraint (DE1-SOC standard name)
create_clock -name "clk_50" -period 20.000ns [get_ports {CLOCK_50}]

derive_pll_clocks
derive_clock_uncertainty

# If you have a generated CPU clock signal in your design
#create_generated_clock -name "main_clk" -source [get_ports {CLOCK_50}] -divide_by 50 [get_nets {cpu_clk_signal}]

# Relax timing for user I/O (actual DE1-SOC port names)
set_false_path -from [get_ports {KEY[*]}]
set_false_path -to [get_ports {LEDR[*]}]
set_false_path -from [get_ports {SW[*]}]
set_false_path -to [get_ports {HEX*}]