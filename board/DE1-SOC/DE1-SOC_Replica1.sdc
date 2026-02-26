# Create a clock constraint for the 12MHz onboard oscillator
# Replace 'clk_input_pin' with the actual name of the clock port in your top-level design
create_clock -name {sys_clk} -period 83.333 -waveform {0 41.666} [get_ports {clk_input_pin}]

# Automatically constrain internal PLL outputs and other clock circuitry
derive_pll_clocks
derive_clock_uncertainty

# Relax timing for user I/O (actual DE1-SOC port names)
set_false_path -from [get_ports {KEY[*]}]
set_false_path -to [get_ports {LEDR[*]}]
set_false_path -from [get_ports {SW[*]}]
set_false_path -to [get_ports {HEX*}]