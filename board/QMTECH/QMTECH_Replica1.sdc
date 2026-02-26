# Create a clock constraint for the 12MHz onboard oscillator
# Replace 'clk_input_pin' with the actual name of the clock port in your top-level design
create_clock -name {sys_clk} -period 83.333 -waveform {0 41.666} [get_ports {clk_input_pin}]

# Automatically constrain internal PLL outputs and other clock circuitry
derive_pll_clocks
derive_clock_uncertainty

