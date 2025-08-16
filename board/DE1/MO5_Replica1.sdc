create_clock -name CLOCK_50 -period 20 [get_ports CLOCK_50]
create_clock -period 41.667 -name ext_clock [get_ports {CLOCK_24[0]}]

derive_pll_clocks
derive_clock_uncertainty

set_clock_groups -asynchronous -group [get_clocks CLOCK_50]

create_clock -name key0_clock -period 10.000 [get_ports {KEY[0]}]

#create_generated_clock -name cpu_clock -source [get_ports CLOCK_50] \
#                      [get_nets {MO5_CLOCK:clks|clock_divider:cpu_clk|clk_out_reg}]

create_generated_clock -name vga_clock -source [get_ports CLOCK_50] \
                      [get_nets {MO5_CLOCK:clks|clock_divider:vga_clk|clk_out_reg}]

#create_generated_clock -name spi_prog_clock -source [get_ports CLOCK_50]\
#                      [get_nets {spi_master:spi|prog_clock_divider:clock|clk_out}]

#create_generated_clock -name spi_clock -source [get_ports CLOCK_50] \
#                      [get_nets {MO5_CLOCK:clks|clock_divider:spi_clk|clk_out}]

#create_generated_clock -name acia_clock -source [get_ports CLOCK_50] \
#                      [get_nets {MO5_CLOCK:clks|clock_divider:acia_clk|clk_out}]
							 
create_generated_clock -name mhz10_clock -source [get_ports CLOCK_50] \
    [get_nets {MO5_CLOCK:clks|clock_divider:mhz10_clk|clk_out_reg}]

create_generated_clock -name mhz1_clock -source [get_ports CLOCK_50] \
    [get_nets {MO5_CLOCK:clks|clock_divider:mhz1_clk|clk_out_reg}]							 
							 
create_generated_clock -name synlt_clock -source [get_ports CLOCK_50] \
    [get_nets {MO5_CLOCK:clks|clock_divider:synlt_clk|clk_out_reg}]
	 
create_generated_clock -name lightpen_baud_clock -source [get_ports CLOCK_50] \
    [get_nets {MO5_LIGHTPEN:LPN|lightpen_capture:LOG|serial_write:SERIAL_WRITER|baud_clock_divider:BAUD_GEN|baud_clk_i}]
	 

create_clock -name kbd_edge_clock -period 40.000 [get_registers {MO5_KBD:KBD|ps2:kbd|edge_detector:edge|r0}]


set_clock_groups -asynchronous -group {CLOCK_50} -group {vga_clock mhz10_clock mhz10_clock synlt_clock lightpen_baud_clock}
#set_clock_groups -asynchronous -group {CLOCK_50} -group {cpu_clock vga_clock mhz10_clock mhz10_clock synlt_clock lightpen_baud_clock}
#set_clock_groups -asynchronous -group {spi_prog_clock spi_clock}
set_clock_groups -asynchronous -group {kbd_edge_clock}

