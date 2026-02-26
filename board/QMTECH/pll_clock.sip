set_global_assignment -entity "pll_clock" -library "lib_pll_clock" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "pll_clock" -library "lib_pll_clock" -name IP_TOOL_VERSION "24.1"
set_global_assignment -entity "pll_clock" -library "lib_pll_clock" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_pll_clock" -name SPD_FILE [file join $::quartus(sip_path) "pll_clock.spd"]

set_global_assignment -library "lib_pll_clock" -name MISC_FILE [file join $::quartus(sip_path) "pll_clock_sim/pll_clock.vho"]
