# =============================================================
# lane_detect.sdc
# Timing constraints for FPGA Vision Remote Lab
# Clock: 74.25 MHz  Period: 13.468 ns
# =============================================================

create_clock -name {clk} -period 13.468 -waveform {0.000 6.734} [get_ports {clk}]

derive_pll_clocks
derive_clock_uncertainty

# Input delays (from HDMI receiver to FPGA)
set_input_delay -clock {clk} -max 3.0 [get_ports {vs_in hs_in de_in}]
set_input_delay -clock {clk} -min 0.5 [get_ports {vs_in hs_in de_in}]
set_input_delay -clock {clk} -max 3.0 [get_ports {r_in[*] g_in[*] b_in[*]}]
set_input_delay -clock {clk} -min 0.5 [get_ports {r_in[*] g_in[*] b_in[*]}]
set_input_delay -clock {clk} -max 3.0 [get_ports {reset_n enable_in[*]}]
set_input_delay -clock {clk} -min 0.0 [get_ports {reset_n enable_in[*]}]

# Output delays (from FPGA to HDMI transmitter)
set_output_delay -clock {clk} -max 3.0 [get_ports {vs_out hs_out de_out clk_o}]
set_output_delay -clock {clk} -min 0.5 [get_ports {vs_out hs_out de_out clk_o}]
set_output_delay -clock {clk} -max 3.0 [get_ports {r_out[*] g_out[*] b_out[*]}]
set_output_delay -clock {clk} -min 0.5 [get_ports {r_out[*] g_out[*] b_out[*]}]
set_output_delay -clock {clk} -max 3.0 [get_ports {led[*]}]
set_output_delay -clock {clk} -min 0.0 [get_ports {led[*]}]
