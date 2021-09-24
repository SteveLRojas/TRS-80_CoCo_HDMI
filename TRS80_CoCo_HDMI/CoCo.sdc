create_clock -period 20ns [get_ports Clk]
derive_pll_clocks
derive_clock_uncertainty
#set_false_path -from * -to [get_ports {LED* audio}]
set_false_path -from * -to [get_ports {TMDS*}]
set_false_path -from * -to [get_ports {ps2_clk_q}]
set_false_path -from * -to [get_ports {ps2_data_q}]
set_input_delay -clock PLL1|altpll_component|auto_generated|pll1|clk[2] -max 1 [all_inputs]
set_input_delay -clock PLL1|altpll_component|auto_generated|pll1|clk[2] -min 1 [all_inputs]
set_output_delay -clock PLL1|altpll_component|auto_generated|pll1|clk[2] -max 2 [all_outputs]
set_output_delay -clock PLL1|altpll_component|auto_generated|pll1|clk[2] -min 1 [all_outputs]

