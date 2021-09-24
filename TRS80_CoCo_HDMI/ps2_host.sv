module ps2_host(
		input logic clk,
		input logic rst,
		input logic ps2_clk_d,
		input logic ps2_data_d,
		input logic[7:0] tx_data,
		input logic tx_req,
		output logic ps2_clk_q,
		output logic ps2_data_q,
		output logic [7:0] rx_data,
		output logic rx_ready,
		output logic tx_ready);

logic prev_ps2_clk;
logic prev_tx_req;
logic tx_last, prev_tx_last;
logic tx_done;
logic rx_inhibit;
logic ps2_clk_s, ps2_data_s;
logic [11:0] rx_shift_reg;
logic [10:0] tx_shift_reg;
logic [12:0] timer;
logic timer_z;
assign timer_z = (timer == 13'h0000);
assign tx_last = &tx_shift_reg;

initial
begin
	rx_shift_reg = 12'b100000000000;
	timer = 13'h0000;
	rx_data = 8'h00;
	rx_ready = 1'b0;
end
always @(posedge clk)
begin
	ps2_clk_s <= ps2_clk_d;
	ps2_data_s <= ps2_data_d;
	prev_ps2_clk <= ps2_clk_s;
	prev_tx_req <= tx_req;
	prev_tx_last <= tx_last;
	if(rst)
	begin
		rx_shift_reg <= 12'b100000000000;
		timer <= 13'h0000;
		rx_data <= 8'h00;
		rx_ready <= 1'b0;
	end
	else if(rx_ready | rx_inhibit)
	begin
		rx_shift_reg <= 12'b100000000000;
	end
	else if(prev_ps2_clk & (~ps2_clk_s))
		rx_shift_reg <= {ps2_data_s, rx_shift_reg[11:1]};
	if(rx_shift_reg[0])	//last bit received
		rx_data <= rx_shift_reg[9:2];
	rx_ready <= rx_shift_reg[0];
	
	if(tx_req & (~prev_tx_req))
	begin
		timer <= 13'b1111111111111;
		rx_inhibit <= 1'b1;
	end
	else if(~timer_z)
		timer <= timer - 13'h0001;
	if(~timer_z)
		tx_shift_reg <= {1'b1, ~^tx_data, tx_data, 1'b0};
	else if(prev_ps2_clk & (~ps2_clk_s))
		tx_shift_reg <= {1'b1, tx_shift_reg[10:1]};
	ps2_clk_q <= ~timer_z;
	ps2_data_q <= (~tx_shift_reg[0]) & timer_z;
	if(tx_last & (~prev_tx_last))
		tx_done <= 1'b1;	//tx_done means the last 0 has been sent (data line is released by host)
	tx_ready <= tx_done & (prev_ps2_clk & (~ps2_clk_s) & (~ps2_data_s));	//detect ACK
	if(tx_ready)
	begin
		rx_inhibit <= 1'b0;
		tx_done <= 1'b0;
	end
end
endmodule
