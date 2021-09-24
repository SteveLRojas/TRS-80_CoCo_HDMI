module PIA(
			input logic clk,
			input logic rst,
			input logic CS,
			input logic RW,
			input logic VMA,
			input logic[1:0] RS,
			output logic IRQA, IRQB,
			input logic[7:0] data_to_PIA,
			output logic[7:0] data_from_PIA,
			input logic CA1, CB1, CA2_in, CB2_in,
			output logic CA2_out, CB2_out,
			output logic[7:0] PA_out, PB_out,
			input logic[7:0] PA_in, PB_in);
logic[7:0] control_A, control_B;
logic[7:0] DDR_A, DDR_B;
logic[7:0] PB_read_data, PA_read_data;
logic prev_CA1, prev_CA2_in;
logic prev_CB1, prev_CB2_in;
logic prev_VMA;
assign PB_read_data = (DDR_B & PB_out) | ((~DDR_B) & PB_in);
assign PA_read_data = (DDR_A & PA_out) | ((~DDR_A) & PA_in);
assign IRQA = (control_A[0] & control_A[7]) | (control_A[3] & control_A[6]);
assign IRQB = (control_B[0] & control_B[7]) | (control_B[3] & control_B[6]);
//register write logic
always @(posedge clk)
begin
	prev_CA1 <= CA1;
	prev_CA2_in <= CA2_in;
	prev_CB1 <= CB1;
	prev_CB2_in <= CB2_in;
	prev_VMA <= VMA;
	if(rst)
	begin
		PA_out <= 8'h00;
		PB_out <= 8'h00;
		DDR_A <= 8'h00;
		DDR_B <= 8'h00;
	end
	else if((~RW) & VMA & CS)	//write enabled
	begin
		if(RS[1] == 1'b0)	//side A
		begin
			if(RS[0])	//control register
				control_A[5:0] <= data_to_PIA[5:0];
			else if(control_A[2])	//data register A
				PA_out <= data_to_PIA;
			else	//data direction A
				DDR_A <= data_to_PIA;
		end
		else	//side B
		begin
			if(RS[0])	//control register
					control_B[5:0] <= data_to_PIA[5:0];
			else if(control_B[2])	//data register B
				PB_out <= data_to_PIA;
			else	//data direction B
				DDR_B <= data_to_PIA;
		end
	end
	//detect IRQA1
	if(control_A[1])	//rising edge
	begin
		if(CA1 & (~prev_CA1))
			control_A[7] <= 1'b1;
	end
	else	//falling edge
	begin
		if(prev_CA1 & (~CA1))
			control_A[7] <= 1'b1;
	end
	//detect IRQB1
	if(control_B[1])	//rising edge
	begin
		if(CB1 & (~prev_CB1))
			control_B[7] <= 1'b1;
	end
	else	//falling edge
	begin
		if(prev_CB1 & (~CB1))
			control_B[7] <= 1'b1;
	end
	//detect IRQA2
	if(~control_A[5])	//CA2 is an input
	begin
		if(control_A[4])	//rising edge
		begin
			if(CA2_in & (~prev_CA2_in))
				control_A[6] <= 1'b1;
		end
		else	//falling edge
		begin
			if(prev_CA2_in & (~CA2_in))
				control_A[6] <= 1'b1;
		end
	end
	else
		control_A[6] <= 1'b0;
	//detect IRQB2
	if(~control_B[5])	//CB2 is an input
	begin
		if(control_B[4])	//rising edge
		begin
			if(CB2_in & (~prev_CB2_in))
				control_B[6] <= 1'b1;
		end
		else	//falling edge
		begin
			if(prev_CB2_in & (~CB2_in))
				control_B[6] <= 1'b1;
		end
	end
	else
		control_B[6] <= 1'b0;
	//clear interrupt flags
	if(RW & CS & VMA)
	begin
		if((RS[1] == 0) && (RS[0] == 1'b0))	//side A
		begin
			if(control_A[2])
			begin
				control_A[7] <= 1'b0;
				control_A[6] <= 1'b0;
			end
		end
		if(RS[1] & (~RS[0]))	//side B
		begin
			if(control_B[2])
			begin
				control_B[7] <= 1'b0;
				control_B[6] <= 1'b0;
			end
		end
	end
	//set CA2 output
	if(control_A[5])	//CA2 is an output
	begin
		if(control_A[4])
			CA2_out <= control_A[3];
		else
		begin
			if(RW & CS & (RS == 2'b00) & control_A[2] & prev_VMA & (~VMA))
					CA2_out <= 1'b0;
			if(control_A[3])
			begin
				if((~CS) & prev_VMA & (~VMA))
					CA2_out <= 1'b1;
			end
			else
			begin
				if(control_A[1])	//rising edge
				begin
					if(CA1 & (~prev_CA1))
						CA2_out <= 1'b1;
					end
				else	//falling edge
				begin
					if(prev_CA1 & (~CA1))
						CA2_out <= 1'b1;
				end
			end
		end
	end
	else
		CA2_out <= 1'b0;
	//set CB2_output
	if(control_B[5])	//CB2 is an output
	begin
		if(control_B[4])
			CB2_out <= control_B[3];
		else
		begin
			if((~RW) & CS & VMA & (RS == 2'b10) & control_B[2])
				CB2_out <= 1'b0;
			if(control_B[3])
			begin
				if((~CS) & (~prev_VMA) & VMA)
					CB2_out <= 1'b1;
			end
			else
			begin
				if(~control_B[7])
				begin
					if(control_B[4])	//rising edge
					begin
						if(CB2_in & (~prev_CB2_in))
							CB2_out <= 1'b1;
					end
					else	//falling edge
					begin
						if(prev_CB2_in & (~CB2_in))
							CB2_out <= 1'b1;
					end
				end
			end
		end
	end
	else
		CB2_out <= 1'b0;
end

always_comb
begin
	if(RS[1] == 1'b0)	//side A
	begin
		if(RS[0])
			data_from_PIA = control_A;
		else if(control_A[2])
			data_from_PIA = PA_read_data;
		else
			data_from_PIA = DDR_A;
	end
	else	//side B
	begin
		if(RS[0])
			data_from_PIA = control_B;
		else if(control_B[2])
			data_from_PIA = PB_read_data;
		else
			data_from_PIA = DDR_B;
	end
end

endmodule
