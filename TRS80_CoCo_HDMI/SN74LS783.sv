module SN74LS783(
			input logic rst,
			input logic clk,
			input logic HSYNC,
			output logic[2:0] S,
			input logic[7:0] IO_data,
			input logic CPU_RW,
			input logic CPU_VMA,
			input logic[15:0] CPU_address,
			input logic[7:0] data_from_CPU,
			output logic[7:0] data_to_CPU,
			input logic[12:0] VDG_address,
			output logic[7:0] VDG_data,
			
			output logic SDRAM_CKE,
			output logic SDRAM_CSn,
			output logic SDRAM_WREn,
			output logic SDRAM_CASn,
			output logic SDRAM_RASn,
			output logic[12:0] SDRAM_A,
			output logic[1:0] SDRAM_BA,
			output logic[1:0] SDRAM_DQM,
			inout wire[15:0] SDRAM_DQ);
logic[15:0] control_reg;
logic control_enable;
logic[15:0] VDG_effective_address;
logic[15:0] CPU_effective_address;
logic[7:0] RAM_to_CPU;
logic RAM_write;
// Control bits:
// 15: TY	map type
// 14: M1
// 13: M0	memory size
// 12: R1
// 11: R0	CPU clock rate
// 10: P1	page #1
// 09: F6
// 08: F5
// 07: F4
// 06: F3
// 05: F2
// 04: F1
// 03: F0	VDG offset
// 02: V2
// 01: V1
// 00: V0	//VDG addressing mode (does not have to match the actual VDG mode)
assign control_enable = (CPU_address[15:5] == 11'b11111111110) & (~CPU_RW) & CPU_VMA;
assign VDG_effective_address = {control_reg[9:3], 9'h00} + {3'b000, VDG_address};
assign CPU_effective_address = {control_reg[10], CPU_address[14:0]};
assign RAM_write = (~CPU_address[15]) & (~CPU_RW) & CPU_VMA;

always @(posedge clk)
begin
	if(rst)
		control_reg <= 16'h00;
	else if(control_enable)
		control_reg[CPU_address[4:1]] <= CPU_address[0];
end

always_comb
begin
	if(CPU_address < 16'h8000)
	begin
		if(CPU_RW)
			S = 3'h0;
		else
			S = 3'h7;
	end
	else if(CPU_address < 16'hA000)
		S = 3'h1;
	else if(CPU_address < 16'hC000)
		S = 3'h2;
	else if(CPU_address < 16'hFF00)
		S = 3'h3;
	else if(CPU_address < 16'hFF20)
		S = 3'h4;
	else if(CPU_address < 16'hFF40)
		S = 3'h5;
	else if(CPU_address < 16'hFF60)
		S = 3'h6;
	else if(CPU_address < 16'hFFE0)
		S = 3'h7;
	else
		S = 3'h2;
	if(CPU_address[15])
		data_to_CPU = IO_data;
	else
		data_to_CPU = RAM_to_CPU;
end

SDRAM_controller SDRAM_inst(
		.clk,
		.reset(rst),
		.HSYNC,
		
		.A_address({8'h00, CPU_effective_address}),
		.A_write(RAM_write),
		.A_data_out(RAM_to_CPU),
		.A_data_in({8'h00, data_from_CPU}),
		
		.B_address({8'h00, VDG_effective_address}),
		.B_data_out(VDG_data),
		
		.SDRAM_CKE,
		.SDRAM_CSn,
		.SDRAM_WREn,
		.SDRAM_CASn,
		.SDRAM_RASn,
		.SDRAM_A,
		.SDRAM_BA,
		.SDRAM_DQM,
		.SDRAM_DQ);
		
endmodule
