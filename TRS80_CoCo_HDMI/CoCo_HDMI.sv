module CoCo_HDMI(
		input logic RST,
		input logic Clk,
		//input logic TAPE_IN,
		//input logic[3:0] button,
		//output logic TAPE_OUT,
		//output logic[3:0] LED,
		output logic[2:0] TMDS,
		output logic TMDS_clk,
		input logic debug,
		//output logic audio,
		input logic ps2_clk_d, ps2_data_d,
		output logic ps2_clk_q, ps2_data_q,
		
		output wire SDRAM_CLK,
		output wire SDRAM_CKE,
		output wire SDRAM_CSn,
		output wire SDRAM_WREn,
		output wire SDRAM_CASn,
		output wire SDRAM_RASn,
		output wire[12:0] SDRAM_A,
		output wire[1:0] SDRAM_BA,
		output wire[1:0] SDRAM_DQM,
		inout wire[15:0] SDRAM_DQ);

//logic[3:0] button_s;
logic debug_s;
logic reset;
logic[2:0] S;
logic[7:0] IO_data;
logic CPU_RW;
logic CPU_VMA;
logic[15:0] CPU_address;
logic[7:0] data_from_CPU;
logic[7:0] data_to_CPU;
logic[12:0] VDG_address;
logic[7:0] VDG_data;
logic[7:0] data_from_PIA0, data_from_PIA1;
logic[7:0] PB1_out;
logic[7:0] base_ROM_data;
logic[7:0] ext_ROM_data;
logic base_ROM_E;
logic ext_ROM_E;
logic PIA0_E;
logic PIA1_E;
logic IRQA0, IRQA1, IRQB0, IRQB1;
logic CPU_clk;
logic clk_25;
logic clk_50;
logic clk_250;
logic HSYNC, VSYNC;

PLL_CPU PLL0(.inclk0(Clk), .c0(CPU_clk));
PLL_GFX PLL1(.inclk0(Clk), .c0(clk_25), .c1(clk_250), .c2(clk_50), .c3(SDRAM_CLK));

always @(posedge clk_50)
begin
	//button_s <= ~button;
	debug_s <= debug;
	reset <= ~RST;
end

SN74LS783 SAM(
			.rst(reset),
			.clk(clk_50),
			.HSYNC,
			.S,
			.IO_data,
			.CPU_RW,
			.CPU_VMA,
			.CPU_address,
			.data_from_CPU,
			.data_to_CPU,
			.VDG_address,
			.VDG_data,
			
			.SDRAM_CKE,
			.SDRAM_CSn,
			.SDRAM_WREn,
			.SDRAM_CASn,
			.SDRAM_RASn,
			.SDRAM_A,
			.SDRAM_BA,
			.SDRAM_DQM,
			.SDRAM_DQ);
			
MC6809 CPU(	
		.clk(CPU_clk), // E clock input (falling edge)
		.rst(reset), // reset input (active high)
		.vma(CPU_VMA), // valid memory address (active high)
    	.lic_out(), // last instruction cycle (active high)
    	.ifetch(), // instruction fetch cycle (active high)
    	.opfetch(), // opcode fetch (active high)
    	.ba(), // bus available (high on sync wait or DMA grant)
    	.bs(), // bus status (high on interrupt or reset vector fetch or DMA grant)
		.addr(CPU_address), // address bus output
		.rw(CPU_RW), // read not write output
		.data_out(data_from_CPU), // data bus output
		.data_in(data_to_CPU), // data bus input
		.irq(1'b0), // interrupt request input (active high)
		.firq(1'b0), // fast interrupt request input (active high)
		.nmi(1'b0), // non maskable interrupt request input (active high)
		.halt(1'b0), // halt input (active high) grants DMA
		.hold(~debug_s) // hold input (active high) extend bus cycle
		);
		
MC6847_gen4 VDG(
			.clk_25(clk_25),
			.clk_250(clk_250),
			.reset(reset),
			.AG(PB1_out[7]),
			.SA(VDG_data[7]),
			.INV(VDG_data[6]),
			.DD(VDG_data),
			.DA(VDG_address),
			.TMDS(TMDS),
			.TMDS_clk(TMDS_clk),
			.virtual_HSYNC(HSYNC),
			.virtual_VSYNC(VSYNC));
			
assign base_ROM_E = (S == 3'h2);
assign ext_ROM_E = (S == 3'h1);
assign PIA0_E = (S == 3'h4);
assign PIA1_E = (S == 3'h5);

PIA PIA0(
			.clk(clk_50),
			.rst(reset),
			.CS(PIA0_E),
			.RW(CPU_RW),
			.VMA(CPU_VMA),
			.RS(CPU_address[1:0]),
			.IRQA(IRQA0), .IRQB(IRQB0),
			.data_to_PIA(data_from_CPU),
			.data_from_PIA(data_from_PIA0),
			.CA1(HSYNC), .CB1(VSYNC), .CA2_in(1'b0), .CB2_in(1'b0),
			.CA2_out(), .CB2_out(),
			.PA_out(), .PB_out(col_select),
			.PA_in({1'b1, key_out}), .PB_in(8'h00));
			
PIA PIA1(
			.clk(clk_50),
			.rst(reset),
			.CS(PIA1_E),
			.RW(CPU_RW),
			.VMA(CPU_VMA),
			.RS(CPU_address[1:0]),
			.IRQA(IRQA1), .IRQB(IRQB1),
			.data_to_PIA(data_from_CPU),
			.data_from_PIA(data_from_PIA1),
			.CA1(1'b0), .CB1(1'b1), .CA2_in(1'b0), .CB2_in(1'b0),
			.CA2_out(), .CB2_out(),
			.PA_out(), .PB_out(PB1_out),
			.PA_in(8'hFF), .PB_in(8'h00));
			
base_ROM ROM0(.address(CPU_address[12:0]), .clock(clk_50), .q(base_ROM_data));
ext_ROM ROM1(.address(CPU_address[12:0]), .clock(clk_50), .q(ext_ROM_data));

always_comb
begin
	IO_data = CPU_address[7:0];
	if(base_ROM_E)
		IO_data = base_ROM_data;
	if(ext_ROM_E)
		IO_data = ext_ROM_data;
	if(PIA0_E)
		IO_data = data_from_PIA0;
	if(PIA1_E)
		IO_data = data_from_PIA1;
end

logic[6:0] key_code;
logic[6:0] key_out;
logic[7:0] col_select;
PS2_keyboard PS2_inst(.clk(clk_50), .reset, .ps2_clk_d, .ps2_data_d, .ps2_clk_q, .ps2_data_q, .key_code);
KEY_MATRIX key_matrix0(.col_select, .key_code, .key_out);
endmodule
