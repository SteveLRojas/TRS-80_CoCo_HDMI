module SDRAM_controller(
		input wire clk,
		input wire reset,
		input wire HSYNC,
		
		input wire[20:0] A_address,
		input wire A_write,
		output reg[7:0] A_data_out,
		input wire[7:0] A_data_in,
		
		input wire[20:0] B_address,
		output reg[7:0] B_data_out,
		
		output wire SDRAM_CKE,
		output wire SDRAM_CSn,
		output wire SDRAM_WREn,
		output wire SDRAM_CASn,
		output wire SDRAM_RASn,
		output reg[10:0] SDRAM_A,
		output reg SDRAM_BA,
		output reg SDRAM_DQM,
		inout wire[7:0] SDRAM_DQ);
		
assign SDRAM_CKE = 1'b1;
assign SDRAM_CSn = 1'b0;

localparam [2:0] SDRAM_CMD_LOADMODE  = 3'b000;
localparam [2:0] SDRAM_CMD_REFRESH   = 3'b001;
localparam [2:0] SDRAM_CMD_PRECHARGE = 3'b010;
localparam [2:0] SDRAM_CMD_ACTIVE    = 3'b011;
localparam [2:0] SDRAM_CMD_WRITE     = 3'b100;
localparam [2:0] SDRAM_CMD_READ      = 3'b101;
localparam [2:0] SDRAM_CMD_NOP       = 3'b111;

reg [2:0] SDRAM_CMD;
reg gate_out;
reg[4:0] state;
reg prev_HSYNC;
reg refresh_flag, refresh_flag_2;
reg[8:0] A_address_hold, B_address_hold;
reg[7:0] A_data_hold;
reg BA_hold;

assign {SDRAM_RASn, SDRAM_CASn, SDRAM_WREn} = SDRAM_CMD;
assign SDRAM_DQ = gate_out ? A_data_hold : 8'hZZ;

localparam[4:0]
			S_reset = 5'h00,
			S_init = 5'h01,
			S_init_NOP = 5'h02,
			S_mode = 5'h3,
			S_mode_NOP = 5'h4,
			S_activate_A = 5'h6,
			S_activate_A_NOP = 5'h7,
			S_write_A = 5'h8,
			S_write_NOP = 5'h9,
			S_activate_B = 5'h0E,
			S_activate_B_NOP = 5'h0F,
			S_read_B = 5'h10,
			S_read_B_NOP = 5'h11,
			S_refresh_1 = 5'h15,
			S_refresh_NOP_11 = 5'h16,
			S_refresh_NOP_12 = 5'h17,
			S_refresh_NOP_13 = 5'h18,
			S_refresh_2 = 5'h19,
			S_refresh_NOP_21 = 5'h1A,
			S_refresh_NOP_22 = 5'h1B,
			S_refresh_NOP_23 = 5'h1C;
			
initial begin
	state = S_reset;
end

always @(posedge clk)
begin
	if(reset)
	begin
		state <= S_reset;
		gate_out <= 1'b0;
	end
	else
	begin
		prev_HSYNC <= HSYNC;
		if((~HSYNC) & prev_HSYNC)
			refresh_flag <= 1'b1;
		else if(state == S_refresh_1)
			refresh_flag <= 1'b0;
		gate_out <= 1'b0;
		SDRAM_DQM <= 1'b0;
		case(state)
			S_reset:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				gate_out <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_init;
			end
			S_init:
			begin
				SDRAM_CMD <= SDRAM_CMD_PRECHARGE;
				SDRAM_A[10] <= 1'b1;	//precharge all
				gate_out <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_init_NOP;
			end
			S_init_NOP:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				gate_out <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_mode;
			end
			S_mode:
			begin
				SDRAM_CMD <= SDRAM_CMD_LOADMODE;
				SDRAM_A <= 11'b00000100000;
				SDRAM_BA <= 1'b0;
				gate_out <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_mode_NOP;
			end
			S_mode_NOP:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				gate_out <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_1;
			end
			S_activate_A:
			begin
				SDRAM_CMD <= SDRAM_CMD_ACTIVE;	//send activate command
				SDRAM_BA <= A_address[20];	//set bank
				SDRAM_A[10:0] <= A_address[19:9];	//set row
				A_address_hold <= A_address[8:0];
				A_data_hold <= A_data_in;
				BA_hold <= A_address[20];
				SDRAM_DQM <= 1'b1;
				state <= S_activate_A_NOP;
			end
			S_activate_A_NOP:
			begin
				B_data_out <= SDRAM_DQ;
				SDRAM_CMD <= SDRAM_CMD_NOP;
				SDRAM_DQM <= 1'b1;
				gate_out <= 1'b0;
				state <= S_write_A;
			end
			S_write_A:
			begin
				if(A_write)
				begin
					SDRAM_CMD <= SDRAM_CMD_WRITE;
					SDRAM_BA <= BA_hold;
					//SDRAM_A[11] <= 1'b0;
					SDRAM_A[9] <= 1'b0;
					SDRAM_A[8:0] <= A_address_hold;	//set col
					SDRAM_A[10] <= 1'b1;	//automatic precharge
					SDRAM_DQM <= 1'b0;
					gate_out <= 1'b1;
				end
				else
				begin
					SDRAM_CMD <= SDRAM_CMD_READ;
					//SDRAM_CMD <= SDRAM_CMD_NOP;
					SDRAM_BA <= BA_hold;
					//SDRAM_A[11] <= 1'b0;
					SDRAM_A[9] <= 1'b0;
					SDRAM_A[8:0] <= A_address_hold;	//set column
					SDRAM_A[10] <= 1'b1;	//auto precharge
					SDRAM_DQM <= 1'b0;
					gate_out <= 1'b0;
				end
				state <= S_write_NOP;
			end
			S_write_NOP:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				SDRAM_DQM <= 1'b0;
				gate_out <= 1'b0;
				if(refresh_flag)
					state <= S_refresh_1;
				else if(refresh_flag_2)
					state <= S_refresh_2;
				else
					state <= S_activate_B;
			end
			S_activate_B:
			begin
				SDRAM_CMD <= SDRAM_CMD_ACTIVE;	//send activate command
				SDRAM_BA <= B_address[20];	//set bank
				BA_hold <= B_address[20];
				SDRAM_A[10:0] <= B_address[19:9];	//set row
				B_address_hold <= B_address[8:0];
				SDRAM_DQM <= 1'b1;
				gate_out <= 1'b0;
				state <= S_activate_B_NOP;
			end
			S_activate_B_NOP:
			begin
				A_data_out <= SDRAM_DQ;
				SDRAM_CMD <= SDRAM_CMD_NOP;
				SDRAM_DQM <= 1'b1;
				state <= S_read_B;
			end
			S_read_B:
			begin
				SDRAM_CMD <= SDRAM_CMD_READ;
				SDRAM_BA <= BA_hold;
				//SDRAM_A[11] <= 1'b0;
				SDRAM_A[9] <= 1'b0;
				SDRAM_A[8:0] <= B_address_hold;	//set column
				SDRAM_A[10] <= 1'b1;	//auto precharge
				SDRAM_DQM <= 1'b0;
				state <= S_read_B_NOP;
			end
			S_read_B_NOP:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				SDRAM_DQM <= 1'b1;
				state <= S_activate_A;
			end
			S_refresh_1:
			begin
				SDRAM_CMD <= SDRAM_CMD_REFRESH;
				SDRAM_DQM <= 1'b1;
				refresh_flag_2 <= 1'bx;
				state <= S_refresh_NOP_11;
			end
			S_refresh_NOP_11:
			begin
				A_data_out <= SDRAM_DQ;
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'bx;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_NOP_12;
			end
			S_refresh_NOP_12:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'bx;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_NOP_13;
			end
			S_refresh_NOP_13:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'b1;
				SDRAM_DQM <= 1'b1;
				state <= S_activate_A;
			end
			S_refresh_2:
			begin
				SDRAM_CMD <= SDRAM_CMD_REFRESH;
				refresh_flag_2 <= 1'bx;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_NOP_21;
			end
			S_refresh_NOP_21:
			begin
				A_data_out <= SDRAM_DQ;
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'bx;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_NOP_22;
			end
			S_refresh_NOP_22:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'bx;
				SDRAM_DQM <= 1'b1;
				state <= S_refresh_NOP_23;
			end
			//S_refresh_NOP_23:
			default:
			begin
				SDRAM_CMD <= SDRAM_CMD_NOP;
				refresh_flag_2 <= 1'b0;
				SDRAM_DQM <= 1'b1;
				state <= S_activate_A;
			end
		endcase
	end
end
endmodule
