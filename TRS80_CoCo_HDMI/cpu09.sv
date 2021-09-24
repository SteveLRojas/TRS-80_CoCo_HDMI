module MC6809(	
		input logic clk, // E clock input (falling edge)
		input logic rst, // reset input (active high)
		output logic vma, // valid memory address (active high)
    	output logic lic_out, // last instruction cycle (active high)
    	output logic ifetch, // instruction fetch cycle (active high)
    	output logic opfetch, // opcode fetch (active high)
    	output logic ba, // bus available (high on sync wait or DMA grant)
    	output logic bs, // bus status (high on interrupt or reset vector fetch or DMA grant)
		output logic[15:0] addr, // address bus output
		output logic rw, // read not write output
		output logic[7:0] data_out, // data bus output
		input logic[7:0] data_in, // data bus input
		input logic irq, // interrupt request input (active high)
		input logic firq, // fast interrupt request input (active high)
		input logic nmi, // non maskable interrupt request input (active high)
		input logic halt, // halt input (active high) grants DMA
		input logic hold // hold input (active high) extend bus cycle
		);

  localparam EBIT = 7;
  localparam FBIT = 6;
  localparam HBIT = 5;
  localparam IBIT = 4;
  localparam NBIT = 3;
  localparam ZBIT = 2;
  localparam VBIT = 1;
  localparam CBIT = 0;

  //
  // Interrupt vector modifiers
  //
  localparam RST_VEC  = 3'b111;
  localparam NMI_VEC  = 3'b110;
  localparam SWI_VEC  = 3'b101;
  localparam IRQ_VEC  = 3'b100;
  localparam FIRQ_VEC = 3'b011;
  localparam SWI2_VEC = 3'b010;
  localparam SWI3_VEC = 3'b001;
  localparam RESV_VEC = 3'b000;

	typedef enum logic[7:0] {// Start off in Reset
	                    reset_state,
							// Fetch Interrupt Vectors (including reset)
							vect_lo_state, vect_hi_state, vect_idle_state,
                    	// Fetch Instruction Cycle
                    	fetch_state,
							// Decode Instruction Cycles
                    	decode_state,
							// Calculate Effective Address
						    imm16_state,
		                indexed_state, index8_state, index16_state, index16_2_state,
							pcrel8_state, pcrel16_state, pcrel16_2_state,
							indexaddr_state, indexaddr2_state,
						    postincr1_state, postincr2_state,
							indirect_state, indirect2_state, indirect3_state,
                    	extended_state,
							// single ops
							single_op_read_state,
						    single_op_exec_state,
	                    single_op_write_state,
							// Dual op states
							dual_op_read8_state, dual_op_read16_state, dual_op_read16_2_state,
						    dual_op_write8_state, dual_op_write16_state,
                    	// 
						    sync_state, halt_state, cwai_state,
							//
							andcc_state, orcc_state,
							tfr_state, 
                    	exg_state, exg1_state, exg2_state,
							lea_state,
							// Multiplication
						    mulea_state, muld_state,
						    mul0_state, mul1_state, mul2_state, mul3_state,
						    mul4_state, mul5_state, mul6_state,
							//  Branches
							lbranch_state, sbranch_state,
							// Jumps, Subroutine Calls and Returns
                    	jsr_state, jmp_state,
                    	push_return_hi_state, push_return_lo_state,
                    	pull_return_hi_state, pull_return_lo_state,
							// Interrupt cycles
							int_nmi_state, int_nmi1_state,
							int_irq_state, int_irq1_state,
							int_firq_state,  int_firq1_state,
							int_entire_state, int_fast_state,
							int_pcl_state,  int_pch_state,
						    int_upl_state,  int_uph_state,
						    int_iyl_state,  int_iyh_state,
						    int_ixl_state,  int_ixh_state,
						    int_dp_state,
				            int_accb_state, int_acca_state,
						    int_cc_state,
						    int_cwai_state, 
							int_nmimask_state, int_firqmask_state, int_swimask_state, int_irqmask_state, 
							// Return From Interrupt
						    rti_cc_state,   rti_entire_state,
							rti_acca_state, rti_accb_state,
						    rti_dp_state,
						    rti_ixl_state,  rti_ixh_state,
						    rti_iyl_state,  rti_iyh_state,
						    rti_upl_state,  rti_uph_state,
						    rti_pcl_state,  rti_pch_state,
							// Push Registers using SP
							pshs_state,
						    pshs_pcl_state,  pshs_pch_state,
						    pshs_upl_state,  pshs_uph_state,
						    pshs_iyl_state,  pshs_iyh_state,
						    pshs_ixl_state,  pshs_ixh_state,
						    pshs_dp_state,
						    pshs_acca_state, pshs_accb_state,
						    pshs_cc_state,
							// Pull Registers using SP
							puls_state,
							puls_cc_state,
							puls_acca_state, puls_accb_state,
							puls_dp_state,
						    puls_ixl_state,  puls_ixh_state,
						    puls_iyl_state,  puls_iyh_state,
						    puls_upl_state,  puls_uph_state,
						    puls_pcl_state,  puls_pch_state,
							// Push Registers using UP
							pshu_state,
						    pshu_pcl_state,  pshu_pch_state,
						    pshu_spl_state,  pshu_sph_state,
						    pshu_iyl_state,  pshu_iyh_state,
						    pshu_ixl_state,  pshu_ixh_state,
						    pshu_dp_state,
						    pshu_acca_state, pshu_accb_state,
						    pshu_cc_state,
							// Pull Registers using UP
							pulu_state,
							pulu_cc_state,
							pulu_acca_state, pulu_accb_state,
							pulu_dp_state,
						    pulu_ixl_state,  pulu_ixh_state,
						    pulu_iyl_state,  pulu_iyh_state,
						    pulu_spl_state,  pulu_sph_state,
						    pulu_pcl_state,  pulu_pch_state } state_type;

typedef enum logic[1:0] {reset_st, push_st, idle_st } st_type;
typedef enum logic[2:0] {latch_iv, swi3_iv, swi2_iv, firq_iv, irq_iv, swi_iv, nmi_iv, reset_iv} iv_type ;
typedef enum logic[3:0] {idle_ad, fetch_ad, read_ad, write_ad, pushu_ad, pullu_ad, pushs_ad, pulls_ad, int_hi_ad, int_lo_ad} addr_type;
typedef enum logic[3:0] {cc_dout, acca_dout, accb_dout, dp_dout,
                    ix_lo_dout, ix_hi_dout, iy_lo_dout, iy_hi_dout,
                    up_lo_dout, up_hi_dout, sp_lo_dout, sp_hi_dout,
                    pc_lo_dout, pc_hi_dout, md_lo_dout, md_hi_dout} dout_type;
typedef enum logic[1:0] {reset_op, fetch_op, latch_op} op_type;
typedef enum logic[1:0] {reset_pre, fetch_pre, load_pre, latch_pre} pre_type;
typedef enum logic[1:0] {reset_cc, load_cc, pull_cc, latch_cc} cc_type;
typedef enum logic[2:0] {reset_acca, load_acca, load_hi_acca, pull_acca, latch_acca} acca_type;
typedef enum logic[1:0] {reset_accb, load_accb, pull_accb, latch_accb} accb_type;
typedef enum logic[1:0] {reset_dp, load_dp, pull_dp, latch_dp} dp_type;
typedef enum logic[2:0] {reset_ix, load_ix, pull_lo_ix, pull_hi_ix, latch_ix} ix_type;
typedef enum logic[2:0] {reset_iy, load_iy, pull_lo_iy, pull_hi_iy, latch_iy} iy_type;
typedef enum logic[2:0] {reset_sp, latch_sp, load_sp, pull_hi_sp, pull_lo_sp} sp_type;
typedef enum logic[2:0] {reset_up, latch_up, load_up, pull_hi_up, pull_lo_up} up_type;
typedef enum logic[2:0] {reset_pc, latch_pc, load_pc, pull_lo_pc, pull_hi_pc, incr_pc} pc_type;
typedef enum logic[2:0] {reset_md, latch_md, load_md, fetch_first_md, fetch_next_md, shiftl_md} md_type;
typedef enum logic[2:0] {reset_ea, latch_ea, load_ea, fetch_first_ea, fetch_next_ea} ea_type;
typedef enum logic[3:0] {cc_left, acca_left, accb_left, dp_left,
						ix_left, iy_left, up_left, sp_left,
                    	accd_left, md_left, pc_left, ea_left} left_type;
typedef enum logic[3:0] {ea_right, zero_right, one_right, two_right,
                    acca_right, accb_right, accd_right,	md_right,
                    md_sign5_right, md_sign8_right} right_type;
typedef enum logic[5:0] {alu_add8, alu_sub8, alu_add16, alu_sub16, alu_adc, alu_sbc, alu_and, alu_ora,
					alu_eor, alu_tst, alu_inc, alu_dec, alu_clr, alu_neg, alu_com, alu_lsr16,
					alu_lsl16, alu_ror8, alu_rol8, alu_mul, alu_asr8, alu_asl8, alu_lsr8, alu_andcc,
					alu_orcc, alu_sex, alu_tfr, alu_abx, alu_seif, alu_sei, alu_see, alu_cle,
					alu_ld8, alu_st8, alu_ld16, alu_st16, alu_lea, alu_nop, alu_daa} alu_type;

logic[7:0] op_code;
logic[7:0] pre_code;
logic[7:0] acca;
logic[7:0] accb;
logic[7:0] cc;
logic[7:0] cc_out;
logic[7:0] dp;
logic[15:0] xreg;
logic[15:0] yreg;
logic[15:0] sp;
logic[15:0] up;
logic[15:0] ea;
logic[15:0] pc;
logic[15:0] md;
logic[15:0] left;
logic[15:0] right;
logic[15:0] out_alu;
logic[2:0] iv;
logic nmi_req;
logic nmi_ack;
logic nmi_enable;
logic fic; // first instruction cycle
logic lic; // last instruction cycle

state_type state;
state_type next_state;
state_type return_state;
state_type saved_state;
st_type st_ctrl;
iv_type iv_ctrl;
pc_type pc_ctrl;
ea_type ea_ctrl; 
op_type op_ctrl;
pre_type pre_ctrl;
md_type md_ctrl;
acca_type acca_ctrl;
accb_type accb_ctrl;
ix_type ix_ctrl;
iy_type iy_ctrl;
cc_type cc_ctrl;
dp_type dp_ctrl;
sp_type sp_ctrl;
up_type up_ctrl;
left_type left_ctrl;
right_type right_ctrl;
alu_type alu_ctrl;
addr_type addr_ctrl;
dout_type dout_ctrl;

//////////////////////////////////
//
// State machine stack
//
//////////////////////////////////
always @(posedge clk)
begin
    if (hold == 0)
    begin
		case (st_ctrl)
    	reset_st:
        	saved_state <= fetch_state;
    	push_st:
			saved_state <= return_state; 
		default: ;
 	   	endcase
    end
end

//////////////////////////////////
//
// Interrupt Vector control
//
//////////////////////////////////
//
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (iv_ctrl)
    	reset_iv:
        	iv <= RST_VEC;
    	nmi_iv:
        	iv <= NMI_VEC;
    	swi_iv:
        	iv <= SWI_VEC;
    	irq_iv:
        	iv <= IRQ_VEC;
    	firq_iv:
        	iv <= FIRQ_VEC;
    	swi2_iv:
        	iv <= SWI2_VEC;
    	swi3_iv:
        	iv <= SWI3_VEC;
		default: ;
    	endcase
    end
end
  
//////////////////////////////////
//
// Program Counter Control
//
//////////////////////////////////
//pc_reg: process( clk, pc_ctrl, hold, pc, out_alu, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (pc_ctrl)
		reset_pc:
			pc <= 0;
		load_pc:
			pc <= out_alu[15:0];
		pull_lo_pc:
			pc[7:0] <= data_in;
		pull_hi_pc:
			pc[15:8] <= data_in;
		incr_pc:
			pc <= pc + 16'h01;
		default: ;
    	endcase
	end
end

//////////////////////////////////
//
// Effective Address  Control
//
//////////////////////////////////

//ea_reg: process( clk, ea_ctrl, hold, ea, out_alu, data_in, dp )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (ea_ctrl)
	 	reset_ea:
			ea <= 0;
		fetch_first_ea:
		begin
			ea[7:0] <= data_in;
    		ea[15:8] <= dp;
    	end
  		fetch_next_ea:
  		begin
			ea[15:8] <= ea[7:0];
    		ea[7:0]  <= data_in;
    	end
		load_ea:
			ea <= out_alu[15:0];
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// Accumulator A
//
////////////////////////////////
//acca_reg : process( clk, acca_ctrl, hold, out_alu, acca, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (acca_ctrl)
    	reset_acca:
			acca <= 0;
		load_acca:
			acca <= out_alu[7:0];
		load_hi_acca:
			acca <= out_alu[15:8];
		pull_acca:
			acca <= data_in;
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// Accumulator B
//
////////////////////////////////
//accb_reg : process( clk, accb_ctrl, hold, out_alu, accb, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (accb_ctrl)
    	reset_accb:
			accb <= 0;
		load_accb:
			accb <= out_alu[7:0];
		pull_accb:
			accb <= data_in;
		default: ;
    endcase
	end
end

////////////////////////////////
//
// X Index register
//
////////////////////////////////
//ix_reg : process( clk, ix_ctrl, hold, out_alu, xreg, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (ix_ctrl)
    	reset_ix:
			xreg <= 0;
		load_ix:
			xreg <= out_alu[15:0];
		pull_hi_ix:
			xreg[15:8] <= data_in;
		pull_lo_ix:
			xreg[7:0] <= data_in;
		default: ;
    endcase
	end
end

////////////////////////////////
//
// Y Index register
//
////////////////////////////////
//iy_reg : process( clk, iy_ctrl, hold, out_alu, yreg, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (iy_ctrl)
    	reset_iy:
			yreg <= 0;
		load_iy:
			yreg <= out_alu[15:0];
		pull_hi_iy:
			yreg[15:8] <= data_in;
		pull_lo_iy:
			yreg[7:0] <= data_in;
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// S stack pointer
//
////////////////////////////////
//sp_reg : process( clk, sp_ctrl, hold, sp, out_alu, data_in, nmi_enable )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (sp_ctrl)
    	reset_sp:
    	begin
			sp <= 0;
			nmi_enable <= 1'b0;
		end
		load_sp:
		begin
			sp <= out_alu[15:0];
			nmi_enable <= 1'b1;
		end
		pull_hi_sp:
			sp[15:8] <= data_in;
		pull_lo_sp:
		begin
			sp[7:0] <= data_in;
			nmi_enable <= 1'b1;
		end
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// U stack pointer
//
////////////////////////////////
//up_reg : process( clk, up_ctrl, hold, up, out_alu, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (up_ctrl)
    	reset_up:
			up <= 0;
		load_up:
			up <= out_alu[15:0];
		pull_hi_up:
			up[15:8] <= data_in;
		pull_lo_up:
			up[7:0] <= data_in;
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// Memory Data
//
////////////////////////////////
//md_reg : process( clk, md_ctrl, hold, out_alu, data_in, md )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (md_ctrl)
    	reset_md:
			md <= 0;
		load_md:
			md <= out_alu[15:0];
		fetch_first_md: // sign extend md for branches
		begin
			md[15:8] <= {8{data_in[7]}};
			md[7:0] <= data_in;
		end
		fetch_next_md:
		begin
			md[15:8] <= md[7:0];
			md[7:0] <= data_in;
		end
		shiftl_md:
		begin
			md[15:1] <= md[14:0];
			md[0] <= 1'b0;
		end
		default: ;
    	endcase
	end
end

//////////////////////////////////
//
// Condition Codes
//
//////////////////////////////////
//cc_reg: process( clk, cc_ctrl, hold, cc_out, cc, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (cc_ctrl)
		reset_cc:
			cc <= 8'b11010000; // set EBIT, FBIT & IBIT
		load_cc:
			cc <= cc_out;
  		pull_cc:
    		cc <= data_in;
		default: ;
    	endcase
	end
end

//////////////////////////////////
//
// Direct Page register
//
//////////////////////////////////
//dp_reg: process( clk, dp_ctrl, hold, out_alu, dp, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (dp_ctrl)
		reset_dp:
			dp <= 0;
		load_dp:
			dp <= out_alu[7:0];
  		pull_dp:
    		dp <= data_in;
		default: ;
    	endcase
	end
end

//////////////////////////////////
//
// op code register
//
//////////////////////////////////
//op_reg: process( clk, op_ctrl, hold, op_code, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (op_ctrl)
		reset_op:
			op_code <= 8'b00010010;
  		fetch_op:
    		op_code <= data_in;
		default: ;
    	endcase
	end
end

//////////////////////////////////
//
// pre byte op code register
//
//////////////////////////////////
//pre_reg: process( clk, pre_ctrl, hold, pre_code, data_in )
always @(posedge clk)
begin
    if (hold == 0)
    begin
    	case (pre_ctrl)
		reset_pre:
			pre_code <= 0;
  		fetch_pre:
    		pre_code <= data_in;
    	load_pre:
			pre_code <= op_code;
		default: ;
    	endcase
	end
end

////////////////////////////////
//
// state machine
//
////////////////////////////////
//change_state: process( clk, rst, state, hold, next_state )
always @(posedge clk)
begin
    if (rst)
    begin
    	fic     <= 1'b0;
		nmi_ack <= 1'b0;
 		state   <= reset_state;
 	end
    else if (hold == 0)
    	begin
			fic <= lic;
			//
			// nmi request is not cleared until nmi input goes low
			//
			if (nmi_req == 1'b0 && nmi_ack == 1'b1)
        		nmi_ack <= 1'b0;
		 
			if (nmi_req == 1'b1 && nmi_ack == 1'b0  && state == int_nmimask_state)
        		nmi_ack <= 1'b1;

        	if (lic == 1'b1)
        	begin
        		if (halt == 1'b1)
		   			state <= halt_state;
        		// service non maskable interrupts
       			else if (nmi_req == 1'b1 && nmi_ack == 1'b0)
       				begin
		  				state <= int_nmi_state;
						//
						// FIRQ & IRQ are level sensitive
						//
					end
          		else if (firq == 1'b1 && cc[FBIT] == 1'b0)
					state  <= int_firq_state;

				else if (irq == 1'b1 && cc[IBIT] == 1'b0)
					state <= int_irq_state;
					//
					// Version 1.27 2015-05-30
					// Exit sync_state on masked interrupt.
					//
					// Version 1.28 2015-05-30
					// Move this code to the state sequencer
					// near line 5566.
					//
					// else if  (state = sync_state) and ((firq = 1'b1) or (irq = 1'b1))then
					//   state <= fetch_state;
          			//
          		else
					state <= next_state;
          	end // lic
          	else
		    	state <= next_state;
		end // hold
end // always block
	
////////////////////////////////////
//
// Detect Edge of NMI interrupt
//
////////////////////////////////////
//nmi_handler : process( clk, rst, nmi, nmi_ack, nmi_req, nmi_enable )
always @(posedge clk or posedge rst)
begin
	if (rst)
		nmi_req <= 1'b0;
	else
	begin
		if (nmi == 1'b1 && nmi_ack == 1'b0 && nmi_enable == 1'b1)
	    	nmi_req <= 1'b1;
	   	else if (nmi == 1'b0 && nmi_ack == 1'b1)
	    	nmi_req <= 1'b0;
	end
end

//////////////////////////////////
//
// Address output multiplexer
//
//////////////////////////////////
always_comb
begin
	ifetch = 1'b0;
	vma    = 1'b1;
	case (addr_ctrl)
    fetch_ad:
    begin
		addr   = pc;
    	rw     = 1'b1;
    	ifetch = 1'b1;
    end
	read_ad:
	begin
		addr   = ea;
    	rw     = 1'b1;
    end
    write_ad:
    begin
		addr   = ea;
		rw     = 1'b0;
	end
	pushs_ad:
	begin
		addr   = sp;
		rw     = 1'b0;
	end
    pulls_ad:
    begin
		addr   = sp;
    	rw     = 1'b1;
    end
	pushu_ad:
	begin
		addr   = up;
		rw     = 1'b0;
	end
    pullu_ad:
    begin
		addr   = up;
    	rw     = 1'b1;
    end
	int_hi_ad:
	begin
		addr   = {12'b111111111111, iv, 1'b0};
    	rw     = 1'b1;
    end
    int_lo_ad:
    begin
		addr   = {12'b111111111111, iv, 1'b1};
    	rw     = 1'b1;
    end
	default:
	begin
    	addr   = 16'b1111111111111111;
    	rw     = 1'b1;
    	vma    = 1'b0;
    end
	endcase
end

////////////////////////////////
//
// Data Bus output
//
////////////////////////////////
always_comb
begin
    unique case (dout_ctrl)
	cc_dout: // condition code register
		data_out = cc;
	acca_dout: // accumulator a
		data_out = acca;
	accb_dout: // accumulator b
		data_out = accb;
	dp_dout: // direct page register
		data_out = dp;
	ix_lo_dout: // X index reg
		data_out = xreg[7:0];
	ix_hi_dout: // X index reg
		data_out = xreg[15:8];
	iy_lo_dout: // Y index reg
		data_out = yreg[7:0];
	iy_hi_dout: // Y index reg
		data_out = yreg[15:8];
	up_lo_dout: // U stack pointer
		data_out = up[7:0];
	up_hi_dout: // U stack pointer
		data_out = up[15:8];
	sp_lo_dout: // S stack pointer
		data_out = sp[7:0];
	sp_hi_dout: // S stack pointer
		data_out = sp[15:8];
	md_lo_dout: // alu output
		data_out = md[7:0];
	md_hi_dout: // alu output
		data_out = md[15:8];
	pc_lo_dout: // low order pc
		data_out = pc[7:0];
	pc_hi_dout: // high order pc
		data_out = pc[15:8];
    endcase
end

//////////////////////////////////
//
// Left Mux
//
//////////////////////////////////
always_comb
begin
	case (left_ctrl)
	cc_left:
	begin
		left[15:8] = 8'b00000000;
		left[7:0]  = cc;
	end
	acca_left:
	begin
		left[15:8] = 8'b00000000;
		left[7:0]  = acca;
	end
	accb_left:
	begin
		left[15:8] = 8'b00000000;
		left[7:0]  = accb;
	end
	dp_left:
	begin
		left[15:8] = 8'b00000000;
		left[7:0]  = dp;
	end
	accd_left:
	begin
		left[15:8] = acca;
		left[7:0]  = accb;
	end
	md_left:
		left = md;
	ix_left:
		left = xreg;
	iy_left:
		left = yreg;
	sp_left:
		left = sp;
	up_left:
		left = up;
	pc_left:
		left = pc;
	default:
//	ea_left:
		left = ea;
    endcase
end

//////////////////////////////////
//
// Right Mux
//
//////////////////////////////////
always_comb
begin
	case (right_ctrl)
	ea_right:
		right = ea;
	zero_right:
		right = 16'b0000000000000000;
	one_right:
		right = 16'b0000000000000001;
	two_right:
		right = 16'b0000000000000010;
	acca_right:
	begin
		if (acca[7] == 1'b0)
			right = {8'b00000000, acca[7:0]};
		else
			right = {8'b11111111, acca[7:0]};
	end
	accb_right:
	begin
		if (accb[7] == 1'b0)
			right = {8'b00000000, accb[7:0]};
		else
			right = {8'b11111111, accb[7:0]};
	end
	accd_right:
		right = {acca, accb};
	md_sign5_right:
	begin
		if (md[4] == 1'b0)
			right = {11'b00000000000, md[4:0]};
		else
			right = {11'b11111111111, md[4:0]};
	end
	md_sign8_right:
	begin
		if (md[7] == 1'b0)
			right = {8'b00000000, md[7:0]};
		else
			right = {8'b11111111, md[7:0]};
	end
	default:
//	md_right:
		right = md;
    endcase
end

//////////////////////////////////
//
// Arithmetic Logic Unit
//
//////////////////////////////////

logic valid_lo, valid_hi;
logic carry_in;
logic[7:0] daa_reg;
always_comb
begin

	case (alu_ctrl)
  	alu_adc, alu_sbc, alu_rol8, alu_ror8:
		carry_in = cc[CBIT];
    alu_asr8:
		carry_in = left[7];
  	default:
		carry_in = 1'b0;
  	endcase

  valid_lo = (left[3:0] <= 9);
  valid_hi = (left[7:4] <= 9);

  //
  // CBIT HBIT VHI VLO DAA
  //    0    0   0   0 66 (!VHI : hi_nybble>8)
  //    0    0   0   1 60
  //    0    0   1   1 00
  //    0    0   1   0 06 ( VHI : hi_nybble<=8)
  //
  //    0    1   1   0 06
  //    0    1   1   1 06
  //    0    1   0   1 66
  //    0    1   0   0 66
  //
  //    1    1   0   0 66
  //    1    1   0   1 66
  //    1    1   1   1 66
  //    1    1   1   0 66
  //
  //    1    0   1   0 66
  //    1    0   1   1 60
  //    1    0   0   1 60
  //    1    0   0   0 66
  //
  // 66 = (!VHI & !VLO) + (CBIT & HBIT) + (HBIT & !VHI) + (CBIT & !VLO) 
  //    = (CBIT & (HBIT + !VLO)) + (!VHI & (HBIT + !VLO))
  //    = (!VLO & (CBIT + !VHI)) + (HBIT & (CBIT + !VHI))
  // 60 = (CBIT & !HBIT & VLO) + (!HBIT & !VHI & VLO) 
  //    = (!HBIT & VLO & (CBIT + !VHI))
  // 06 = (!CBIT & VHI & (!VLO + VHI)
  // 00 = (!CBIT & !HBIT & VHI & VLO)
  //
	if (cc[CBIT] == 1'b0)
	begin
    	// CBIT=0
    	if( cc[HBIT] == 1'b0 )
    	begin
	 		// HBIT=0
			if (valid_lo)
			begin
		  		// lo <= 9 (no overflow in low nybble)
		  		if (valid_hi)
		  		begin
		    		// hi <= 9 (no overflow in either low or high nybble)
		    		daa_reg = 8'b00000000;
		    	end
		  		else
		  		begin
		   			// hi > 9 (overflow in high nybble only)
		    		daa_reg = 8'b01100000;
		  		end
		  	end
			else
			begin
				// lo > 9 (overflow in low nybble)
				//
				// since there is already an overflow in the low nybble
				// you need to make room in the high nybble for the low nybble carry
				// so compare the high nybble with 8 rather than 9
				// if the high nybble is 9 there will be an overflow on the high nybble
				// after the decimal adjust which means it will roll over to an invalid BCD digit
				//
	    		if( left[7:4] <= 8 )
	    		begin
		    		// hi <= 8 (overflow in low nybble only)
		    		daa_reg = 8'b00000110;
		    	end
		  		else
		  		begin
		    		// hi > 8 (overflow in low and high nybble)
					daa_reg = 8'b01100110;
				end
			end
		end
    	else
    	begin
			// HBIT=1 (overflow in low nybble)
			if (valid_hi)
			begin
				// hi <= 9 (overflow in low nybble only)
				daa_reg = 8'b00000110;
			end
			else
			begin
				// hi > 9 (overflow in low and high nybble)
				daa_reg = 8'b01100110;
			end
		end
	end
	else
	begin
    	// CBIT=1 (carry => overflow in high nybble)
    	if ( cc[HBIT] == 1'b0 )
    	begin
			// HBIT=0 (half carry clear => may or may not be an overflow in the low nybble)
			if (valid_lo)
			begin
				// lo <=9  (overflow in high nybble only)
				daa_reg = 8'b01100000;
			end
	   		else
	   		begin
				// lo >9  (overflow in low and high nybble)
				daa_reg = 8'b01100110;
			end
		end
 		else
 		begin
			// HBIT=1 (overflow in low and high nybble)
			daa_reg = 8'b01100110;
		end
	end

	case (alu_ctrl)
  	alu_add8, alu_inc, alu_add16, alu_adc, alu_mul:
		out_alu = left + right + {15'b000000000000000, carry_in};
  	alu_sub8, alu_dec, alu_sub16, alu_sbc:
		out_alu = left - right - {15'b000000000000000, carry_in};
    alu_abx:
		out_alu = left + {8'b00000000, right[7:0]};
  	alu_and:
		out_alu   = left & right; 	// and/bit
  	alu_ora:
		out_alu   = left | right; 	// or
  	alu_eor:
		out_alu   = left ^ right; 	// eor/xor
  	alu_lsl16, alu_asl8, alu_rol8:
		out_alu   = {left[14:0], carry_in}; 	// rol8/asl8/lsl16
  	alu_lsr16:
		out_alu   = {carry_in, left[15:1]}; 	// lsr16
  	alu_lsr8, alu_asr8, alu_ror8:
		out_alu   = {8'b00000000, carry_in, left[7:1]}; 	// ror8/asr8/lsr8
  	alu_neg:
		out_alu   = right - left; 	// neg (right=0)
  	alu_com:
		out_alu   = ~left;
  	alu_clr, alu_ld8, alu_ld16, alu_lea:
		out_alu   = right; 	         // clr, ld
	alu_st8, alu_st16, alu_andcc, alu_orcc, alu_tfr:
		out_alu   = left;
	alu_daa:
		out_alu   = left + {8'b00000000, daa_reg};
	alu_sex:
		if (left[7] == 1'b0)
			out_alu = {8'b00000000, left[7:0]};
		else
			out_alu = {8'b11111111, left[7:0]};
  	default:
		out_alu   = left; // nop
    endcase

	//
	// carry bit
	//
    case (alu_ctrl)
  	alu_add8, alu_adc:
    	cc_out[CBIT] = (left[7] & right[7]) | (left[7] & ~out_alu[7]) | (right[7] & ~out_alu[7]);
  	alu_sub8, alu_sbc:
    	cc_out[CBIT] = ((~left[7]) & right[7]) | ((~left[7]) & out_alu[7]) | (right[7] & out_alu[7]);
  	alu_add16:
    	cc_out[CBIT] = (left[15] & right[15]) | (left[15] & ~out_alu[15]) | (right[15] & ~out_alu[15]);
  	alu_sub16:
    	cc_out[CBIT] = ((~left[15]) & right[15]) | ((~left[15]) & out_alu[15]) | (right[15] & out_alu[15]);
	alu_ror8, alu_lsr16, alu_lsr8, alu_asr8:
		cc_out[CBIT] = left[0];
	alu_rol8, alu_asl8:
		cc_out[CBIT] = left[7];
	alu_lsl16:
		cc_out[CBIT] = left[15];
	alu_com:
		cc_out[CBIT] = 1'b1;
	alu_neg, alu_clr:
		cc_out[CBIT] = out_alu[7] | out_alu[6] | out_alu[5] | out_alu[4] | out_alu[3] | out_alu[2] | out_alu[1] | out_alu[0];
    alu_mul:
		cc_out[CBIT] = out_alu[7];
    alu_daa:
		if (daa_reg[7:4] == 4'b0110)
			cc_out[CBIT] = 1'b1;
		else
			cc_out[CBIT] = 1'b0;
  	alu_andcc:
    	cc_out[CBIT] = left[CBIT] & cc[CBIT];
  	alu_orcc:
    	cc_out[CBIT] = left[CBIT] | cc[CBIT];
  	alu_tfr:
    	cc_out[CBIT] = left[CBIT];
  	default:
    	cc_out[CBIT] = cc[CBIT];
    endcase

	//
	// Zero flag
	//
    case (alu_ctrl)
  	alu_add8, alu_sub8,
	alu_adc, alu_sbc,
  	alu_and, alu_ora, alu_eor,
  	alu_inc, alu_dec, 
	alu_neg, alu_com, alu_clr,
	alu_rol8, alu_ror8, alu_asr8, alu_asl8, alu_lsr8,
	alu_ld8, alu_st8, alu_sex, alu_daa:
    	cc_out[ZBIT] = ~(out_alu[7] | out_alu[6] | out_alu[5] | out_alu[4] | out_alu[3] | out_alu[2] | out_alu[1] | out_alu[0]);
  	alu_add16, alu_sub16, alu_mul,
  	alu_lsl16, alu_lsr16,
	alu_ld16, alu_st16, alu_lea:
    	cc_out[ZBIT] = ~(out_alu[15] | out_alu[14] | out_alu[13] | out_alu[12] |
	                    out_alu[11] | out_alu[10] | out_alu[9] | out_alu[8] |
  	                    out_alu[7] | out_alu[6] | out_alu[5] | out_alu[4] |
	                    out_alu[3] | out_alu[2] | out_alu[1] | out_alu[0]);
  	alu_andcc:
    	cc_out[ZBIT] = left[ZBIT] & cc[ZBIT];
  	alu_orcc:
    	cc_out[ZBIT] = left[ZBIT] | cc[ZBIT];
  	alu_tfr:
    	cc_out[ZBIT] = left[ZBIT];
  	default:
    	cc_out[ZBIT] = cc[ZBIT];
    endcase

    //
	// negative flag
	//
    case (alu_ctrl)
  	alu_add8, alu_sub8,
	alu_adc, alu_sbc,
	alu_and, alu_ora, alu_eor,
  	alu_rol8, alu_ror8, alu_asr8, alu_asl8, alu_lsr8,
  	alu_inc, alu_dec, alu_neg, alu_com, alu_clr,
	alu_ld8 , alu_st8, alu_sex, alu_daa:
    	cc_out[NBIT] = out_alu[7];
	alu_add16, alu_sub16,
	alu_lsl16, alu_lsr16,
	alu_ld16, alu_st16:
		cc_out[NBIT] = out_alu[15];
  	alu_andcc:
    	cc_out[NBIT] = left[NBIT] & cc[NBIT];
  	alu_orcc:
    	cc_out[NBIT] = left[NBIT] | cc[NBIT];
  	alu_tfr:
    	cc_out[NBIT] = left[NBIT];
  	default:
    	cc_out[NBIT] = cc[NBIT];
    endcase

    //
	// Interrupt mask flag
    //
    case (alu_ctrl)
  	alu_andcc:
    	cc_out[IBIT] = left[IBIT] & cc[IBIT];
  	alu_orcc:
    	cc_out[IBIT] = left[IBIT] | cc[IBIT];
  	alu_tfr:
    	cc_out[IBIT] = left[IBIT];
    alu_seif, alu_sei:
		cc_out[IBIT] = 1'b1;
  	default:
		cc_out[IBIT] = cc[IBIT];             // interrupt mask
    endcase

    //
    // Half Carry flag
	//
    case (alu_ctrl)
  	alu_add8, alu_adc:
    	cc_out[HBIT] = (left[3] & right[3]) | (right[3] & ~out_alu[3]) | (left[3] & ~out_alu[3]);
  	alu_andcc:
    	cc_out[HBIT] = left[HBIT] & cc[HBIT];
  	alu_orcc:
    	cc_out[HBIT] = left[HBIT] | cc[HBIT];
  	alu_tfr:
    	cc_out[HBIT] = left[HBIT];
  	default:
		cc_out[HBIT] = cc[HBIT];
    endcase

    //
    // Overflow flag
	//
    case (alu_ctrl)
  	alu_add8, alu_adc:
    	cc_out[VBIT] = (left[7] & right[7] & (~out_alu[7])) | ((~left[7]) & (~right[7]) & out_alu[7]);
	alu_sub8, alu_sbc:
    	cc_out[VBIT] = (left[7] & (~right[7]) & (~out_alu[7])) | ((~left[7]) & right[7] & out_alu[7]);
  	alu_add16:
    	cc_out[VBIT] = (left[15] & right[15] & (~out_alu[15])) | ((~left[15]) & (~right[15]) & out_alu[15]);
	alu_sub16:
    	cc_out[VBIT] = (left[15] & (~right[15]) & (~out_alu[15])) | ((~left[15]) & right[15] & out_alu[15]);
	alu_inc:
		cc_out[VBIT] = ((~left[7]) & left[6] & left[5] & left[4] & left[3] & left[2] & left[1] & left[0]);
	alu_dec, alu_neg:
		cc_out[VBIT] = (left[7] & (~left[6]) & (~left[5]) & (~left[4]) & (~left[3]) & (~left[2]) & (~left[1]) & (~left[0]));
// 6809 Programming reference manual says
// V not affected by ASR, LSR and ROR
// This is different to the 6800
// John Kent 6th June 2006
//	 alu_asr8 =>
//	   cc_out(VBIT) <= left(0) xor left(7);
//	 alu_lsr8 | alu_lsr16 =>
//	   cc_out(VBIT) <= left(0);
//	 alu_ror8 =>
//      cc_out(VBIT) <= left(0) xor cc(CBIT);
    alu_lsl16:
    	cc_out[VBIT] = left[15] ^ left[14];
	alu_rol8, alu_asl8:
    	cc_out[VBIT] = left[7] ^ left[6];
//
// 11th July 2006 - John Kent
// What DAA does with V is anyones guess
// It is undefined in the 6809 programming manual
//
	alu_daa:
    	cc_out[VBIT] = left[7] ^ out_alu[7] ^ cc[CBIT];
// CLR resets V Bit
// John Kent 6th June 2006
	alu_and, alu_ora, alu_eor, alu_com, alu_clr, alu_st8, alu_st16, alu_ld8, alu_ld16, alu_sex:
    	cc_out[VBIT] = 1'b0;
  	alu_andcc:
    	cc_out[VBIT] = left[VBIT] & cc[VBIT];
  	alu_orcc:
    	cc_out[VBIT] = left[VBIT] | cc[VBIT];
  	alu_tfr:
    	cc_out[VBIT] = left[VBIT];
  	default:
		cc_out[VBIT] = cc[VBIT];
    endcase

	case (alu_ctrl)
  	alu_andcc:
    	cc_out[FBIT] = left[FBIT] & cc[FBIT];
  	alu_orcc:
    	cc_out[FBIT] = left[FBIT] | cc[FBIT];
  	alu_tfr:
    	cc_out[FBIT] = left[FBIT];
    alu_seif:
		cc_out[FBIT] = 1'b1;
	default:
    	cc_out[FBIT] = cc[FBIT];
	endcase

	case (alu_ctrl)
  	alu_andcc:
    	cc_out[EBIT] = left[EBIT] & cc[EBIT];
  	alu_orcc:
    	cc_out[EBIT] = left[EBIT] | cc[EBIT];
  	alu_tfr:
    	cc_out[EBIT] = left[EBIT];
    alu_see:
		cc_out[EBIT] = 1'b1;
    alu_cle:
		cc_out[EBIT] = 1'b0;
	default:
		cc_out[EBIT] = cc[EBIT];
	endcase
end // always_comb

////////////////////////////////////
//
// state sequencer
//
////////////////////////////////////
logic cond_true;  // variable used to evaluate coditional branches
always_comb
begin
	cond_true  = 1'b1;
	ba         = 1'b0;
	bs         = 1'b0;
	lic        = 1'b0;
	opfetch    = 1'b0;
	iv_ctrl    = latch_iv;
	// Registers preserved
	cc_ctrl    = latch_cc;
	acca_ctrl  = latch_acca;
	accb_ctrl  = latch_accb;
	dp_ctrl    = latch_dp;
	ix_ctrl    = latch_ix;
	iy_ctrl    = latch_iy;
	up_ctrl    = latch_up;
	sp_ctrl    = latch_sp;
	pc_ctrl    = latch_pc;
	md_ctrl    = latch_md;
	ea_ctrl    = latch_ea;
	op_ctrl    = latch_op;
	pre_ctrl   = latch_pre;
	// ALU Idle
	left_ctrl  = pc_left;
	right_ctrl = zero_right;
	alu_ctrl   = alu_nop;
	// Bus idle
	addr_ctrl  = idle_ad;
	dout_ctrl  = cc_dout;
	// Next State Fetch
	st_ctrl      = idle_st;
	return_state = fetch_state;
	next_state   = fetch_state;

	case (state)
	reset_state:        //  released from reset
	begin
    	// reset the registers
    	iv_ctrl    = reset_iv;
    	op_ctrl    = reset_op;
    	pre_ctrl   = reset_pre;
    	cc_ctrl    = reset_cc;
    	acca_ctrl  = reset_acca;
    	accb_ctrl  = reset_accb;
    	dp_ctrl    = reset_dp;
    	ix_ctrl    = reset_ix;
    	iy_ctrl    = reset_iy;
    	up_ctrl    = reset_up;
    	sp_ctrl    = reset_sp;
    	pc_ctrl    = reset_pc;
    	ea_ctrl    = reset_ea;
    	md_ctrl    = reset_md;
    	st_ctrl    = reset_st;
    	next_state = vect_hi_state;
    end

	//
	// Jump via interrupt vector
	// iv holds interrupt type
	// fetch PC hi from vector location
	//
	vect_hi_state:
	begin
    	// fetch pc low interrupt vector
    	pc_ctrl    = pull_hi_pc;
    	addr_ctrl  = int_hi_ad;
    	bs         = 1'b1;
    	next_state = vect_lo_state;
    end

	//
	// jump via interrupt vector
	// iv holds vector type
	// fetch PC lo from vector location
	//
	vect_lo_state:
	begin
    	// fetch the vector low byte
    	pc_ctrl    = pull_lo_pc;
    	addr_ctrl  = int_lo_ad;
    	bs         = 1'b1;
    	next_state = fetch_state;
    end

	vect_idle_state:
	begin
    	//
    	// Last Instruction Cycle for SWI, SWI2 & SWI3
    	//
    	if (op_code == 8'b00111111)
    		lic      = 1'b1;
    	next_state = fetch_state;
    end

	//
	// Here to fetch an instruction
	// PC points to opcode
	//
	fetch_state:
	begin
    	// fetch the op code
    	opfetch    = 1'b1;
    	op_ctrl    = fetch_op;
    	pre_ctrl   = fetch_pre;
    	ea_ctrl    = reset_ea;
    	// Fetch op code
    	addr_ctrl  = fetch_ad;
    	// Advance the PC to fetch next instruction byte
    	pc_ctrl    = incr_pc;
    	next_state = decode_state;
    end

	//
	// Here to decode instruction
	// and fetch next byte of intruction
	// whether it be necessary or not
	//
	decode_state:
	begin
		// fetch first byte of address or immediate data
    	ea_ctrl    = fetch_first_ea;
    	md_ctrl    = fetch_first_md;
    	addr_ctrl  = fetch_ad;
    	case (op_code[7:4])
    	//
    	// direct single op (2 bytes)
    	// 6809 => 6 cycles
    	// cpu09 => 5 cycles
    	// 1 op=(pc) / pc=pc+1
    	// 2 ea_hi=dp / ea_lo=(pc) / pc=pc+1
    	// 3 md_lo=(ea) / pc=pc
    	// 4 alu_left=md / md=alu_out / pc=pc
    	// 5 (ea)=md_lo / pc=pc
    	//
    	// Exception is JMP
    	// 6809 => 3 cycles
    	// cpu09 => 3 cycles
    	// 1 op=(pc) / pc=pc+1
    	// 2 ea_hi=dp / ea_lo=(pc) / pc=pc+1
    	// 3 pc=ea
               //
		4'b0000: 
		begin
    		// advance the PC
    		pc_ctrl    = incr_pc;

    		case (op_code[3:0])
    		4'b1110: // jmp
        		next_state = jmp_state;

    		4'b1111: // clr
        		next_state = single_op_exec_state;

    		default:
        		next_state = single_op_read_state;
    		endcase
    	end

    	// acca / accb inherent instructions
		4'b0001:
		begin
    		case (op_code[3:0])
    		//
    		// Page2 pre byte
    		// pre=(pc) / pc=pc+1
    		// op=(pc) / pc=pc+1
    		//
    		4'b0000: // page2
    		begin
        		opfetch    = 1'b1;
        		op_ctrl    = fetch_op;
				pre_ctrl   = load_pre;
        		// advance pc
        		pc_ctrl    = incr_pc;
        		next_state = decode_state;
        	end

    			//
    			// Page3 pre byte
    			// pre=(pc) / pc=pc+1
    			// op=(pc) / pc=pc+1
    			//
    		4'b0001: // page3
    		begin
        		opfetch    = 1'b1;
        		op_ctrl    = fetch_op;
				pre_ctrl   = load_pre;
        		// advance pc
        		pc_ctrl    = incr_pc;
        		next_state = decode_state;
        	end

    			//
    			// nop - No operation ( 1 byte )
    			// 6809 => 2 cycles
    			// cpu09 => 2 cycles
    			// 1 op=(pc) / pc=pc+1
    			// 2 decode
    			// 
    		4'b0010: // nop
    		begin
        		lic          = 1'b1;
        		next_state   = fetch_state;
        	end

    			//
    			// sync - halt execution until an interrupt is received
     			// interrupt may be NMI, IRQ or FIRQ
    			// program execution continues if the 
    			// interrupt is asserted for 3 clock cycles
    			// note that registers are not pushed onto the stack
    			// CPU09 => Interrupts need only be asserted for one clock cycle
    			//
    		4'b0011: // sync
    		begin
        		next_state   = sync_state;
        	end

    			//
    			// lbra // long branch (3 bytes)
    			// 6809 => 5 cycles
    			// cpu09 => 4 cycles
    			// 1 op=(pc) / pc=pc+1
    			// 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1
    			// 3 md_hi=md_lo / md_lo=(pc) / pc=pc+1
    			// 4 pc=pc+md
    			//
    		4'b0110:
    		begin
        		// increment the pc
        		pc_ctrl    = incr_pc;
        		next_state = lbranch_state;
        	end

    			//
    			// lbsr - long branch to subroutine (3 bytes)
    			// 6809 => 9 cycles
    			// cpu09 => 6 cycles
    			// 1 op=(pc) /pc=pc+1
    			// 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1 / sp=sp-1
    			// 3 md_hi=md_lo / md_lo=(pc) / pc=pc+1
    			// 4 (sp)= pc_lo / sp=sp-1 / pc=pc
    			// 5 (sp)=pc_hi / pc=pc
    			// 6 pc=pc+md
    			//
    		4'b0111:
    		begin
        		// pre decrement sp
        		left_ctrl  = sp_left;
        		right_ctrl = one_right;
        		alu_ctrl   = alu_sub16;
        		sp_ctrl    = load_sp;
        		// increment the pc
        		pc_ctrl    = incr_pc;
        		next_state = lbranch_state;
        	end

    			//
    			// Decimal Adjust Accumulator
    			//
    		4'b1001: // daa
    		begin
        		left_ctrl  = acca_left;
        		right_ctrl = accb_right;
        		alu_ctrl   = alu_daa;
        		cc_ctrl    = load_cc;
        		acca_ctrl  = load_acca;
        		lic        = 1'b1;
        		next_state = fetch_state;
        	end

    			//
    			// OR Condition Codes
    			//
    		4'b1010: // orcc
    		begin
        		// increment the pc
        		pc_ctrl      = incr_pc;
        		next_state   = orcc_state;
        	end

    			//
    			// AND Condition Codes
    			//
    		4'b1100: // andcc
    		begin
        		// increment the pc
        		pc_ctrl      = incr_pc;
        		next_state   = andcc_state;
        	end

    			//
    			// Sign Extend
    			//
    		4'b1101: // sex
    		begin
        		left_ctrl  = accb_left;
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_sex;
        		cc_ctrl    = load_cc;
        		acca_ctrl  = load_hi_acca;
        		lic        = 1'b1;
        		next_state = fetch_state;
        	end

    			//
    			// Exchange Registers
    			//
    		4'b1110: // exg
    		begin
        		// increment the pc
        		pc_ctrl    = incr_pc;
        		next_state = exg_state;
        	end

    			//
    			// Transfer Registers
    			//
    		4'b1111: // tfr
    		begin
        		// increment the pc
        		pc_ctrl      = incr_pc;
        		next_state   = tfr_state;
        	end

    		default:
    		begin
        		// increment the pc
        		pc_ctrl    = incr_pc;
        		lic        = 1'b1;
        		next_state = fetch_state;
        	end
    		endcase
    	end

		//
		// conditional branch
		//
    	4'b0010: // branch conditional
    	begin
    		// increment the pc
    		pc_ctrl    = incr_pc;
			case (pre_code)
			8'b00010000: // page 2
				//
				// lbcc // long branch conditional
				// 6809 => branch 6 cycles, no branch 5 cycles
				// cpu09 => always 5 cycles
				// 1 pre=(pc) / pc=pc+1
				// 2 op=(pc) / pc=pc+1
				// 3 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1
        		// 4 md_hi=md_lo / md_lo=(pc) / pc=pc+1
				// 5 if cond pc=pc+md else pc=pc
				//
				next_state = lbranch_state;
			
			default: // page 1
				//
				// Short branch conditional
				// 6809 => always 3 cycles
				// cpu09 => always == 3 cycles
				// 1 op=(pc) / pc=pc+1
				// 2 md_hi=sign(pc) / md_lo=(pc) / pc=pc+1 / test cc
				// 3 if cc tru pc=pc+md else pc=pc
				//
				next_state = sbranch_state;
			endcase
		end
		
    	//
    	// Single byte stack operators
    	// Do not advance PC
    	//
    	4'b0011:
    	begin
    		//
    		// lea - load effective address (2+ bytes)
    		// 6809 => 4 cycles + addressing mode
    		// cpu09 => 4 cycles + addressing mode
    		// 1 op=(pc) / pc=pc+1
    		// 2 md_lo=(pc) / pc=pc+1
    		// 3 calculate ea
    		// 4 ix/iy/sp/up == ea
    		//
    		case (op_code[3:0])
			4'b0000,  // leax
        	4'b0001,  // leay
        	4'b0010,  // leas
        	4'b0011: // leau
        	begin
        		// advance PC
        		pc_ctrl      = incr_pc;
        		st_ctrl      = push_st;
        		return_state = lea_state;
        		next_state   = indexed_state;
        	end

				//
				// pshs - push registers onto sp stack
				// 6809 => 5 cycles + registers
				// cpu09 => 3 cycles + registers
				//  1 op=(pc) / pc=pc+1
				//  2 ea_lo=(pc) / pc=pc+1 
				//  3 if ea(7:0) != "00000000" then sp=sp-1
				//  4 if ea(7) == 1 (sp)=pcl, sp=sp-1
				//  5 if ea(7) == 1 (sp)=pch
				//    if ea(6:0) != "0000000" then sp=sp-1
				//  6 if ea(6) == 1 (sp)=upl, sp=sp-1
				//  7 if ea(6) == 1 (sp)=uph
				//    if ea(5:0) != "000000" then sp=sp-1
				//  8 if ea(5) == 1 (sp)=iyl, sp=sp-1
				//  9 if ea(5) == 1 (sp)=iyh
				//    if ea(4:0) != "00000" then sp=sp-1
				// 10 if ea(4) == 1 (sp)=ixl, sp=sp-1
				// 11 if ea(4) == 1 (sp)=ixh
				//    if ea(3:0) != "0000" then sp=sp-1
				// 12 if ea(3) == 1 (sp)=dp
				//    if ea(2:0) != "000" then sp=sp-1
				// 13 if ea(2) == 1 (sp)=accb
				//    if ea(1:0) != "00" then sp=sp-1
				// 14 if ea(1) == 1 (sp)=acca
				//    if ea(0:0) != "0" then sp=sp-1
				// 15 if ea(0) == 1 (sp)=cc
				//
    		4'b0100: // pshs
    		begin
        		// advance PC
        		pc_ctrl    = incr_pc;
        		next_state = pshs_state;
        	end

				//
				// puls - pull registers of sp stack
				// 6809 => 5 cycles + registers
				// cpu09 => 3 cycles + registers
				//
    		4'b0101: // puls
    		begin
        		// advance PC
        		pc_ctrl    = incr_pc;
        		next_state = puls_state;
        	end

				//
				// pshu - push registers onto up stack
				// 6809 => 5 cycles + registers
				// cpu09 => 3 cycles + registers
				//
    		4'b0110: // pshu
    		begin
        		// advance PC
        		pc_ctrl    = incr_pc;
        		next_state = pshu_state;
        	end
        
				//
				// pulu - pull registers of up stack
				// 6809 => 5 cycles + registers
				// cpu09 => 3 cycles + registers
				//
    		4'b0111: // pulu
    		begin
        		// advance PC
        		pc_ctrl    = incr_pc;
        		next_state = pulu_state;
        	end

				//
				// rts - return from subroutine
				// 6809 => 5 cycles
				// cpu09 => 4 cycles 
				// 1 op=(pc) / pc=pc+1
				// 2 decode op
				// 3 pc_hi == (sp) / sp=sp+1
				// 4 pc_lo == (sp) / sp=sp+1
				//
    		4'b1001:
    		begin
  				next_state   = pull_return_hi_state;
  			end

				//
				// ADD accb to index register
				// *** Note: this is an unsigned addition.
				//           does not affect any condition codes
				// 6809 => 3 cycles
				// cpu09 => 2 cycles
				// 1 op=(pc) / pc=pc+1
				// 2 alu_left=ix / alu_right=accb / ix=alu_out / pc=pc
				//
    		4'b1010: // abx
    		begin
        		left_ctrl    = ix_left;
        		right_ctrl   = accb_right;
        		alu_ctrl     = alu_abx;
        		ix_ctrl      = load_ix;
        		lic          = 1'b1;
        		next_state   = fetch_state;
        	end

				//
				// Return From Interrupt
				//
			4'b1011: // rti
			begin
				next_state   = rti_cc_state;
			end

				//
				// CWAI
				//
			4'b1100: // cwai #$<cc_mask>
			begin
				// pre decrement sp
				left_ctrl    = sp_left;
				right_ctrl   = one_right;
				alu_ctrl     = alu_sub16;
				sp_ctrl      = load_sp;
				// increment pc
				pc_ctrl      = incr_pc;
				next_state   = cwai_state;
			end

				//
				// MUL Multiply
				//
    		4'b1101: // mul
    		begin
				// move acca to md
				left_ctrl  = acca_left;
				right_ctrl = zero_right;
				alu_ctrl   = alu_st16;
				md_ctrl    = load_md; // over ride md_ctrl <= second byte fetch 
				next_state = mulea_state;
			end
			
			    //
			    // SWI Software Interrupt
				// Do not advance PC
			    //
    		4'b1111: // swi
    		begin
        		// predecrement SP
        		left_ctrl    = sp_left;
        		right_ctrl   = one_right;
        		alu_ctrl     = alu_sub16;
        		sp_ctrl      = load_sp;
        		st_ctrl      = push_st;
		  		case (pre_code)
		  		8'b00010000: // page 2
		  		begin
					iv_ctrl      = swi2_iv;
					return_state = vect_hi_state;
				end
				
				8'b00010001: // page 3
				begin
					iv_ctrl 	= swi3_iv;
					return_state = vect_hi_state;
				end
				
				default: // page 1
				begin
					iv_ctrl  = swi_iv;
					return_state = int_swimask_state;
				end
				endcase
        		next_state   = int_entire_state;
        	end

    		default:
    		begin
        		lic          = 1'b1;
        		next_state   = fetch_state;
        	end
    		endcase
    	end
			//
		    // Accumulator A Single operand
		    // source == acca, dest == acca
		    // Do not advance PC
		    // Typically 2 cycles 1 bytes
		    // 1 opcode fetch
		    // 2 post byte fetch / instruction decode
		    // Note that there is no post byte
		    // so do not advance PC in decode cycle
		    // Re-run opcode fetch cycle after decode
		    // 
    	4'b0100: // acca single op
    	begin
    		left_ctrl  = acca_left;
    		case (op_code[3:0])
 
    		4'b0000: // neg
    		begin
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_neg;
        		acca_ctrl  = load_acca;
        		cc_ctrl    = load_cc;
        	end
 
    		4'b0011: // com
    		begin
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_com;
        		acca_ctrl  = load_acca;
        		cc_ctrl    = load_cc;
        	end

    		4'b0100: // lsr
    		begin
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_lsr8;
        		acca_ctrl  = load_acca;
        		cc_ctrl    = load_cc;
        	end

			4'b0110: // ror
			begin
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_ror8;
        		acca_ctrl  = load_acca;
        		cc_ctrl    = load_cc;
        	end

    		4'b0111: // asr
    		begin
        		right_ctrl = zero_right;
        		alu_ctrl   = alu_asr8;
        		acca_ctrl  = load_acca;
        		cc_ctrl    = load_cc;
        	end

    		4'b1000: // asl
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_asl8;
		        acca_ctrl  = load_acca;
		        cc_ctrl    = load_cc;
		    end

    		4'b1001: // rol
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_rol8;
		        acca_ctrl  = load_acca;
		        cc_ctrl    = load_cc;
		    end

    		4'b1010: // dec
    		begin
		        right_ctrl = one_right;
		        alu_ctrl   = alu_dec;
		        acca_ctrl  = load_acca;
		        cc_ctrl    = load_cc;
		    end

    		4'b1011: // undefined
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        acca_ctrl  = latch_acca;
		        cc_ctrl    = latch_cc;
		    end

    		4'b1100: // inc
    		begin
		        right_ctrl = one_right;
		        alu_ctrl   = alu_inc;
		        acca_ctrl  = load_acca;
		        cc_ctrl    = load_cc;
		    end

    		4'b1101: // tst
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_st8;
		        acca_ctrl  = latch_acca;
		        cc_ctrl    = load_cc;
		    end

    		4'b1110: // jmp (not defined)
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        acca_ctrl  = latch_acca;
		        cc_ctrl    = latch_cc;
		    end

    		4'b1111: // clr
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_clr;
		        acca_ctrl  = load_acca;
		        cc_ctrl    = load_cc;
		    end

    		default:
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        acca_ctrl  = latch_acca;
		        cc_ctrl    = latch_cc;
		    end

    		endcase
    		lic        = 1'b1;
    		next_state = fetch_state;
    	end

		//
		// Single Operand accb
		// source == accb, dest == accb
		// Typically 2 cycles 1 bytes
		// 1 opcode fetch
		// 2 post byte fetch / instruction decode
		// Note that there is no post byte
		// so do not advance PC in decode cycle
		// Re-run opcode fetch cycle after decode
		//
    	4'b0101:
    	begin
			left_ctrl  = accb_left;
			case (op_code[3:0])
			4'b0000: // neg
			begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_neg;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b0011: // com
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_com;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b0100: // lsr
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_lsr8;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b0110: // ror
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_ror8;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b0111: // asr
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_asr8;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1000: // asl
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_asl8;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1001: // rol
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_rol8;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1010: // dec
    		begin
		        right_ctrl = one_right;
		        alu_ctrl   = alu_dec;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1011: // undefined
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        accb_ctrl  = latch_accb;
		        cc_ctrl    = latch_cc;
		    end

    		4'b1100: // inc
    		begin
		        right_ctrl = one_right;
		        alu_ctrl   = alu_inc;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1101: // tst
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_st8;
		        accb_ctrl  = latch_accb;
		        cc_ctrl    = load_cc;
		    end

    		4'b1110: // jmp (undefined)
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        accb_ctrl  = latch_accb;
		        cc_ctrl    = latch_cc;
		    end

    		4'b1111: // clr
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_clr;
		        accb_ctrl  = load_accb;
		        cc_ctrl    = load_cc;
		    end

    		default:
    		begin
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        accb_ctrl  = latch_accb;
		        cc_ctrl    = latch_cc;
		    end
    		endcase
			lic          = 1'b1;
			next_state   = fetch_state;
		end

		    //
		    // Single operand indexed
		    // Two byte instruction so advance PC
		    // EA should hold index offset
		    //
    	4'b0110: // indexed single op
    	begin
			// increment the pc 
			pc_ctrl    = incr_pc;
			st_ctrl    = push_st;
 
    		case (op_code[3:0])
    		 4'b1110: // jmp
        		return_state = jmp_state;

    		 4'b1111: // clr
        		return_state = single_op_exec_state;

    		default:
        		return_state = single_op_read_state;

    		endcase
    		next_state = indexed_state;
    	end

		    //
		    // Single operand extended addressing
		    // three byte instruction so advance the PC
		    // Low order EA holds high order address
		    //
    	4'b0111: // extended single op
    	begin
			// increment PC
			pc_ctrl    = incr_pc;
			st_ctrl    = push_st;

    		case (op_code[3:0])
    		 4'b1110: // jmp
     			return_state = jmp_state;
  
    		 4'b1111: // clr
    			return_state = single_op_exec_state;
  
    		default:
        		return_state = single_op_read_state;
       
    		endcase
    		next_state = extended_state;
    	end

    	4'b1000: // acca immediate
    	begin
    		// increment the pc
    		pc_ctrl    = incr_pc;

    		case (op_code[3:0])
    		4'b0011, // subd #, cmpd #, cmpu #
        	4'b1100, // cmpx #, cmpy #, cmps #
        	4'b1110: // ldx #,  ldy #, undef #
        	begin
				next_state   = imm16_state;
			end

				//
				// bsr offset - Branch to subroutine (2 bytes)
				// 6809 => 7 cycles
				// cpu09 => 5 cycles
				// 1 op=(pc) / pc=pc+1
				// 2 md_hi=sign(pc) / md_lo=(pc) / sp=sp-1 / pc=pc+1
				// 3 (sp)=pc_lo / sp=sp-1
				// 4 (sp)=pc_hi
				// 5 pc=pc+md
				//
    		4'b1101: // bsr
    		begin
		        // pre decrement SP
		        left_ctrl  = sp_left;
		        right_ctrl = one_right;
		        alu_ctrl   = alu_sub16;
		        sp_ctrl    = load_sp;
			    //
			    st_ctrl      = push_st;
		        return_state = sbranch_state;
		        next_state   = push_return_lo_state;
		    end

    		default:
    		begin
        		lic          = 1'b1;
        		next_state   = fetch_state;
        	end
    		endcase
    	end

    	4'b1001: // acca direct
    	begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // subd <,  cmpd <, cmpu <
			4'b1100, // cmpx <,  cmpy <, cmps <
			4'b1110: // ldx <,   ldy <, undef <
			begin
        		next_state   = dual_op_read16_state;
        	end

    		4'b0111:  // sta direct
    		begin
        		next_state = dual_op_write8_state;
        	end

				//
				// jsr direct - Jump to subroutine in direct page (2 bytes)
				// 6809 => 7 cycles
				// cpu09 => 5 cycles
				// 1 op=(pc) / pc=pc+1
				// 2 ea_hi=0 / ea_lo=(pc) / sp=sp-1 / pc=pc+1
				// 3 (sp)=pc_lo / sp=sp-1
				// 4 (sp)=pc_hi
				// 5 pc=ea
				//
    		4'b1101: // jsr direct
    		begin
		        // pre decrement sp
		        left_ctrl  = sp_left;
		        right_ctrl = one_right;
		        alu_ctrl   = alu_sub16;
		        sp_ctrl    = load_sp;
		        //
		        st_ctrl      = push_st;
		        return_state = jmp_state;
		        next_state   = push_return_lo_state;
		    end

    		4'b1111: // stx <, sty <, undef <
    		begin
		        // idle ALU
		        left_ctrl  = ix_left;
		        right_ctrl = zero_right;
		        alu_ctrl   = alu_nop;
		        cc_ctrl    = latch_cc;
		        sp_ctrl    = latch_sp;
		        next_state = dual_op_write16_state;
		    end

    		default:
    		begin
        		next_state   = dual_op_read8_state;
        	end
    		endcase
    	end

    	4'b1010: // acca indexed
    	begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // subd x, cmpd x, cmpu x
			4'b1100, // cmpx x, cmpy x. cmps ,x
			4'b1110: // ldx x,  ldy x, undef x
			begin
		        st_ctrl      = push_st;
		        return_state = dual_op_read16_state;
		        next_state   = indexed_state;
		    end
		  
    		4'b0111:  // staa ,x
    		begin
		        st_ctrl      = push_st;
		        return_state = dual_op_write8_state;
		        next_state   = indexed_state;
		    end

    		4'b1101: // jsr ,x
    		begin
		        // DO NOT pre decrement SP
		        st_ctrl      = push_st;
		        return_state = jsr_state;
		        next_state   = indexed_state;
		    end

    		4'b1111: // stx ind, sty ind
    		begin
		        st_ctrl      = push_st;
		        return_state = dual_op_write16_state;
		        next_state   = indexed_state;
		    end

    		default:
    		begin
		        st_ctrl      = push_st;
		        return_state = dual_op_read8_state;
		        next_state   = indexed_state;
		    end
    		endcase
    	end

		4'b1011: // acca extended
		begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // subd >, cmpd >, cmpu >
			4'b1100, // cmpx >, cmpy >, cmps >
			4'b1110: // ldx >   ldy >, undef >
			begin
    			st_ctrl      = push_st;
    			return_state = dual_op_read16_state;
    			next_state   = extended_state;
    		end

    		4'b0111:  // staa >
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write8_state;
				next_state   = extended_state;
			end

    		4'b1101: // jsr >extended
    		begin
				// DO NOT pre decrement sp
				st_ctrl      = push_st;
				return_state = jsr_state;
				next_state   = extended_state;
			end

    		4'b1111: // stx >, sty >
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write16_state;
				next_state   = extended_state;
			end

    		default:
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_read8_state;
				next_state   = extended_state;
			end
    		endcase
    	end

		4'b1100: // accb immediate
		begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // addd #, undef #, undef #
			4'b1100, // ldd #, undef #, undef #
			4'b1110: // ldu #, lds #, undef #
			begin
    			next_state   = imm16_state;
    		end

			default:
			begin
				lic          = 1'b1;
				next_state   = fetch_state;
			end
			endcase
		end

		4'b1101: // accb direct
		begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // addd <, undef <, undef <
			4'b1100, // ldd <, undef <, undef <
			4'b1110: // ldu, lds <, undef <
				next_state   = dual_op_read16_state;

			4'b0111:  // stab <
    			next_state   = dual_op_write8_state;

    		4'b1101:  // std <,
    			next_state   = dual_op_write16_state;

    		4'b1111: // stu <, sts <
    			next_state   = dual_op_write16_state;

    		default:
    			next_state   = dual_op_read8_state;
    		endcase
    	end

		4'b1110: // accb indexed
		begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // addd x, undef x, undef x
			4'b1100, // ldd x, undef x, undef x
			4'b1110: // ldu x, lds x, undef x
			begin
				st_ctrl      = push_st;
				return_state = dual_op_read16_state;
				next_state   = indexed_state;
			end

    		4'b0111:  // stab x
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write8_state;
				next_state   = indexed_state;
			end

    		4'b1101:  // std x
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write16_state;
				next_state   = indexed_state;
			end

    		4'b1111: // stu x, sts x, undef x
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write16_state;
				next_state   = indexed_state;
			end

			default:
			begin
				st_ctrl      = push_st;
				return_state = dual_op_read8_state;
				next_state   = indexed_state;
			end 
    		endcase
    	end

		4'b1111: // accb extended
		begin
			// increment the pc
			pc_ctrl    = incr_pc;
			case (op_code[3:0])
			4'b0011, // addd >, undef >, undef >
			4'b1100, // ldd >, undef >, undef >
			4'b1110: // ldu >, lds >, undef >
			begin
				st_ctrl      = push_st;
				return_state = dual_op_read16_state;
				next_state   = extended_state;
			end

    		4'b0111:  // stab extended
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write8_state;
				next_state   = extended_state;
			end

    		4'b1101:  // std extended
    		begin
				st_ctrl      = push_st;
				return_state = dual_op_write16_state;
				next_state   = extended_state;
			end

			4'b1111: // stu  extended
			begin
				st_ctrl      = push_st;
				return_state = dual_op_write16_state;
				next_state   = extended_state;
			end

			default:
			begin
			    st_ctrl      = push_st;
			    return_state = dual_op_read8_state;
			    next_state   = extended_state;
			end
    		endcase
    	end
		//
		// not sure why I need this
		//	
		default:
		begin
			lic = 1'b1;
			next_state = fetch_state;
		end
		endcase
	end

		//
		// here if ea holds low byte
		// Direct
		// Extended
		// Indexed
		// read memory location
		//
	single_op_read_state:
	begin
		// read memory into md
		md_ctrl    = fetch_first_md;
		addr_ctrl  = read_ad;
		dout_ctrl  = md_lo_dout;
		next_state = single_op_exec_state;
	end

    single_op_exec_state:
    begin
        case (op_code[3:0])
        4'b0000: // neg
        begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_neg;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
        4'b0011: // com
        begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_com;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
        4'b0100: // lsr
        begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_lsr8;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
        4'b0110: // ror
        begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_ror8;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
        4'b0111: // asr
        begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_asr8;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		4'b1000: // asl
		begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_asl8;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		4'b1001: // rol
		begin
			left_ctrl  = md_left;
			 right_ctrl = zero_right;
			alu_ctrl   = alu_rol8;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		4'b1010: // dec
		begin
			left_ctrl  = md_left;
			right_ctrl = one_right;
			alu_ctrl   = alu_dec;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		4'b1011: // undefined
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
		4'b1100: // inc
		begin
			left_ctrl  = md_left;
			right_ctrl = one_right;
			alu_ctrl   = alu_inc;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		4'b1101: // tst
		begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_st8;
			cc_ctrl    = load_cc;
			lic        = 1'b1;
			next_state = fetch_state;
		end
		4'b1110: // jmp
		begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_ld16;
			pc_ctrl    = load_pc;
			lic          = 1'b1;
			next_state = fetch_state;
		end
		4'b1111: // clr
		begin
			left_ctrl  = md_left;
			right_ctrl = zero_right;
			alu_ctrl   = alu_clr;
			cc_ctrl    = load_cc;
			md_ctrl    = load_md;
			next_state = single_op_write_state;
		end
		default:
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
		endcase
	end
		//
		// single operand 8 bit write
		// Write low 8 bits of ALU output
		// EA holds address
		// MD holds data
		//
	single_op_write_state:
	begin
		// write ALU low byte output
		addr_ctrl  = write_ad;
		dout_ctrl  = md_lo_dout;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// here if ea holds address of low byte
		// read memory location
		//
	dual_op_read8_state:
	begin
		// read first data byte from ea
		md_ctrl    = fetch_first_md;
		addr_ctrl  = read_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// Here to read a 16 bit value into MD
		// pointed to by the EA register
		// The first byte is read
		// and the EA is incremented
		//
	dual_op_read16_state:
	begin
		// increment the effective address
		left_ctrl  = ea_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// read the high byte of the 16 bit data
		md_ctrl    = fetch_first_md;
		addr_ctrl  = read_ad;
		next_state = dual_op_read16_2_state;
	end

		//
		// here to read the second byte
		// pointed to by EA into MD
		//
	dual_op_read16_2_state:
	begin
		// read the low byte of the 16 bit data
		md_ctrl    = fetch_next_md;
		addr_ctrl  = read_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// 16 bit Write state
		// EA hold address of memory to write to
		// Advance the effective address in ALU
		// decode op_code to determine which
		// register to write
		//
	dual_op_write16_state:
	begin
		// increment the effective address
		left_ctrl  = ea_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// write the ALU hi byte at ea
		addr_ctrl  = write_ad;
		if (op_code[6] == 1'b0)
		begin
			case (op_code[3:0])
			4'b1111: // stx / sty
			begin
				case (pre_code)
				8'b00010000: // page 2 // sty
					dout_ctrl  = iy_hi_dout;
				default:     // page 1 // stx
					dout_ctrl  = ix_hi_dout;
				endcase
			end
			default:
				dout_ctrl  = md_hi_dout;
			endcase
		end
        else
        begin
			case (op_code[3:0])
			4'b1101: // std
				dout_ctrl  = acca_dout; // acca is high byte of ACCD
			4'b1111: // stu / sts
			begin
				case (pre_code)
				8'b00010000: // page 2 // sts
					dout_ctrl  = sp_hi_dout;
				default:     // page 1 // stu
					dout_ctrl  = up_hi_dout;
				endcase
			end
			default:
				dout_ctrl  = md_hi_dout;
			endcase
        end
		next_state   = dual_op_write8_state;
	end

		//
		// Dual operand 8 bit write
		// Write 8 bit accumulator
		// or low byte of 16 bit register
		// EA holds address
		// decode opcode to determine
		// which register to apply to the bus
		// Also set the condition codes here
		//
	dual_op_write8_state:
	begin
		if (op_code[6] == 1'b0)
		begin
			case (op_code[3:0])
			4'b0111: // sta
				dout_ctrl  = acca_dout;
			4'b1111: // stx / sty
			begin
				case (pre_code)
				8'b00010000: // page 2 // sty
					dout_ctrl  = iy_lo_dout;
				default:     // page 1 // stx
					dout_ctrl  = ix_lo_dout;
				endcase
			end
			default:
				dout_ctrl  = md_lo_dout;
			endcase
		end
        else
        begin        
			case (op_code[3:0])
			4'b0111: // stb
				dout_ctrl  = accb_dout;
			4'b1101: // std
				dout_ctrl  = accb_dout; // accb is low byte of accd
			4'b1111: // stu / sts
			begin
				case (pre_code)
				 8'b00010000: // page 2 // sts
					dout_ctrl  = sp_lo_dout;
				default:     // page 1 // stu
					dout_ctrl  = up_lo_dout;
				endcase
			end
			default:
				dout_ctrl  = md_lo_dout;
			endcase
        end
		// write ALU low byte output
		addr_ctrl    = write_ad;
		lic          = 1'b1;
		next_state   = fetch_state;
	end

		//
		// 16 bit immediate addressing mode
		//
	imm16_state:
	begin
		// increment pc
		pc_ctrl    = incr_pc;
		// fetch next immediate byte
		md_ctrl    = fetch_next_md;
		addr_ctrl  = fetch_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// md & ea holds 8 bit index offset
		// calculate the effective memory address
		// using the alu
		//
	indexed_state:
	begin
		//
		// decode indexing mode
		//
		if (md[7] == 1'b0)
		begin
			case (md[6:5])
			 2'b00:
				left_ctrl  = ix_left;
			 2'b01:
				left_ctrl  = iy_left;
			 2'b10:
				left_ctrl  = up_left;
			default:
			//  2'b11:
				left_ctrl  = sp_left;
			endcase
			right_ctrl   = md_sign5_right;
			alu_ctrl     = alu_add16;
			ea_ctrl      = load_ea;
			next_state   = saved_state;
		end
		else
		begin
			case (md[3:0])
			4'b0000:     // ,R+
			begin
				case (md[6:5])
				 2'b00:
					left_ctrl  = ix_left;
				 2'b01:
					left_ctrl  = iy_left;
				 2'b10:
					left_ctrl  = up_left;
				default:
					left_ctrl  = sp_left;
				endcase
				//
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				next_state = postincr1_state;
			end

			4'b0001:     // ,R++
			begin
				case (md[6:5])
				 2'b00:
					left_ctrl  = ix_left;
				 2'b01:
					left_ctrl  = iy_left;
				 2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				next_state = postincr2_state;
			end

			4'b0010:     // ,-R
			begin
				case (md[6:5])
				2'b00:
				begin
					left_ctrl  = ix_left;
					ix_ctrl    = load_ix;
				end
				2'b01:
				begin
					left_ctrl  = iy_left;
					iy_ctrl    = load_iy;
				end
				2'b10:
				begin
					left_ctrl  = up_left;
					up_ctrl    = load_up;
				end
				default:
				begin
				//  2'b11:
					left_ctrl  = sp_left;
					sp_ctrl    = load_sp;
				end
				endcase
				right_ctrl   = one_right;
				alu_ctrl     = alu_sub16;
				ea_ctrl      = load_ea;
				next_state   = saved_state;
			end

			4'b0011:     // ,//R
			begin
				case (md[6:5])
				2'b00:
				begin
					left_ctrl  = ix_left;
					ix_ctrl    = load_ix;
				end
				2'b01:
				begin
					left_ctrl  = iy_left;
					iy_ctrl    = load_iy;
				end
				2'b10:
				begin
					left_ctrl  = up_left;
					up_ctrl    = load_up;
				end
				default:
				begin
				//  2'b11:
					left_ctrl  = sp_left;
					sp_ctrl    = load_sp;
				end
				endcase
				right_ctrl = two_right;
				alu_ctrl   = alu_sub16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b0100:     // ,R (zero offset)
			begin
				case (md[6:5])
				 2'b00:
					left_ctrl  = ix_left;
				 2'b01:
					left_ctrl  = iy_left;
				 2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b0101:     // ACCB,R
			begin
				case (md[6:5])
				2'b00:
					left_ctrl  = ix_left;
				2'b01:
					left_ctrl  = iy_left;
				2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = accb_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b0110:     // ACCA,R
			begin
				case (md[6:5])
				2'b00:
					left_ctrl  = ix_left;
				2'b01:
					left_ctrl  = iy_left;
				2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = acca_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b0111:     // undefined
			begin
				case (md[6:5])
				 2'b00:
					left_ctrl  = ix_left;
				 2'b01:
					left_ctrl  = iy_left;
				 2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b1000:     // offset8,R
			begin
				md_ctrl    = fetch_first_md; // pick up 8 bit offset
				addr_ctrl  = fetch_ad;
				pc_ctrl    = incr_pc;
				next_state = index8_state;
			end

			4'b1001:     // offset16,R
			begin
				md_ctrl    = fetch_first_md; // pick up first byte of 16 bit offset
				addr_ctrl  = fetch_ad;
				pc_ctrl    = incr_pc;
				next_state = index16_state;
			end

			4'b1010:     // undefined
			begin
				case (md[6:5])
				 2'b00:
					left_ctrl  = ix_left;
				 2'b01:
					left_ctrl  = iy_left;
				 2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				//
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b1011:     // ACCD,R
			begin
				case (md[6:5])
				2'b00:
					left_ctrl  = ix_left;
				2'b01:
					left_ctrl  = iy_left;
				2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = accd_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			4'b1100:     // offset8,PC
			begin
				// fetch 8 bit offset
				md_ctrl    = fetch_first_md;
				addr_ctrl  = fetch_ad;
				pc_ctrl    = incr_pc;
				next_state = pcrel8_state;
			end

			4'b1101:     // offset16,PC
			begin
				// fetch offset
				md_ctrl    = fetch_first_md;
				addr_ctrl  = fetch_ad;
				pc_ctrl    = incr_pc;
				next_state = pcrel16_state;
			end

			4'b1110:     // undefined
			begin
				case (md[6:5])
				2'b00:
					left_ctrl  = ix_left;
				2'b01:
					left_ctrl  = iy_left;
				2'b10:
					left_ctrl  = up_left;
				default:
				//  2'b11:
					left_ctrl  = sp_left;
				endcase
				right_ctrl = zero_right;
				alu_ctrl   = alu_add16;
				ea_ctrl    = load_ea;
				if (md[4] == 1'b0)
					next_state   = saved_state;
				else
					next_state   = indirect_state;
			end

			default:
			begin
				// 4'b1111:     // [,address]
				// advance PC to pick up address
				md_ctrl    = fetch_first_md;
				addr_ctrl  = fetch_ad;
				pc_ctrl    = incr_pc;
				next_state = indexaddr_state;
			end
			endcase
		end
	end

		// load index register with ea plus one
	postincr1_state:
	begin
		left_ctrl  = ea_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		case (md[6:5])
		2'b00:
			ix_ctrl    = load_ix;
		2'b01:
			iy_ctrl    = load_iy;
		2'b10:
			up_ctrl    = load_up;
		default:
		//  2'b11:
			sp_ctrl    = load_sp;
		endcase
		// return to previous state
		if (md[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end

		// load index register with ea plus two
	postincr2_state:
	begin
		// increment register by two (address)
		left_ctrl  = ea_left;
		right_ctrl = two_right;
		alu_ctrl   = alu_add16;
		case (md[6:5])
		2'b00:
			ix_ctrl    = load_ix;
		2'b01:
			iy_ctrl    = load_iy;
		2'b10:
			up_ctrl    = load_up;
		default:
		//  2'b11:
			sp_ctrl    = load_sp;
		endcase
		// return to previous state
		if (md[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end
		//
		// ea == index register + md (8 bit signed offset)
		// ea holds post byte
		//
	index8_state:
	begin
		case (ea[6:5])
		2'b00:
			left_ctrl  = ix_left;
		2'b01:
			left_ctrl  = iy_left;
		2'b10:
			left_ctrl  = up_left;
		default:
		//  2'b11:
			left_ctrl  = sp_left;
		endcase
		// ea == index reg + md
		right_ctrl = md_sign8_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// return to previous state
		if (ea[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end

		// fetch low byte of 16 bit indexed offset
	index16_state:
	begin
		// advance pc
		pc_ctrl    = incr_pc;
		// fetch low byte
		md_ctrl    = fetch_next_md;
		addr_ctrl  = fetch_ad;
		next_state = index16_2_state;
	end

		// ea == index register + md (16 bit offset)
		// ea holds post byte
	index16_2_state:
	begin
		case (ea[6:5])
		2'b00:
			left_ctrl  = ix_left;
		2'b01:
			left_ctrl  = iy_left;
		2'b10:
			left_ctrl  = up_left;
		default:
		//  2'b11:
			left_ctrl  = sp_left;
		endcase
		// ea == index reg + md
		right_ctrl = md_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// return to previous state
		if (ea[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end
		//
		// pc relative with 8 bit signed offest
		// md holds signed offset
		//
	pcrel8_state:
	begin
		// ea == pc + signed md
		left_ctrl  = pc_left;
		right_ctrl = md_sign8_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// return to previous state
		if (ea[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end

		// pc relative addressing with 16 bit offset
		// pick up the low byte of the offset in md
		// advance the pc
	pcrel16_state:
	begin
		// advance pc
		pc_ctrl    = incr_pc;
		// fetch low byte
		md_ctrl    = fetch_next_md;
		addr_ctrl  = fetch_ad;
		next_state = pcrel16_2_state;
	end

		// pc relative with16 bit signed offest
		// md holds signed offset
	pcrel16_2_state:
	begin
		// ea == pc +  md
		left_ctrl  = pc_left;
		right_ctrl = md_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// return to previous state
		if (ea[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end

		// indexed to address
		// pick up the low byte of the address
		// advance the pc
	indexaddr_state:
	begin
		// advance pc
		pc_ctrl    = incr_pc;
		// fetch low byte
		md_ctrl    = fetch_next_md;
		addr_ctrl  = fetch_ad;
		next_state = indexaddr2_state;
	end

		// indexed to absolute address
		// md holds address
		// ea hold indexing mode byte
	indexaddr2_state:
	begin
		// ea == md
		left_ctrl  = pc_left;
		right_ctrl = md_right;
		alu_ctrl   = alu_ld16;
		ea_ctrl    = load_ea;
		// return to previous state
		if (ea[4] == 1'b0)
			next_state   = saved_state;
		else
			next_state   = indirect_state;
	end

		//
		// load md with high byte of indirect address
		// pointed to by ea
		// increment ea
		//
	indirect_state:
	begin
		// increment ea
		left_ctrl  = ea_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		ea_ctrl    = load_ea;
		// fetch high byte
		md_ctrl    = fetch_first_md;
		addr_ctrl  = read_ad;
		next_state = indirect2_state;
	end
		//
		// load md with low byte of indirect address
		// pointed to by ea
		// ea has previously been incremented
		//
	indirect2_state:
	begin
		// fetch high byte
		md_ctrl    = fetch_next_md;
		addr_ctrl  = read_ad;
		dout_ctrl  = md_lo_dout;
		next_state = indirect3_state;
	end
		//
		// complete idirect addressing
		// by loading ea with md
		//
	indirect3_state:
	begin
		// load ea with md
		left_ctrl  = ea_left;
		right_ctrl = md_right;
		alu_ctrl   = alu_ld16;
		ea_ctrl    = load_ea;
		// return to previous state
		next_state   = saved_state;
	end

		//
		// ea holds the low byte of the absolute address
		// Move ea low byte into ea high byte
		// load new ea low byte to for absolute 16 bit address
		// advance the program counter
		//
	extended_state: // fetch ea low byte
	begin
		// increment pc
		pc_ctrl      = incr_pc;
		// fetch next effective address bytes
		ea_ctrl      = fetch_next_ea;
		addr_ctrl    = fetch_ad;
		// return to previous state
		next_state   = saved_state;
	end

	lea_state: // here on load effective address
	begin
		// load index register with effective address
		left_ctrl  = pc_left;
		right_ctrl = ea_right;
		alu_ctrl   = alu_lea;
		case (op_code[3:0])
		4'b0000: // leax
		begin
			cc_ctrl    = load_cc;
			ix_ctrl    = load_ix;
		end
		4'b0001: // leay
		begin
			cc_ctrl    = load_cc;
			iy_ctrl    = load_iy;
		end
		4'b0010: // leas
			sp_ctrl    = load_sp;
		4'b0011: // leau
			up_ctrl    = load_up;
		default: ;

		endcase
		lic          = 1'b1;
		next_state   = fetch_state;
	end

		//
		// jump to subroutine
		// sp=sp-1
		// call push_return_lo_state to save pc
		// return to jmp_state
		//
	jsr_state:
	begin
		// decrement sp
		left_ctrl    = sp_left;
		right_ctrl   = one_right;
		alu_ctrl     = alu_sub16;
		sp_ctrl      = load_sp;
		// call push_return_state
		st_ctrl      = push_st;
		return_state = jmp_state;
		next_state   = push_return_lo_state;
	end

		//
		// Load pc with ea
		// (JMP)
		//
	jmp_state:
	begin
		// load PC with effective address
		left_ctrl  = pc_left;
		right_ctrl = ea_right;
		alu_ctrl   = alu_ld16;
		pc_ctrl    = load_pc;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// long branch or branch to subroutine
		// pick up next md byte
		// md_hi == md_lo
		// md_lo == (pc)
		// pc=pc+1
		// if a lbsr push return address
		// continue to sbranch_state
		// to evaluate conditional branches
		//
	lbranch_state:
	begin
		pc_ctrl    = incr_pc;
		// fetch the next byte into md_lo
		md_ctrl    = fetch_next_md;
		addr_ctrl  = fetch_ad;
		// if lbsr - push return address
		// then continue on to short branch
		if (op_code == 8'b00010111)
		begin
			st_ctrl      = push_st;
			return_state = sbranch_state;
			next_state   = push_return_lo_state;
		end
		else
			next_state   = sbranch_state;
	end

		//
		// here to execute conditional branch
		// short conditional branch md == signed 8 bit offset
		// long branch md == 16 bit offset
		// 
	sbranch_state:
	begin
		left_ctrl  = pc_left;
		right_ctrl = md_right;
		alu_ctrl   = alu_add16;
		// Test condition for branch
		if (op_code[7:4] == 4'b0010) // conditional branch
		begin
			case (op_code[3:0])
			4'b0000: // bra
				cond_true = 1'b1;
			4'b0001: // brn
				cond_true = 1'b0;
			4'b0010: // bhi
				cond_true = ((cc[CBIT] | cc[ZBIT]) == 1'b0);
			4'b0011: // bls
				cond_true = ((cc[CBIT] | cc[ZBIT]) == 1'b1);
			4'b0100: // bcc/bhs
				cond_true = (cc[CBIT] == 1'b0);
			4'b0101: // bcs/blo
				cond_true = (cc[CBIT] == 1'b1);
			4'b0110: // bne
				cond_true = (cc[ZBIT] == 1'b0);
			4'b0111: // beq
				cond_true = (cc[ZBIT] == 1'b1);
			4'b1000: // bvc
				cond_true = (cc[VBIT] == 1'b0);
			4'b1001: // bvs
				cond_true = (cc[VBIT] == 1'b1);
			4'b1010: // bpl
				cond_true = (cc[NBIT] == 1'b0);
			4'b1011: // bmi
				cond_true = (cc[NBIT] == 1'b1);
			4'b1100: // bge
				cond_true = ((cc[NBIT] ^ cc[VBIT]) == 1'b0);
			4'b1101: // blt
				cond_true = ((cc[NBIT] ^ cc[VBIT]) == 1'b1);
			4'b1110: // bgt
				cond_true = ((cc[ZBIT] | (cc[NBIT] ^ cc[VBIT])) == 1'b0);
			4'b1111: // ble
				cond_true = ((cc[ZBIT] | (cc[NBIT] ^ cc[VBIT])) == 1'b1);
			default: ;

			endcase
		end
		if (cond_true)
			pc_ctrl    = load_pc;
		lic          = 1'b1;
		next_state   = fetch_state;
	end

		//
		// push return address onto the S stack
		//
		// (sp) == pc_lo
		// sp == sp - 1
		//
	push_return_lo_state:
	begin
		// decrement the sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write PC low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = pc_lo_dout;
		next_state = push_return_hi_state;
	end

		//
		// push program counter hi byte onto the stack
		// (sp) == pc_hi
		// sp == sp
		// return to originating state
		//
	push_return_hi_state:
	begin
		// write pc hi bytes
		addr_ctrl    = pushs_ad;
		dout_ctrl    = pc_hi_dout;
		next_state   = saved_state;
	end

		//
		// RTS pull return address from stack
		//
	pull_return_hi_state:
	begin
		// increment the sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read pc hi
		pc_ctrl    = pull_hi_pc;
		addr_ctrl  = pulls_ad;
		next_state = pull_return_lo_state;
	end

	pull_return_lo_state:
	begin
		// increment the SP
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read pc low
		pc_ctrl    = pull_lo_pc;
		addr_ctrl  = pulls_ad;
		dout_ctrl  = pc_lo_dout;
		//
		lic          = 1'b1;
		next_state   = fetch_state;
	end

	andcc_state:
	begin
		// AND CC with md
		left_ctrl  = md_left;
		right_ctrl = zero_right;
		alu_ctrl   = alu_andcc;
		cc_ctrl    = load_cc;
		//
		lic        = 1'b1;
		next_state = fetch_state;
	end

	orcc_state:
	begin
		// OR CC with md
		left_ctrl  = md_left;
		right_ctrl = zero_right;
		alu_ctrl   = alu_orcc;
		cc_ctrl    = load_cc;
		//
		lic        = 1'b1;
		next_state = fetch_state;
	end

	tfr_state:
	begin
		// select source register
		case (md[7:4])
		4'b0000:
			left_ctrl = accd_left;
		4'b0001:
			left_ctrl = ix_left;
		4'b0010:
			left_ctrl = iy_left;
		4'b0011:
			left_ctrl = up_left;
		4'b0100:
			left_ctrl = sp_left;
		4'b0101:
			left_ctrl = pc_left;
		4'b1000:
			left_ctrl = acca_left;
		4'b1001:
			left_ctrl = accb_left;
		4'b1010:
			left_ctrl = cc_left;
		4'b1011:
			left_ctrl = dp_left;
		default:
			left_ctrl  = md_left;
		endcase
		right_ctrl = zero_right;
		alu_ctrl   = alu_tfr;
		// select destination register
		case (md[3:0])
		4'b0000: // accd
		begin
			acca_ctrl  = load_hi_acca;
			accb_ctrl  = load_accb;
		end
		4'b0001: // ix
			ix_ctrl    = load_ix;
		4'b0010: // iy
			iy_ctrl    = load_iy;
		4'b0011: // up
			up_ctrl    = load_up;
		4'b0100: // sp
			sp_ctrl    = load_sp;
		4'b0101: // pc
			pc_ctrl    = load_pc;
		4'b1000: // acca
			acca_ctrl  = load_acca;
		4'b1001: // accb
			accb_ctrl  = load_accb;
		4'b1010: // cc
			cc_ctrl    = load_cc;
		4'b1011: //dp
			dp_ctrl    = load_dp;
		default: ;

		endcase
		//
		lic          = 1'b1;
		next_state   = fetch_state;
	end

	exg_state:
	begin
		// save destination register
		case (md[3:0])
		4'b0000:
			left_ctrl = accd_left;
		4'b0001:
			left_ctrl = ix_left;
		4'b0010:
			left_ctrl = iy_left;
		4'b0011:
			left_ctrl = up_left;
		4'b0100:
			left_ctrl = sp_left;
		4'b0101:
			left_ctrl = pc_left;
		4'b1000:
			left_ctrl = acca_left;
		4'b1001:
			left_ctrl = accb_left;
		4'b1010:
			left_ctrl = cc_left;
		4'b1011:
			left_ctrl = dp_left;
		default:
			left_ctrl  = md_left;
		endcase
		right_ctrl = zero_right;
		alu_ctrl   = alu_tfr;
		ea_ctrl    = load_ea;
		// call tranfer microcode
		next_state   = exg1_state;
	end

	exg1_state:
	begin
		// select source register
		case (md[7:4])
		4'b0000:
			left_ctrl = accd_left;
		4'b0001:
			left_ctrl = ix_left;
		4'b0010:
			left_ctrl = iy_left;
		4'b0011:
			left_ctrl = up_left;
		4'b0100:
			left_ctrl = sp_left;
		4'b0101:
			left_ctrl = pc_left;
		4'b1000:
			left_ctrl = acca_left;
		4'b1001:
			left_ctrl = accb_left;
		4'b1010:
			left_ctrl = cc_left;
		4'b1011:
			left_ctrl = dp_left;
		default:
			left_ctrl  = md_left;
		endcase
		right_ctrl = zero_right;
		alu_ctrl   = alu_tfr;
		// select destination register
		case (md[3:0])
		4'b0000: // accd
		begin
			acca_ctrl  = load_hi_acca;
			accb_ctrl  = load_accb;
		end
		4'b0001: // ix
			ix_ctrl    = load_ix;
		4'b0010: // iy
			iy_ctrl    = load_iy;
		4'b0011: // up
			up_ctrl    = load_up;
		4'b0100: // sp
			sp_ctrl    = load_sp;
		4'b0101: // pc
			pc_ctrl    = load_pc;
		4'b1000: // acca
			acca_ctrl  = load_acca;
		4'b1001: // accb
			accb_ctrl  = load_accb;
		4'b1010: // cc
			cc_ctrl    = load_cc;
		4'b1011: //dp
			dp_ctrl    = load_dp;
		default: ;

		endcase
		next_state   = exg2_state;
	end

	exg2_state:
	begin
		// restore destination
		left_ctrl  = ea_left;
		right_ctrl = zero_right;
		alu_ctrl   = alu_tfr;
		// save as source register
		case (md[7:4])
		4'b0000: // accd
		begin
			acca_ctrl  = load_hi_acca;
			accb_ctrl  = load_accb;
		end
		4'b0001: // ix
			ix_ctrl    = load_ix;
		4'b0010: // iy
			iy_ctrl    = load_iy;
		4'b0011: // up
			up_ctrl    = load_up;
		4'b0100: // sp
			sp_ctrl    = load_sp;
		4'b0101: // pc
			pc_ctrl    = load_pc;
		4'b1000: // acca
			acca_ctrl  = load_acca;
		4'b1001: // accb
			accb_ctrl  = load_accb;
		4'b1010: // cc
			cc_ctrl    = load_cc;
		4'b1011: //dp
			dp_ctrl    = load_dp;
		default: ;

		endcase
		lic          = 1'b1;
		next_state   = fetch_state;
	end

	mulea_state:
	begin
		// move accb to ea
		left_ctrl  = accb_left;
		right_ctrl = zero_right;
		alu_ctrl   = alu_st16;
		ea_ctrl    = load_ea;
		next_state = muld_state;
	end

	muld_state:
	begin
		// clear accd
		left_ctrl  = acca_left;
		right_ctrl = zero_right;
		alu_ctrl   = alu_ld8;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		next_state = mul0_state;
	end

	mul0_state:
	begin
		// if bit 0 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[0] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul1_state;
	end

	mul1_state:
	begin
		// if bit 1 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[1] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul2_state;
	end

	mul2_state:
	begin
		// if bit 2 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[2] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul3_state;
	end

	mul3_state:
	begin
		// if bit 3 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[3] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul4_state;
	end

	mul4_state:
	begin
		// if bit 4 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[4] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul5_state;
	end

	mul5_state:
	begin
		// if bit 5 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[5] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		next_state = mul6_state;
	end

	mul6_state:
	begin
		// if bit 6 of ea set, add accd to md
		left_ctrl  = accd_left;
		if (ea[6] == 1'b1)
			right_ctrl = md_right;
		else
			right_ctrl = zero_right;
		alu_ctrl   = alu_mul;
		cc_ctrl    = load_cc;
		acca_ctrl  = load_hi_acca;
		accb_ctrl  = load_accb;
		md_ctrl    = shiftl_md;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// Enter here on pushs
		// ea holds post byte
		//
	pshs_state:
	begin
		// decrement sp if any registers to be pushed
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		// idle	address
		addr_ctrl  = idle_ad;
		dout_ctrl  = cc_dout; 
		if (ea[7:0] == 8'b00000000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		if (ea[7] == 1'b1)
			next_state = pshs_pcl_state;
		else if (ea[6] == 1'b1)
			next_state = pshs_upl_state;
		else if (ea[5] == 1'b1)
			next_state = pshs_iyl_state;
		else if (ea[4] == 1'b1)
			next_state = pshs_ixl_state;
		else if (ea[3] == 1'b1)
			next_state = pshs_dp_state;
		else if (ea[2] == 1'b1)
			next_state = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state = pshs_cc_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pshs_pcl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write pc low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = pc_lo_dout; 
		next_state = pshs_pch_state;
	end

	pshs_pch_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[6:0] == 7'b0000000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write pc hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = pc_hi_dout; 
		if (ea[6] == 1'b1)
			next_state = pshs_upl_state;
		else if (ea[5] == 1'b1)
			next_state = pshs_iyl_state;
		else if (ea[4] == 1'b1)
			next_state = pshs_ixl_state;
		else if (ea[3] == 1'b1)
			next_state = pshs_dp_state;
		else if (ea[2] == 1'b1)
			next_state = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state = pshs_cc_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end


	pshs_upl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write pc low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = up_lo_dout; 
		next_state = pshs_uph_state;
	end

	pshs_uph_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[5:0] == 6'b000000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write pc hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = up_hi_dout; 
		if (ea[5] == 1'b1)
			next_state   = pshs_iyl_state;
		else if (ea[4] == 1'b1)
			next_state   = pshs_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshs_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_iyl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write iy low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = iy_lo_dout; 
		next_state = pshs_iyh_state;
	end

	pshs_iyh_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[4:0] == 5'b00000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write iy hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = iy_hi_dout; 
		if (ea[4] == 1'b1)
			next_state   = pshs_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshs_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_ixl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = ix_lo_dout; 
		next_state = pshs_ixh_state;
	end

	pshs_ixh_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[3:0] == 4'b0000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write ix hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = ix_hi_dout; 
		if (ea[3] == 1'b1)
			next_state   = pshs_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_dp_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[2:0] == 3'b000)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write dp
		addr_ctrl  = pushs_ad;
		dout_ctrl  = dp_dout; 
		if (ea[2] == 1'b1)
			next_state   = pshs_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_accb_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[1:0] == 2'b00)
			sp_ctrl    = latch_sp;
		else
			sp_ctrl    = load_sp;
		// write accb
		addr_ctrl  = pushs_ad;
		dout_ctrl  = accb_dout; 
		if (ea[1] == 1'b1)
			next_state   = pshs_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_acca_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[0] == 1'b1)
			sp_ctrl    = load_sp;
		else
			sp_ctrl    = latch_sp;
		// write acca
		addr_ctrl  = pushs_ad;
		dout_ctrl  = acca_dout; 
		if (ea[0] == 1'b1)
			next_state   = pshs_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshs_cc_state:
	begin
		// idle sp
		// write cc
		addr_ctrl  = pushs_ad;
		dout_ctrl  = cc_dout; 
		lic          = 1'b1;
		next_state = fetch_state;
	end

		//
		// enter here on PULS
		// ea hold register mask
		//
	puls_state:
	begin
		if (ea[0] == 1'b1)
			next_state = puls_cc_state;
		else if (ea[1] == 1'b1)
			next_state = puls_acca_state;
		else if (ea[2] == 1'b1)
			next_state = puls_accb_state;
		else if (ea[3] == 1'b1)
			next_state = puls_dp_state;
		else if (ea[4] == 1'b1)
			next_state = puls_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_cc_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read cc
		cc_ctrl    = pull_cc;
		addr_ctrl  = pulls_ad;
		if (ea[1] == 1'b1)
			next_state = puls_acca_state;
		else if (ea[2] == 1'b1)
			next_state = puls_accb_state;
		else if (ea[3] == 1'b1)
			next_state = puls_dp_state;
		else if (ea[4] == 1'b1)
			next_state = puls_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_acca_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read acca
		acca_ctrl  = pull_acca;
		addr_ctrl  = pulls_ad;
		if (ea[2] == 1'b1)
			next_state = puls_accb_state;
		else if (ea[3] == 1'b1)
			next_state = puls_dp_state;
		else if (ea[4] == 1'b1)
			next_state = puls_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic          = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_accb_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read accb
		accb_ctrl  = pull_accb;
		addr_ctrl  = pulls_ad;
		if (ea[3] == 1'b1)
			next_state = puls_dp_state;
		else if (ea[4] == 1'b1)
			next_state = puls_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_dp_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read dp
		dp_ctrl    = pull_dp;
		addr_ctrl  = pulls_ad;
		if (ea[4] == 1'b1)
			next_state = puls_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_ixh_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull ix hi
		ix_ctrl    = pull_hi_ix;
		addr_ctrl  = pulls_ad;
		next_state = puls_ixl_state;
	end

	puls_ixl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read ix low
		ix_ctrl    = pull_lo_ix;
		addr_ctrl  = pulls_ad;
		if (ea[5] == 1'b1)
			next_state = puls_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_iyh_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull iy hi
		iy_ctrl    = pull_hi_iy;
		addr_ctrl  = pulls_ad;
		next_state   = puls_iyl_state;
	end

	puls_iyl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read iy low
		iy_ctrl    = pull_lo_iy;
		addr_ctrl  = pulls_ad;
		if (ea[6] == 1'b1)
			next_state = puls_uph_state;
		else if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_uph_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull up hi
		up_ctrl    = pull_hi_up;
		addr_ctrl  = pulls_ad;
		next_state = puls_upl_state;
	end

	puls_upl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read up low
		up_ctrl    = pull_lo_up;
		addr_ctrl  = pulls_ad;
		if (ea[7] == 1'b1)
			next_state = puls_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	puls_pch_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull pc hi
		pc_ctrl    = pull_hi_pc;
		addr_ctrl  = pulls_ad;
		next_state = puls_pcl_state;
	end

	puls_pcl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read pc low
		pc_ctrl    = pull_lo_pc;
		addr_ctrl  = pulls_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// Enter here on pshu
		// ea holds post byte
		//
	pshu_state:
	begin
		// decrement up if any registers to be pushed
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[7:0] == 8'b00000000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write idle bus
		if (ea[7] == 1'b1)
			next_state   = pshu_pcl_state;
		else if (ea[6] == 1'b1)
			next_state   = pshu_spl_state;
		else if (ea[5] == 1'b1)
			next_state   = pshu_iyl_state;
		else if (ea[4] == 1'b1)
			next_state   = pshu_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshu_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end
		//
		// push PC onto U stack
		//
	pshu_pcl_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		up_ctrl    = load_up;
		// write pc low
		addr_ctrl  = pushu_ad;
		dout_ctrl  = pc_lo_dout; 
		next_state = pshu_pch_state;
	end

	pshu_pch_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[6:0] == 7'b0000000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write pc hi
		addr_ctrl  = pushu_ad;
		dout_ctrl  = pc_hi_dout; 
		if (ea[6] == 1'b1)
			next_state   = pshu_spl_state;
		else if (ea[5] == 1'b1)
			next_state   = pshu_iyl_state;
		else if (ea[4] == 1'b1)
			next_state   = pshu_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshu_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_spl_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		up_ctrl    = load_up;
		// write sp low
		addr_ctrl  = pushu_ad;
		dout_ctrl  = sp_lo_dout; 
		next_state = pshu_sph_state;
	end

	pshu_sph_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[5:0] == 6'b000000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write sp hi
		addr_ctrl  = pushu_ad;
		dout_ctrl  = sp_hi_dout; 
		if (ea[5] == 1'b1)
			next_state   = pshu_iyl_state;
		else if (ea[4] == 1'b1)
			next_state   = pshu_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshu_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_iyl_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		up_ctrl    = load_up;
		// write iy low
		addr_ctrl  = pushu_ad;
		dout_ctrl  = iy_lo_dout; 
		next_state = pshu_iyh_state;
	end

	pshu_iyh_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[4:0] == 5'b00000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write iy hi
		addr_ctrl  = pushu_ad;
		dout_ctrl  = iy_hi_dout; 
		if (ea[4] == 1'b1)
			next_state   = pshu_ixl_state;
		else if (ea[3] == 1'b1)
			next_state   = pshu_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_ixl_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		up_ctrl    = load_up;
		// write ix low
		addr_ctrl  = pushu_ad;
		dout_ctrl  = ix_lo_dout; 
		next_state = pshu_ixh_state;
	end

	pshu_ixh_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[3:0] == 4'b0000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write ix hi
		addr_ctrl  = pushu_ad;
		dout_ctrl  = ix_hi_dout; 
		if (ea[3] == 1'b1)
			next_state   = pshu_dp_state;
		else if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_dp_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[2:0] == 3'b000)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write dp
		addr_ctrl  = pushu_ad;
		dout_ctrl  = dp_dout; 
		if (ea[2] == 1'b1)
			next_state   = pshu_accb_state;
		else if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_accb_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[1:0] == 2'b00)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write accb
		addr_ctrl  = pushu_ad;
		dout_ctrl  = accb_dout; 
		if (ea[1] == 1'b1)
			next_state   = pshu_acca_state;
		else if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_acca_state:
	begin
		// decrement up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		if (ea[0] == 1'b0)
			up_ctrl    = latch_up;
		else
			up_ctrl    = load_up;
		// write acca
		addr_ctrl  = pushu_ad;
		dout_ctrl  = acca_dout; 
		if (ea[0] == 1'b1)
			next_state   = pshu_cc_state;
		else
		begin
			lic          = 1'b1;
			next_state   = fetch_state;
		end
	end

	pshu_cc_state:
	begin
		// idle up
		// write cc
		addr_ctrl  = pushu_ad;
		dout_ctrl  = cc_dout; 
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// enter here on PULU
		// ea hold register mask
		//
	pulu_state:
	begin
		// idle UP
		// idle bus
		if (ea[0] == 1'b1)
			next_state = pulu_cc_state;
		else if (ea[1] == 1'b1)
			next_state = pulu_acca_state;
		else if (ea[2] == 1'b1)
			next_state = pulu_accb_state;
		else if (ea[3] == 1'b1)
			next_state = pulu_dp_state;
		else if (ea[4] == 1'b1)
			next_state = pulu_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_cc_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read cc
		cc_ctrl    = pull_cc;
		addr_ctrl  = pullu_ad;
		if (ea[1] == 1'b1)
			next_state = pulu_acca_state;
		else if (ea[2] == 1'b1)
			next_state = pulu_accb_state;
		else if (ea[3] == 1'b1)
			next_state = pulu_dp_state;
		else if (ea[4] == 1'b1)
			next_state = pulu_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_acca_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read acca
		acca_ctrl  = pull_acca;
		addr_ctrl  = pullu_ad;
		if (ea[2] == 1'b1)
			next_state = pulu_accb_state;
		else if (ea[3] == 1'b1)
			next_state = pulu_dp_state;
		else if (ea[4] == 1'b1)
			next_state = pulu_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_accb_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read accb
		accb_ctrl  = pull_accb;
		addr_ctrl  = pullu_ad;
		if (ea[3] == 1'b1)
			next_state = pulu_dp_state;
		else if (ea[4] == 1'b1)
			next_state = pulu_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_dp_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read dp
		dp_ctrl    = pull_dp;
		addr_ctrl  = pullu_ad;
		if (ea[4] == 1'b1)
			next_state = pulu_ixh_state;
		else if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_ixh_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read ix hi
		ix_ctrl    = pull_hi_ix;
		addr_ctrl  = pullu_ad;
		next_state = pulu_ixl_state;
	end

	pulu_ixl_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read ix low
		ix_ctrl    = pull_lo_ix;
		addr_ctrl  = pullu_ad;
		if (ea[5] == 1'b1)
			next_state = pulu_iyh_state;
		else if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_iyh_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read iy hi
		iy_ctrl    = pull_hi_iy;
		addr_ctrl  = pullu_ad;
		next_state = pulu_iyl_state;
	end

	pulu_iyl_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read iy low
		iy_ctrl    = pull_lo_iy;
		addr_ctrl  = pullu_ad;
		if (ea[6] == 1'b1)
			next_state = pulu_sph_state;
		else if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_sph_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read sp hi
		sp_ctrl    = pull_hi_sp;
		addr_ctrl  = pullu_ad;
		next_state = pulu_spl_state;
	end

	pulu_spl_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read sp low
		sp_ctrl    = pull_lo_sp;
		addr_ctrl  = pullu_ad;
		if (ea[7] == 1'b1)
			next_state = pulu_pch_state;
		else
		begin
			lic        = 1'b1;
			next_state = fetch_state;
		end
	end

	pulu_pch_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// pull pc hi
		pc_ctrl    = pull_hi_pc;
		addr_ctrl  = pullu_ad;
		next_state = pulu_pcl_state;
	end

	pulu_pcl_state:
	begin
		// increment up
		left_ctrl  = up_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		up_ctrl    = load_up;
		// read pc low
		pc_ctrl    = pull_lo_pc;
		addr_ctrl  = pullu_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// pop the Condition codes
		//
	rti_cc_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read cc
		cc_ctrl    = pull_cc;
		addr_ctrl  = pulls_ad;
		next_state = rti_entire_state;
	end

		//
		// Added RTI cycle 11th July 2006 John Kent.
		// test the "Entire" Flag
		// that has just been popped off the stack
		//
	rti_entire_state:
	begin
		//
		// The Entire flag must be recovered from the stack
		// before testing.
		//
		if (cc[EBIT] == 1'b1)
			next_state   = rti_acca_state;
		else
			next_state   = rti_pch_state;
	end

	rti_acca_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read acca
		acca_ctrl  = pull_acca;
		addr_ctrl  = pulls_ad;
		next_state = rti_accb_state;
	end

	rti_accb_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read accb
		accb_ctrl  = pull_accb;
		addr_ctrl  = pulls_ad;
		next_state = rti_dp_state;
	end

	rti_dp_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read dp
		dp_ctrl    = pull_dp;
		addr_ctrl  = pulls_ad;
		next_state = rti_ixh_state;
	end

	rti_ixh_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read ix hi
		ix_ctrl    = pull_hi_ix;
		addr_ctrl  = pulls_ad;
		next_state = rti_ixl_state;
	end

	rti_ixl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read ix low
		ix_ctrl    = pull_lo_ix;
		addr_ctrl  = pulls_ad;
		next_state = rti_iyh_state;
	end

	rti_iyh_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read iy hi
		iy_ctrl    = pull_hi_iy;
		addr_ctrl  = pulls_ad;
		next_state = rti_iyl_state;
	end

	rti_iyl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read iy low
		iy_ctrl    = pull_lo_iy;
		addr_ctrl  = pulls_ad;
		next_state = rti_uph_state;
	end


	rti_uph_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read up hi
		up_ctrl    = pull_hi_up;
		addr_ctrl  = pulls_ad;
		next_state = rti_upl_state;
	end

	rti_upl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// read up low
		up_ctrl    = pull_lo_up;
		addr_ctrl  = pulls_ad;
		next_state = rti_pch_state;
	end

	rti_pch_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull pc hi
		pc_ctrl    = pull_hi_pc;
		addr_ctrl  = pulls_ad;
		next_state = rti_pcl_state;
	end

	rti_pcl_state:
	begin
		// increment sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_add16;
		sp_ctrl    = load_sp;
		// pull pc low
		pc_ctrl    = pull_lo_pc;
		addr_ctrl  = pulls_ad;
		lic        = 1'b1;
		next_state = fetch_state;
	end

		//
		// here on NMI interrupt
		// Complete execute cycle of the last instruction.
		// If it was a dual operand instruction
		//
	int_nmi_state:
		next_state   = int_nmi1_state;

		// Idle bus cycle
	int_nmi1_state:
	begin
		// pre decrement sp
		left_ctrl    = sp_left;
		right_ctrl   = one_right;
		alu_ctrl     = alu_sub16;
		sp_ctrl      = load_sp;
		iv_ctrl      = nmi_iv;
		st_ctrl      = push_st;
		return_state = int_nmimask_state;
		next_state   = int_entire_state;
	end

		//
		// here on IRQ interrupt
		// Complete execute cycle of the last instruction.
		// If it was a dual operand instruction
		//
	int_irq_state:
		next_state   = int_irq1_state;

		// pre decrement the sp
		// Idle bus cycle
	int_irq1_state:
	begin
		// pre decrement sp
		left_ctrl    = sp_left;
		right_ctrl   = one_right;
		alu_ctrl     = alu_sub16;
		sp_ctrl      = load_sp;
		iv_ctrl      = irq_iv;
		st_ctrl      = push_st;
		return_state = int_irqmask_state;
		next_state   = int_entire_state;
	end

		//
		// here on FIRQ interrupt
		// Complete execution cycle of the last instruction
		// if it was a dual operand instruction
		//
	int_firq_state:
		next_state   = int_firq1_state;

		// Idle bus cycle
	int_firq1_state:
	begin
		// pre decrement sp
		left_ctrl    = sp_left;
		right_ctrl   = one_right;
		alu_ctrl     = alu_sub16;
		sp_ctrl      = load_sp;
		iv_ctrl      = firq_iv;
		st_ctrl      = push_st;
		return_state = int_firqmask_state;
		next_state   = int_fast_state;
	end

		//
		// CWAI entry point
		// stack pointer already pre-decremented
		// mask condition codes
		//
	cwai_state:
	begin
		// AND CC with md
		left_ctrl    = md_left;
		right_ctrl   = zero_right;
		alu_ctrl     = alu_andcc;
		cc_ctrl      = load_cc;
		st_ctrl      = push_st;
		return_state = int_cwai_state;
		next_state   = int_entire_state;
	end

		//
		// wait here for an interrupt
		//
	int_cwai_state:
	begin
		if (nmi_req == 1'b1)
		begin
			iv_ctrl    = nmi_iv;
			next_state = int_nmimask_state;
			//
			// FIRQ & IRQ are level sensitive
			//
		end
		else if ((firq == 1'b1) && (cc[FBIT] == 1'b0))
		begin
			iv_ctrl     = firq_iv;
			next_state  = int_firqmask_state;
		end
		else if ((irq == 1'b1) && (cc[IBIT] == 1'b0))
		begin
			iv_ctrl     = irq_iv;
			next_state  = int_irqmask_state;
		end
		else
		begin
			next_state = int_cwai_state;
		end
	end

		//
		// State to mask I Flag and F Flag (NMI)
		//
	int_nmimask_state:
	begin
		alu_ctrl   = alu_seif;
		cc_ctrl    = load_cc;
		next_state = vect_hi_state;
	end

		//
		// State to mask I Flag and F Flag (FIRQ)
		//
	int_firqmask_state:
	begin
		alu_ctrl   = alu_seif;
		cc_ctrl    = load_cc;
		next_state = vect_hi_state;
	end


		//
		// State to mask I Flag and F Flag (SWI)
		//
	int_swimask_state:
	begin
		alu_ctrl   = alu_seif;
		cc_ctrl    = load_cc;
		next_state = vect_hi_state;
	end

		//
		// State to mask I Flag only (IRQ)
		//
	int_irqmask_state:
	begin
		alu_ctrl   = alu_sei;
		cc_ctrl    = load_cc;
		next_state = vect_hi_state;
	end

		//
		// set Entire Flag on SWI, SWI2, SWI3 and CWAI, IRQ and NMI
		// before stacking all registers
		//
	int_entire_state:
	begin
		// set entire flag
		alu_ctrl   = alu_see;
		cc_ctrl    = load_cc;
		next_state = int_pcl_state;
	end

		//
		// clear Entire Flag on FIRQ
		// before stacking all registers
		//
	int_fast_state:
	begin
		// clear entire flag
		alu_ctrl   = alu_cle;
		cc_ctrl    = load_cc;
		next_state = int_pcl_state;
	end

	int_pcl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write pc low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = pc_lo_dout; 
		next_state = int_pch_state;
	end

	int_pch_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write pc hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = pc_hi_dout; 
		if (cc[EBIT] == 1'b1)
			next_state   = int_upl_state;
		else
			next_state   = int_cc_state;
	end

	int_upl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write up low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = up_lo_dout; 
		next_state = int_uph_state;
	end

	int_uph_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = up_hi_dout; 
		next_state = int_iyl_state;
	end

	int_iyl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = iy_lo_dout; 
		next_state = int_iyh_state;
	end

	int_iyh_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = iy_hi_dout; 
		next_state = int_ixl_state;
	end

	int_ixl_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix low
		addr_ctrl  = pushs_ad;
		dout_ctrl  = ix_lo_dout; 
		next_state = int_ixh_state;
	end

	int_ixh_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write ix hi
		addr_ctrl  = pushs_ad;
		dout_ctrl  = ix_hi_dout; 
		next_state = int_dp_state;
	end

	int_dp_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write accb
		addr_ctrl  = pushs_ad;
		dout_ctrl  = dp_dout; 
		next_state = int_accb_state;
	end

	int_accb_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write accb
		addr_ctrl  = pushs_ad;
		dout_ctrl  = accb_dout; 
		next_state = int_acca_state;
	end

	int_acca_state:
	begin
		// decrement sp
		left_ctrl  = sp_left;
		right_ctrl = one_right;
		alu_ctrl   = alu_sub16;
		sp_ctrl    = load_sp;
		// write acca
		addr_ctrl  = pushs_ad;
		dout_ctrl  = acca_dout; 
		next_state = int_cc_state;
	end

	int_cc_state:
	begin
		// write cc
		addr_ctrl  = pushs_ad;
		dout_ctrl  = cc_dout;
		next_state = saved_state;
	end

		//
		// According to the 6809 programming manual:
		// If an interrupt is received and is masked 
		// or lasts for less than three cycles, the PC 
		// will advance to the next instruction.
		// If an interrupt is unmasked and lasts
		// for more than three cycles, an interrupt
		// will be generated.
		// Note that I don't wait 3 clock cycles.
		// John Kent 11th July 2006
		//
	sync_state:
	begin
		lic        = 1'b1;
		ba         = 1'b1;
		//
		// Version 1.28 2015-05-30
		// Exit sync_state on interrupt.
		// If the interrupts are active
		// they will be caught in the state_machine process
		// and the interrupt service routine microcode will be executed.
		// Masked interrupts will exit the sync_state.
		// Moved from the state_machine process to the state_sequencer process
		//
		if ((firq == 1'b1) || (irq == 1'b1))
			next_state = fetch_state;
		else
			next_state = sync_state;
	end
				
	halt_state:
	begin
		//
		// 2011-10-30 John Kent
		// ba & bs should be high
		ba           = 1'b1;
		bs           = 1'b1;
		if (halt == 1'b1)
			next_state   = halt_state;
		else
			next_state   = fetch_state;
	end

	endcase

//
// Ver 1.23 2011-10-30 John Kent
// First instruction cycle might be
// fetch_state
// halt_state
// int_nmirq_state
// int_firq_state
//
	if (fic == 1'b1)
	begin
		//
		case (op_code[7:6])
		//
		// ver 1.29 
		// hide last cycle of multiply in fetch
		//
		2'b00:
		begin
			case (op_code[5:0])
			6'b111101: // mul
			begin
				// if bit 7 of ea set, add accd to md
				left_ctrl  = accd_left;
				if (ea[7] == 1'b1)
					right_ctrl = md_right;
				else
					right_ctrl = zero_right;
				alu_ctrl   = alu_mul;
				cc_ctrl    = load_cc;
				acca_ctrl  = load_hi_acca;
				accb_ctrl  = load_accb;
			end
			default: ;

			endcase
		end
		2'b10: // acca
		begin
			case (op_code[3:0])
			4'b0000: // suba
			begin
				left_ctrl  = acca_left;
				right_ctrl = md_right;
				alu_ctrl   = alu_sub8;
				cc_ctrl    = load_cc;
				acca_ctrl  = load_acca;
			end
			4'b0001: // cmpa
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_sub8;
				cc_ctrl     = load_cc;
			end
			4'b0010: // sbca
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_sbc;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b0011:
			begin
				case (pre_code)
				8'b00010000: // page 2 // cmpd
				begin
					left_ctrl   = accd_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
				end
				8'b00010001: // page 3 // cmpu
				begin
					left_ctrl   = up_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
				end
				default: // page 1 // subd
				begin
					left_ctrl   = accd_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
					acca_ctrl   = load_hi_acca;
					accb_ctrl   = load_accb;
				end
				endcase
			end
			4'b0100: // anda
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_and;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b0101: // bita
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_and;
				cc_ctrl     = load_cc;
			end
			4'b0110: // ldaa
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_ld8;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b0111: // staa
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_st8;
				cc_ctrl     = load_cc;
			end
			4'b1000: // eora
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_eor;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b1001: // adca
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_adc;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b1010: // oraa
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_ora;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b1011: // adda
			begin
				left_ctrl   = acca_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_add8;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_acca;
			end
			4'b1100:
			begin
				case (pre_code)
				8'b00010000: // page 2 // cmpy
				begin
					left_ctrl   = iy_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
				end
				8'b00010001: // page 3 // cmps
				begin
					left_ctrl   = sp_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
				end
				default: // page 1 // cmpx
				begin
					left_ctrl   = ix_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_sub16;
					cc_ctrl     = load_cc;
				end
				endcase
			end
			4'b1101: ;// bsr / jsr

			4'b1110: // ldx
			begin
				case (pre_code)
				8'b00010000: // page 2 // ldy
				begin
					left_ctrl   = iy_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_ld16;
					cc_ctrl     = load_cc;
					iy_ctrl     = load_iy;
				end
				default:   // page 1 // ldx
				begin
					left_ctrl   = ix_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_ld16;
					cc_ctrl     = load_cc;
					ix_ctrl     = load_ix;
				end
				endcase
			end
			4'b1111: // stx
			begin
				case (pre_code)
				8'b00010000: // page 2 // sty
				begin
					left_ctrl   = iy_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_st16;
					cc_ctrl     = load_cc;
				end
				default:     // page 1 // stx
				begin
					left_ctrl   = ix_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_st16;
					cc_ctrl     = load_cc;
				end
				endcase
			end
			default: ;

			endcase
		end
		2'b11: // accb dual op
		begin
			case (op_code[3:0])
			4'b0000: // subb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_sub8;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b0001: // cmpb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_sub8;
				cc_ctrl     = load_cc;
			end
			4'b0010: // sbcb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_sbc;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b0011: // addd
			begin
				left_ctrl   = accd_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_add16;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_hi_acca;
				accb_ctrl   = load_accb;
			end
			4'b0100: // andb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_and;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b0101: // bitb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_and;
				cc_ctrl     = load_cc;
			end
			4'b0110: // ldab
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_ld8;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b0111: // stab
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_st8;
				cc_ctrl     = load_cc;
			end
			4'b1000: // eorb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_eor;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b1001: // adcb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_adc;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b1010: // orab
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_ora;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b1011: // addb
			begin
				left_ctrl   = accb_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_add8;
				cc_ctrl     = load_cc;
				accb_ctrl   = load_accb;
			end
			4'b1100: // ldd
			begin
				left_ctrl   = accd_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_ld16;
				cc_ctrl     = load_cc;
				acca_ctrl   = load_hi_acca;
				accb_ctrl   = load_accb;
			end
			4'b1101: // std
			begin
				left_ctrl   = accd_left;
				right_ctrl  = md_right;
				alu_ctrl    = alu_st16;
				cc_ctrl     = load_cc;
			end
			4'b1110: // ldu
			begin
				case (pre_code)
				8'b00010000: // page 2 // lds
				begin
					left_ctrl   = sp_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_ld16;
					cc_ctrl     = load_cc;
					sp_ctrl     = load_sp;
				end
				default: // page 1 // ldu
				begin
					left_ctrl   = up_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_ld16;
					cc_ctrl     = load_cc;
					up_ctrl     = load_up;
				end
				endcase
			end
			4'b1111:
			begin
				case (pre_code)
				8'b00010000: // page 2 // sts
				begin
					left_ctrl   = sp_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_st16;
					cc_ctrl     = load_cc;
				end
				default:     // page 1 // stu
				begin
					left_ctrl   = up_left;
					right_ctrl  = md_right;
					alu_ctrl    = alu_st16;
					cc_ctrl     = load_cc;
				end
				endcase
			end
			default: ;

			endcase
		end
		default: ;

		endcase

	end // first instruction cycle (fic)
	lic_out = lic;
end
endmodule
