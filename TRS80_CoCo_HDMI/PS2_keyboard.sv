//                     /\         /\__
//                   // \       (  0 )_____/\            __
//                  // \ \     (vv          o|          /^v\
//                //    \ \   (vvvv  ___-----^        /^^/\vv\
//              //  /     \ \ |vvvvv/               /^^/    \v\
//             //  /       (\\/vvvv/              /^^/       \v\
//            //  /  /  \ (  /vvvv/              /^^/---(     \v\
//           //  /  /    \( /vvvv/----(O        /^^/           \v\
//          //  /  /  \  (/vvvv/               /^^/             \v|
//        //  /  /    \( vvvv/                /^^/               ||
//       //  /  /    (  vvvv/                 |^^|              //
//      //  / /    (  |vvvv|                  /^^/            //
//     //  / /   (    \vvvvv\          )-----/^^/           //
//    // / / (          \vvvvv\            /^^^/          //
//   /// /(               \vvvvv\        /^^^^/          //
//  ///(              )-----\vvvvv\    /^^^^/-----(      \\
// //(                        \vvvvv\/^^^^/               \\
///(                            \vvvv^^^/                 //
//                                \vv^/         /        //
//                                             /<______//
//                                            <<<------/
//                                             \<
//                                              \
//**************************************************
//* Keyboard module for TRS80 model MC10           *
//* Copyright (C) 2019 Esteban Looser-Rojas.       *
//* Connects to standard PS/2 keyboard and emulates*
//* the keyboard matrix of the TRS80 MC10 computer.*
//* Some keys were left unused.                    *
//**************************************************
module PS2_keyboard(input logic clk, reset, input logic ps2_clk_d, ps2_data_d, output logic[6:0] key_code, output logic ps2_clk_q, ps2_data_q);
logic[7:0] rx_data;
logic[7:0] tx_data;
logic rx_ready;
logic tx_ready;
logic tx_req;
logic [2:0] status;
logic caps_lock, next_caps_lock, shift_key, next_shift_key, num_lock, next_num_lock, scroll_lock, next_scroll_lock, control_key, next_control_key, alt_key, next_alt_key; 
logic[7:0] scan_code, next_scan_code;
logic special_make, next_special_make;
enum logic[3:0] {read_first, process_first, read_second, process_second, read_third, process_third} state, next_state;

assign status = {caps_lock, num_lock, scroll_lock};
always_ff @(posedge clk)
begin
	if(reset)
		state <= read_first;
	else
		state <= next_state;
	scan_code <= next_scan_code;
	caps_lock <= next_caps_lock;
	shift_key <= next_shift_key;
	num_lock <= next_num_lock;
	scroll_lock <= next_scroll_lock;
	control_key <= next_control_key;
	alt_key <= next_alt_key;
	special_make <= next_special_make;
end

always_comb
begin
//default values
next_state = state;
next_scan_code = scan_code;
next_caps_lock = caps_lock;
next_shift_key = shift_key;
next_num_lock = num_lock;
next_scroll_lock = scroll_lock;
next_control_key = control_key;
next_alt_key = alt_key;
next_special_make = special_make;
	case (state)
		read_first:
		begin
			if(rx_ready)
				next_state = process_first;
			else
				next_state = state;
		end
		process_first:
		begin
			if(rx_data == 8'he0 || rx_data == 8'hf0)	//break code or special break code
				next_state = read_second;
			else
				next_state = read_first;
		end
		read_second:
		begin
			if(rx_ready)
				next_state = process_second;
			else
				next_state = state;
		end
		process_second:
		begin
			if(rx_data == 8'hf0)	//special break code
				next_state = read_third;
			else
				next_state = read_first;
		end
		read_third:
		begin
			if(rx_ready)
				next_state = process_third;
			else
				next_state = state;
		end
		process_third: next_state = read_first;
		default: next_state = state;
	endcase
	
	case (state)
		process_first:
		begin
			if(rx_data == 8'he0 || rx_data == 8'hf0)
				next_scan_code = 8'h00;
			else
				next_scan_code = rx_data;
			if(rx_data == 8'he0)
				next_special_make = 1'b1;
			else
				next_special_make = 1'b0;
			if(rx_data == 8'h12 || rx_data == 8'h59)	//handle shift keys
				next_shift_key = 1'b1;
			if(rx_data == 8'h58)	//handle caps lock
				next_caps_lock = ~caps_lock;
			if(rx_data == 8'h7e)	//handle scroll lock
				next_scroll_lock = ~scroll_lock;
			if(rx_data == 8'h77)	//handle num lock
				next_num_lock = ~num_lock;
			if(rx_data == 8'h14)	//handle left control key
				next_control_key = 1'b1;
			if(rx_data == 8'h11)	//handle left alt key
				next_alt_key = 1'b1;
		end
		process_second:
		begin
			if(special_make)
				next_scan_code = rx_data;
			if(rx_data == 8'h12 || rx_data == 8'h59)	//handle shift keys
				next_shift_key = 1'b0;
			if(special_make)
			begin
				if(rx_data == 8'h14)	//handle right control key
					next_control_key = 1'b1;
				if(rx_data == 8'h11)	//handle right alt key
					next_alt_key = 1'b1;
			end
			else
			begin
				if(rx_data == 8'h14)	//handle left control
					next_control_key = 1'b0;
				if(rx_data == 8'h11)	//handle left alt key
					next_alt_key = 1'b0;
			end
		end
		process_third:
		begin
			if(rx_data == 8'h14)	//handle right control key
				next_control_key = 1'b0;
			if(rx_data == 8'h11)	//handle right alt key
				next_alt_key = 1'b0;
		end
		default: ;
	endcase
end

ps2_host ps2_host_inst(.clk(clk), .rst(reset), .ps2_clk_d, .ps2_data_d, .ps2_clk_q, .ps2_data_q, .tx_data, .rx_data, .rx_ready, .tx_ready, .tx_req);
keymapper keymapper_inst(.scan_code, .shift_key, .control_key, .caps_lock, .alt_key, .special_make, .keycode(key_code));
kb_indicators kb_led(.clk, .reset, .status, .tx_ready, .tx_data, .send_req(tx_req));
endmodule