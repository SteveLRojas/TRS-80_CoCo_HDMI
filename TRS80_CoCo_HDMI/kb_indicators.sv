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
//* Controls indicator lights on PS/2 keyboards.   *
//* Some keys were left unused.                    *
//**************************************************
module kb_indicators(
			input logic clk,
			input logic reset,
			input logic [2:0] status,
			input logic tx_ready,
			output logic[7:0] tx_data,
			output logic send_req);
enum logic[3:0] {trig_state, delay_state, send_1_state, long_wait_state, send_2_state} next_state, state;
logic[19:0] counter, next_counter;
logic[7:0] next_tx_data;
logic[2:0] prev_status;

always_ff @(posedge clk)
begin
	if(reset)
	begin
	state <= trig_state;
	end
	else
		state <= next_state;
	counter <= next_counter;
	tx_data <= next_tx_data;
	prev_status <= status;
end

always_comb
begin
	next_state = state;
	next_counter = 20'h00;
	next_tx_data = tx_data;
	send_req = 1'b0;
	case (state)
	trig_state:
	begin
		if(status != prev_status)
			next_state = delay_state;
		next_tx_data = 8'hED;
		next_counter = 20'h00;
	end
	delay_state:
	begin
		next_counter = counter + 20'h01;
		if(counter == 20'hfffff)
			next_state = send_1_state;
	end
	send_1_state:
	begin
		send_req = 1'b1;
		if(tx_ready)
			next_state = long_wait_state;
		else
			next_state = state;
	end
	long_wait_state:
	begin
		next_counter = counter + 20'h01;
		if(counter == 20'hfffff)
			next_state = send_2_state;
		next_tx_data = {5'b00000, status};
	end
	send_2_state:
	begin
		send_req = 1'b1;
		if(tx_ready)
			next_state = trig_state;
		else
			next_state = state;
	end
	default: ;
	endcase
end
endmodule
