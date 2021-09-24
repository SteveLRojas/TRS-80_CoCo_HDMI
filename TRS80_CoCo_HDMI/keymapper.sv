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
//* Key mapper module for TRS80 Color Computer     *
//* Copyright (C) 2020 Esteban Looser-Rojas.       *
//* Converts scan codes from PS/2 keyboards to     *
//* corresponding row and column address of TRS80  *
//* keypad.                                        *
//**************************************************
module keymapper(input logic[7:0] scan_code, input logic shift_key, control_key, caps_lock, alt_key, special_make, output logic[6:0] keycode);
logic caps, shift_out;
logic [2:0] row, col;
assign caps = shift_key ^ caps_lock ^ control_key ^ alt_key;
assign keycode = {~shift_out, col, row};

always_comb
begin
	shift_out = caps;
	case (scan_code)
		8'h66:	//backspace key
		begin
			row = 3'h3;
			col = 3'h5;
		end
		8'h29:	//space key
		begin
			row = 3'h3;
			col = 3'h7;
		end
		8'h0d:	//tab key
		begin
			row = 3'h3;
			col = 3'h7;
		end
		8'h5a:	//enter key
		begin
			row = 3'h6;
			col = 3'h0;
		end
		8'h76:	//escape key
		begin
			row = 3'h6;
			col = 3'h2;
		end
		8'h1c:	//A key
		begin
			row = 3'h0;
			col = 3'h1;
		end
		8'h32:	//B key
		begin
			row = 3'h0;
			col = 3'h2;
		end
		8'h21:	//C key
		begin
			row = 3'h0;
			col = 3'h3;
		end
		8'h23:	//D key
		begin
			row = 3'h0;
			col = 3'h4;
		end
		8'h24:	//E key
		begin
			row = 3'h0;
			col = 3'h5;
		end
		8'h2b:	//F key
		begin
			row = 3'h0;
			col = 3'h6;
		end
		8'h34:	//G key
		begin
			row = 3'h0;
			col = 3'h7;
		end
		8'h33:	//H key
		begin
			row = 3'h1;
			col = 3'h0;
		end
		8'h43:	//I key
		begin
			row = 3'h1;
			col = 3'h1;
		end
		8'h3b:	//J key
		begin
			row = 3'h1;
			col = 3'h2;
		end
		8'h42:	//K key
		begin
			row = 3'h1;
			col = 3'h3;
		end
		8'h4b:	//L key
		begin
			row = 3'h1;
			col = 3'h4;
		end
		8'h3a:	//M key
		begin
			row = 3'h1;
			col = 3'h5;
		end
		8'h31:	//N key
		begin
			row = 3'h1;
			col = 3'h6;
		end
		8'h44:	//O key
		begin
			row = 3'h1;
			col = 3'h7;
		end
		8'h4d:	//P key
		begin
			row = 3'h2;
			col = 3'h0;
		end
		8'h15:	//Q key
		begin
			row = 3'h2;
			col = 3'h1;
		end
		8'h2d:	//R key
		begin
			row = 3'h2;
			col = 3'h2;
		end
		8'h1b:	//S key
		begin
			row = 3'h2;
			col = 3'h3;
		end
		8'h2c:	//T key
		begin
			row = 3'h2;
			col = 3'h4;
		end
		8'h3c:	//U key
		begin
			row = 3'h2;
			col = 3'h5;
		end
		8'h2a:	//V key
		begin
			row = 3'h2;
			col = 3'h6;
		end
		8'h1d:	//W key
		begin
			row = 3'h2;
			col = 3'h7;
		end
		8'h22:	//X key
		begin
			row = 3'h3;
			col = 3'h0;
		end
		8'h35:	//Y key
		begin
			row = 3'h3;
			col = 3'h1;
		end
		8'h1a:	//Z key
		begin
			row = 3'h3;
			col = 3'h2;
		end
		8'h45:	//0 key
		begin
			if(shift_key)
			begin
				row = 3'h5;
				col = 3'h1;
				shift_out = 1'b1;
			end
			else
			begin
				row = 3'h4;
				col = 3'h0;
			end
		end
		8'h16:	//1 key
		begin
			row = 3'h4;
			col = 3'h1;
		end
		8'h1e:	//2 key
		begin
			if(shift_key)
			begin
				row = 3'h0;
				col = 3'h0;
				shift_out = 1'b0;
			end
			else
			begin
				row = 3'h4;
				col = 3'h2;
			end
		end
		8'h26:	//3 key
		begin
			row = 3'h4;
			col = 3'h3;
		end
		8'h25:	//4 key
		begin
			row = 3'h4;
			col = 3'h4;
		end
		8'h2e:	//5 key
		begin
			row = 3'h4;
			col = 3'h5;
		end
		8'h36:	//6 key
		begin
			if(shift_key)
			begin
				row = 3'h3;
				col = 3'h3;
				shift_out = 1'b0;
			end
			else
			begin
				row = 3'h4;
				col = 3'h6;
			end
		end
		8'h3d:	//7 key
		begin
			if(shift_key)
			begin
				row = 3'h4;
				col = 3'h6;
				shift_out = 1'b1;
			end
			else
			begin
				row = 3'h4;
				col = 3'h7;
			end
		end
		8'h3e:	//8 key
		begin
			if(shift_key)
			begin
				row = 3'h5;
				col = 3'h2;
				shift_out = 1'b1;
			end
			else
			begin
				row = 3'h5;
				col = 3'h0;
			end
		end
		8'h46:	//9 key
		begin
			if(shift_key)
			begin
				row = 3'h5;
				col = 3'h0;
				shift_out = 1'b1;
			end
			else
			begin
				row = 3'h5;
				col = 3'h1;
			end
		end
		8'h4e:	//- key
		begin
			row = 3'h5;
			col = 3'h5;
		end
		8'h55:	//= key
		begin
			if(shift_key)
			begin
				row = 3'h5;
				col = 3'h3;
			end
			else
			begin
				row = 3'h5;
				col = 3'h5;
			end
			shift_out = 1'b1;
		end
		8'h54:	//[ key
		begin
			row = 3'h5;
			col = 3'h0;
			shift_out = 1'b1;
		end
		8'h71:	//delete key and kp . key
		begin
			if(special_make)
			begin
				row = 3'h3;	//backspace
				col = 3'h5;
			end
			else
			begin
				row = 3'h5;	// . key
				col = 3'h6;
			end
		end
		8'h75:	//up key and kp 8 key
		begin
			if(special_make)
			begin
				row = 3'h3;
				col = 3'h3;
			end
			else
			begin
				row = 3'h5;
				col = 3'h0;
			end
		end
		8'h6b:	//left key and kp 4 key
		begin
			if(special_make)
			begin
				row = 3'h3;
				col = 3'h5;
			end
			else
			begin
				row = 3'h4;
				col = 3'h4;
			end
		end
		8'h72:	//down key and kp 2 key
		begin
			if(special_make)
			begin
				row = 3'h3;
				col = 3'h4;
			end
			else
			begin
				row = 3'h4;
				col = 3'h2;
			end
		end
		8'h74:	//right key and kp 6 key
		begin
			if(special_make)
			begin
				row = 3'h3;
				col = 3'h6;
			end
			else
			begin
				row = 3'h4;
				col = 3'h6;
			end
		end
		8'h7c:	//kp * key
		begin
			row = 3'h5;
			col = 3'h2;
			shift_out = 1'b1;
		end
		8'h7b:	//kp - key
		begin
			row = 3'h5;
			col = 3'h5;
		end
		8'h79:	//kp + key
		begin
			row = 3'h5;
			col = 3'h3;
			shift_out = 1'b1;
		end
		8'h70:	//kp 0 key
		begin
			row = 3'h4;
			col = 3'h0;
		end
		8'h69:	//kp 1 key
		begin
			row = 3'h4;
			col = 3'h1;
		end
		8'h7a:	//kp 3 key
		begin
			row = 3'h4;
			col = 3'h3;
		end
		8'h73:	//kp 5 key
		begin
			row = 3'h4;
			col = 3'h5;
		end
		8'h6c:	//kp 7 key
		begin
			row = 3'h4;
			col = 3'h7;
		end
		8'h7d:	//kp 9 key
		begin
			row = 3'h5;
			col = 3'h1;
		end
		8'h5b:	//] key
		begin
			row = 3'h5;
			col = 3'h1;
			shift_out = 1'b1;
		end
		8'h4c:	//; key
		begin
			if(shift_key)
			begin
				row = 3'h5;
				col = 3'h2;
				shift_out = 1'b0;
			end
			else
			begin
				row = 3'h5;
				col = 3'h3;
			end
		end
		8'h52:	//' key
		begin
			if(shift_key)
			begin
				row = 3'h4;
				col = 3'h2;
			end
			else
			begin
				row = 3'h4;
				col = 3'h7;
			end
			shift_out = 1'b1;
		end
		8'h41:	//, key
		begin
			row = 3'h5;
			col = 3'h4;
		end
		8'h49:	//. key
		begin
			row = 3'h5;
			col = 3'h6;
		end
		8'h4a:	/// key
		begin
			row = 3'h5;
			col = 3'h7;
		end
		default:
		begin
			row = 3'h7;
			col = 3'h7;
			shift_out = 1'b0;
		end
	endcase
end
endmodule
