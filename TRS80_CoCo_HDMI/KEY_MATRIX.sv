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
//* Key matrix module for TRS80 Color Computer     *
//* Copyright (C) 2020 Esteban Looser-Rojas.       *
//* Uses row and column addresses to emulate the   *
//* TRS-80 key matrix. Handles shift key separately*
//* so it can be pressed with other keys.          *
//**************************************************
module KEY_MATRIX(input logic[7:0] col_select, input logic[6:0] key_code, output logic[6:0] key_out);
logic[7:0] column_exp;
logic[6:0] row_exp;
logic[2:0] column, row;
assign column = key_code[5:3];
assign row = key_code[2:0];
always_comb
begin
	unique case(column)
	3'h0: column_exp = 8'b11111110;
	3'h1: column_exp = 8'b11111101;
	3'h2: column_exp = 8'b11111011;
	3'h3: column_exp = 8'b11110111;
	3'h4: column_exp = 8'b11101111;
	3'h5: column_exp = 8'b11011111;
	3'h6: column_exp = 8'b10111111;
	3'h7: column_exp = 8'b01111111;
	endcase
	
	unique case(row)
	3'h0: row_exp = 7'b1111110;
	3'h1: row_exp = 7'b1111101;
	3'h2: row_exp = 7'b1111011;
	3'h3: row_exp = 7'b1110111;
	3'h4: row_exp = 7'b1101111;
	3'h5: row_exp = 7'b1011111;
	3'h6: row_exp = 7'b0111111;
	3'h7: row_exp = 7'b1111111;	//row does not exist
	endcase
	
	if(column_exp == col_select)
		key_out = row_exp;
	else
		key_out = 7'h7f;
	if(col_select == 8'b01111111)	//col 7
		key_out[6] = key_code[6];	//shift key
end
endmodule