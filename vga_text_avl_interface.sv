/************************************************************************
Avalon-MM Interface VGA Text mode display

Register Map:
0x000-0x0257 : VRAM, 80x30 (2400 byte, 600 word) raster order (first column then row)
0x258        : control register

VRAM Format:
X->
[ 31  30-24][ 23  22-16][ 15  14-8 ][ 7    6-0 ]
[IV3][CODE3][IV2][CODE2][IV1][CODE1][IV0][CODE0]

IVn = Draw inverse glyph
CODEn = Glyph code from IBM codepage 437

Control Register Format:
[[31-25][24-21][20-17][16-13][ 12-9][ 8-5 ][ 4-1 ][   0    ] 
[[RSVD ][FGD_R][FGD_G][FGD_B][BKG_R][BKG_G][BKG_B][RESERVED]

VSYNC signal = bit which flips on every Vsync (time for new frame), used to synchronize software
BKG_R/G/B = Background color, flipped with foreground when IVn bit is set
FGD_R/G/B = Foreground color, flipped with background when Inv bit is set

************************************************************************/


module vga_text_avl_interface (
	// Avalon Clock Input, note this clock is also used for VGA, so this must be 50Mhz
	// We can put a clock divider here in the future to make this IP more generalizable
	input logic CLK,
	
	// Avalon Reset Input
	input logic RESET,
	
	// Avalon-MM Slave Signals
	input  logic AVL_READ,					// Avalon-MM Read
	input  logic AVL_WRITE,					// Avalon-MM Write
	input  logic AVL_CS,					// Avalon-MM Chip Select
	input  logic [3:0] AVL_BYTE_EN,			// Avalon-MM Byte Enable
	input  logic [11:0] AVL_ADDR,			// Avalon-MM Address
	input  logic [31:0] AVL_WRITEDATA,		// Avalon-MM Write Data
	output logic [31:0] AVL_READDATA,		// Avalon-MM Read Data
	
	// Exported Conduit (mapped to VGA port - make sure you export in Platform Designer)
	output logic [3:0]  red, green, blue,	// VGA color channels (mapped to output pins in top-level)
	output logic hs, vs						// VGA HS/VS
);

logic[10:0] addrB;
logic[31:0] ram_out;

vram ram(.address_a(AVL_ADDR[10:0]), .address_b(addrB), .byteena_a(AVL_BYTE_EN), .byteena_b(4'hF), .clock(CLK), .data_a(AVL_WRITEDATA), .data_b(), .rden_a(AVL_READ & AVL_CS), .rden_b(1'b1), .wren_a(AVL_WRITE & AVL_CS & (~AVL_ADDR[11])), .wren_b(1'b0), .q_a(AVL_READDATA), .q_b(ram_out));
//ppu gpu(.clk(CLK), .rst(RESET), .pixel(red));
logic CLK_25, BLANK;
logic [9:0] DrawX, DrawY;
logic [3:0] Mod16;
logic [1:0] color;

logic [3:0] redTemp;
logic [3:0] greenTemp;
logic [3:0] blueTemp;


vga_controller vcont(.Clk(CLK), .Reset(RESET), .hs(hs), .vs(vs), .pixel_clk(CLK_25), .blank(BLANK), .sync(), .DrawX(DrawX), .DrawY(DrawY));

calc_addr ca(.DrawX(DrawX), .DrawY(DrawY), .addr(addrB));
logic[1:0] finder_out;
inner_finder inf(.r(ram_out), .DrawX(DrawX), .pixel(finder_out));
logic[3:0] cm_red, cm_blue, cm_green;
color_mapper cm(.pixel(finder_out), .r(cm_red), .g(cm_green), .b(cm_blue));
blank_check bc(.blank(BLANK), .r(cm_red), .g(cm_green), .b(cm_blue), .DrawX(DrawX), .ro(redTemp), .go(greenTemp), .bo(blueTemp));


sync syncRed(.Clk(CLK_25), .d(redTemp), .q(red));
sync syncGreen(.Clk(CLK_25), .d(greenTemp), .q(green));
sync syncBlue(.Clk(CLK_25), .d(blueTemp), .q(blue));



//always_ff begin
//	if(~BLANK || DrawX >= 160 || DrawY >= 144)
//		begin
//			redTemp = 4'h0;
//			greenTemp = 4'h0;
//			blueTemp = 4'h0;
//		end
//	else
//		begin
//			addrB = DrawX / 4;
//			addrB += (DrawY * 10);
//			Mod16 = DrawX % 16;
//			case (Mod16)
//			4'b0000: color = ram_out[31:30];
//			4'b0001: color = ram_out[29:28];
//			4'b0010: color = ram_out[27:26];
//			4'b0011: color = ram_out[25:24];
//			4'b0100: color = ram_out[23:22];
//			4'b0101: color = ram_out[21:20];
//			4'b0110: color = ram_out[19:18];
//			4'b0111: color = ram_out[17:16];
//			4'b1000: color = ram_out[15:14];
//			4'b1001: color = ram_out[13:12];
//			4'b1010: color = ram_out[11:10];
//			4'b1011: color = ram_out[9:8];
//			4'b1100: color = ram_out[7:6];
//			4'b1101: color = ram_out[5:4];
//			4'b1110: color = ram_out[3:2];
//			4'b1111: color = ram_out[1:0];
//			endcase
//			case(color)
//			2'b00:
//				begin
//					redTemp = 4'hF;
//					greenTemp = 4'hF;
//					blueTemp = 4'hF;
//				end
//			2'b01:
//				begin
//					redTemp = 4'hA;
//					greenTemp = 4'hA;
//					blueTemp = 4'hA;
//				end
//			2'b10:
//				begin
//					redTemp = 4'h5;
//					greenTemp = 4'h5;
//					blueTemp = 4'h5;
//				end
//			2'b11:
//				begin
//					redTemp = 4'h0;
//					greenTemp = 4'h0;
//					blueTemp = 4'h0;
//				end
//			endcase
//			
//			
//		end
//
//end




endmodule

module calc_addr(input[9:0] DrawX, DrawY,
						output [10:0] addr);
assign addr = (DrawY * 10) + (DrawX >> 4);
endmodule

module inner_finder(input logic[31:0] r, 
							input logic[9:0] DrawX,
							output logic[1:0] pixel);
	always @ * begin
		case(DrawX & 4'hF)
		0:  pixel = r[31:30];
		1:  pixel = r[29:28];
		2:  pixel = r[27:26];
		3:  pixel = r[25:24];
		4:  pixel = r[23:22];
		5:  pixel = r[21:20];
		6:  pixel = r[19:18];
		7:  pixel = r[17:16];
		8:  pixel = r[15:14];
		9:  pixel = r[13:12];
		10:  pixel = r[11:10];
		11:  pixel = r[9:8];
		12:  pixel = r[7:6];
		13:  pixel = r[5:4];
		14:  pixel = r[3:2];
		15:  pixel = r[1:0];
		endcase
	end
endmodule
					



module blank_check(input logic blank,
						input logic[3:0] r, g, b, input[9:0] DrawX,
						output logic[3:0] ro, go, bo);
	assign ro = blank ? ((r == 4'hF && DrawX % 16 == 0) ? 4'h0 : r) : 4'h0;
	assign go = blank ? g : 4'h0;
	assign bo = blank ? b : 4'h0;
						
endmodule



module color_map_blank(input logic blank, input logic[9:0] DrawX, DrawY,
								output logic [3:0] r, g, b);
	
	assign r = ((DrawX % 3) == 0 && blank) ? 4'hF : 4'h0;
	assign g = ((DrawX % 3) == 1 && blank) ? 4'hF : 4'h0;
	assign b = ((DrawX % 3) == 2 && blank) ? 4'hF : 4'h0;
	
endmodule

