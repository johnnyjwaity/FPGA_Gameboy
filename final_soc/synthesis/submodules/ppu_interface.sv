module ppu_interface (
	// Avalon Clock Input, note this clock is also used for VGA, so this must be 50Mhz
	// We can put a clock divider here in the future to make this IP more generalizable
	input logic CLK, CLK_4,
	
	// Avalon Reset Input
	input logic RESET,
	
	// Avalon-MM Slave Signals
	input  logic AVL_READ,					// Avalon-MM Read
	input  logic AVL_WRITE,					// Avalon-MM Write
	input  logic AVL_CS,					// Avalon-MM Chip Select
	input  logic [15:0] AVL_ADDR,			// Avalon-MM Address
	input  logic [7:0] AVL_WRITEDATA,		// Avalon-MM Write Data
	output logic [7:0] AVL_READDATA,		// Avalon-MM Read Data
	
	// Exported Conduit (mapped to VGA port - make sure you export in Platform Designer)
	output logic [3:0]  red, green, blue,	// VGA color channels (mapped to output pins in top-level)
	output logic hs, vs						// VGA HS/VS
);

logic [15:0] ADDR_OFFSET;
logic [1:0] pixel_out;
logic pix_clk, vblank_int, lcdc_int, mmio_rd, mmio_wr, vram_rd, vram_wr, oam_rd, oam_wr, vblank_ack, lcdc_ack, gb_hs, gb_vs, gb_valid, pix_wr;
logic[7:0] mmio_out, vram_out, oam_out;
logic [3:0] redTemp;
logic [3:0] greenTemp;
logic [3:0] blueTemp;

offset of(.in_a(AVL_ADDR), .out_a(ADDR_OFFSET));

mmio_calc mc(.addr(ADDR_OFFSET), .in_rd(AVL_CS & AVL_READ), .in_wr(AVL_CS & AVL_WRITE), .out_rd(mmio_rd), .out_wr(mmio_wr));
vram_calc vc(.addr(ADDR_OFFSET), .in_rd(AVL_CS & AVL_READ), .in_wr(AVL_CS & AVL_WRITE), .out_rd(vram_rd), .out_wr(vram_wr));
oam_calc oc(.addr(ADDR_OFFSET), .in_rd(AVL_CS & AVL_READ), .in_wr(AVL_CS & AVL_WRITE), .out_rd(oam_rd), .out_wr(oam_wr));
color_mapper cm(.pixel(pixel_out), .r(redTemp), .g(greenTemp), .b(blueTemp));

logic[23:0] mix_pix;
assign red = pix_wr ? mix_pix[23:20] : 4'h0;
assign green = pix_wr ? mix_pix[15:12] : 4'h0;
assign blue = pix_wr ? mix_pix[7:4] : 4'h0;
//sync syncRed(.Clk(pix_clk), .d(redTemp), .q(red));
//sync syncGreen(.Clk(pix_clk), .d(greenTemp), .q(green));
//sync syncBlue(.Clk(pix_clk), .d(blueTemp), .q(blue));

vga_mixer vga(
    .clk(CLK),
    .rst(RESET),
    // GameBoy Image Input
    // Its clock need to be phase aligned, integer dividable by VGA clock
    // No clock domain crossing sync has been implemented here.
    .gb_hs(gb_hs),
    .gb_vs(gb_vs),
    .gb_pclk(pix_clk),
    .gb_pdat(pixel_out),
    .gb_valid(gb_valid),
    .gb_en(1'b1),
    // Debugger Char Input
    .dbg_x(),
    .dbg_y(),
    .dbg_char(),
    // VGA signal Output
    .pix_almost_full(hs),
    .pix_next_frame(vs),
    .pix(mix_pix),
    .pix_wr(pix_wr),
    // Debug
    .hold()
    );


ppu p(
    .clk(CLK_4),
    .rst(RESET),
    // MMIO Bus, 0xFF40 - 0xFF4B, always visible to CPU
    .mmio_a(ADDR_OFFSET),
    .mmio_dout(mmio_out),
    .mmio_din(AVL_WRITEDATA),
    .mmio_rd(mmio_rd),
    .mmio_wr(mmio_wr),
    // VRAM Bus, 0x8000 - 0x9FFF
    .vram_a(ADDR_OFFSET),
    .vram_dout(vram_out),
    .vram_din(AVL_WRITEDATA),
    .vram_rd(vram_rd),
    .vram_wr(vram_wr),
    // OAM Bus,  0xFE00 - 0xFE9F
    .oam_a(ADDR_OFFSET),
    .oam_dout(oam_out),
    .oam_din(AVL_WRITEDATA),
    .oam_rd(oam_rd),
    .oam_wr(oam_wr),
    // Interrupt interface
    .int_vblank_req(vblank_int),
    .int_lcdc_req(lcdc_int),
    .int_vblank_ack(vblank_ack),
    .int_lcdc_ack(lcdc_ack),
    // Pixel output
    .cpl(pix_clk), // Pixel Clock, = ~clk
    .pixel(pixel_out), // Pixel Output
    .valid(gb_valid), // Pixel Valid
    .hs(gb_hs), // Horizontal Sync, High Valid
    .vs(gb_vs), // Vertical Sync, High Valid
    //Debug output
    .scx(),
    .scy(),
    .state()
);
    


endmodule

module mmio_calc(input logic[15:0] addr,
						input logic in_rd, in_wr,
						output logic out_rd, out_wr);
					
	assign out_rd = (addr <= 16'hFF4B && addr >= 16'hFF40) ? in_rd : 1'b0;
	assign out_wr = (addr <= 16'hFF4B && addr >= 16'hFF40) ? in_wr : 1'b0;
endmodule

module vram_calc(input logic[15:0] addr,
						input logic in_rd, in_wr,
						output logic out_rd, out_wr);
					
	assign out_rd = (addr <= 16'h9FFF && addr >= 16'h8000) ? in_rd : 1'b0;
	assign out_wr = (addr <= 16'h9FFF && addr >= 16'h8000) ? in_wr : 1'b0;
endmodule

module oam_calc(input logic[15:0] addr,
						input logic in_rd, in_wr,
						output logic out_rd, out_wr);
					
	assign out_rd = (addr <= 16'hFE9F && addr >= 16'hFE00) ? in_rd : 1'b0;
	assign out_wr = (addr <= 16'hFE9F && addr >= 16'hFE00) ? in_wr : 1'b0;
endmodule

module offset(input logic [15:0] in_a,
					output logic [15:0] out_a);
	assign out_a = in_a + 16'h8000;
					
endmodule


