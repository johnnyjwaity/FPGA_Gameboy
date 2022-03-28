module emulator(	  input	       MAX10_CLK1_50, 
					  input  [1:0]  KEY,
					  input  [7:0]  SW,
					  output [7:0]  LEDR,
					  output [12:0] DRAM_ADDR,
					  output [1:0]  DRAM_BA,
					  output        DRAM_CAS_N,
					  output	    DRAM_CKE,
					  output	    DRAM_CS_N,
					  inout  [15:0] DRAM_DQ,
					  output		DRAM_LDQM,
					  output 		DRAM_UDQM,
					  output	    DRAM_RAS_N,
					  output	    DRAM_WE_N,
					  output	    DRAM_CLK,
					  output             VGA_HS,
						output             VGA_VS,
						output   [ 3: 0]   VGA_R,
						output   [ 3: 0]   VGA_G,
						output   [ 3: 0]   VGA_B,
						///////// ARDUINO /////////
      inout    [15: 0]   ARDUINO_IO,
      inout              ARDUINO_RESET_N 
				  
				  );
				  logic SPI0_CS_N, SPI0_SCLK, SPI0_MISO, SPI0_MOSI, USB_GPX, USB_IRQ, USB_RST;
				  
				  assign ARDUINO_IO[10] = SPI0_CS_N;
	assign ARDUINO_IO[13] = SPI0_SCLK;
	assign ARDUINO_IO[11] = SPI0_MOSI;
	assign ARDUINO_IO[12] = 1'bZ;
	assign SPI0_MISO = ARDUINO_IO[12];
	
	assign ARDUINO_IO[9] = 1'bZ; 
	assign USB_IRQ = ARDUINO_IO[9];
		
	//Assignments specific to Circuits At Home UHS_20
	assign ARDUINO_RESET_N = USB_RST;
	assign ARDUINO_IO[7] = USB_RST;//USB reset 
	assign ARDUINO_IO[8] = 1'bZ; //this is GPX (set to input)
	assign USB_GPX = 1'b0;//GPX is not needed for standard USB host - set to 0 to prevent interrupt
	
	//Assign uSD CS to '1' to prevent uSD card from interfering with USB Host (if uSD card is plugged in)
	assign ARDUINO_IO[6] = 1'b1;
				  
	final_soc finalsoc (.clk_clk(MAX10_CLK1_50),
									 .reset_reset_n(KEY[0]),
									//SDRAM
									.sdram_clk_clk(DRAM_CLK),                            //clk_sdram.clk
									.sdram_wire_addr(DRAM_ADDR),                         //sdram_wire.addr
									.sdram_wire_ba(DRAM_BA),                             //.ba
									.sdram_wire_cas_n(DRAM_CAS_N),                       //.cas_n
									.sdram_wire_cke(DRAM_CKE),                           //.cke
									.sdram_wire_cs_n(DRAM_CS_N),                         //.cs_n
									.sdram_wire_dq(DRAM_DQ),                             //.dq
									.sdram_wire_dqm({DRAM_UDQM,DRAM_LDQM}),              //.dqm
									.sdram_wire_ras_n(DRAM_RAS_N),                       //.ras_n
									.sdram_wire_we_n(DRAM_WE_N),
									.vga_port_blue(VGA_B),    //   vga_port.blue
									.vga_port_green(VGA_G),   //           .green
									.vga_port_red(VGA_R),     //           .red
									.vga_port_vs(VGA_VS),      //           .vs
									.vga_port_hs(VGA_HS),
							//USB SPI	
		.spi0_SS_n(SPI0_CS_N),
		.spi0_MOSI(SPI0_MOSI),
		.spi0_MISO(SPI0_MISO),
		.spi0_SCLK(SPI0_SCLK),
		
		//USB GPIO
		.usb_rst_export(USB_RST),
		.usb_irq_export(USB_IRQ),
		.usb_gpx_export(USB_GPX),
.key_external_connection_export    (KEY),		
									 );
									 
	assign LEDR[0] = 1;
	
endmodule
