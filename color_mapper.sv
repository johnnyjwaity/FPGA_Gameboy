module color_mapper(input logic[1:0] pixel,
							output logic[3:0] r, g, b);
	assign r = 4'hF - (pixel * 5);
	assign g = 4'hF - (pixel * 5);
	assign b = 4'hF - (pixel * 5);
endmodule	