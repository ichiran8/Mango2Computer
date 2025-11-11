`timescale 1ns / 10ps

module vga_test (
	input logic clk_25mhz,	// clock 25MHz
	input logic clk_12mhz,  // clock 12.5MHz
	input logic [7:0] data_in,	    // data from RAM
	output logic [15:0] vid_addr,	// video memory addr
	output logic read_mem,  // read memory when high
	output logic hsync,	    // horizontal sync
	output logic vsync,	    // vertical sync
	
	input logic text,		// text mode
	input logic hires,	    // hires/lores graphics mode
	input logic mix,		// mix mode
	input logic page2,	    // page2/page1
	
	input logic color,	// color/blackwhite
	
	output logic [11:0] video	// RGB444 video output
);
// internal signals
logic hsync_i = 1;
logic hde = 0;
logic [2:0]pixcnt = 0;
logic pixtc;
logic [5:0]chrcnt = 0;
logic nl;
logic [9:0]vc = 523;	
logic vsync_i = 1;
logic vde = 0;
logic mixed_line = 0;

// horizontal timing
assign hsync = hsync_i;
// pixel counter
always_ff @(posedge clk_12mhz) begin
	if (pixtc | nl) pixcnt <= 0;
	else pixcnt <= (pixcnt == 6) ? 0 : pixcnt + 1;
end

assign pixtc = (pixcnt == 6);

// count 40 characters per line
always_ff @(posedge clk_12mhz) begin
	if (nl) chrcnt <= 0;
	else if (pixtc) chrcnt <= chrcnt + 1;
end

assign nl = (chrcnt == 57);

// horizontal display enable / hsync for VGA
always_ff @(posedge clk_12mhz) begin
	hde <= nl ? 1 : ((pixtc & (chrcnt == 39)) ? 0 : hde);
	if (pixtc) begin
		if (chrcnt == 43) hsync_i <= 0;
		if (chrcnt == 51) hsync_i <= 1;
	end
end

// vertical timing
assign vsync = vsync_i;

always_ff @(posedge hde) begin
	if (vc == 10'd524) begin vc <= 0; vde <= 1; mixed_line <= 0; end 
	else vc <= vc + 1;
	if (vc == 10'd319) mixed_line <= 1;
	if (vc == 10'd383) vde <= 0;
	if (vc == 10'd441) vsync_i <= 0;
	if (vc == 10'd443) vsync_i <= 1;
end

// display enable and data read from memory
logic de;
assign de = hde & vde;
assign read_mem = de & (pixcnt == 0);

// video intermediate sum for addressing
logic [3:0] sum;
assign sum = {vc[8:7], vc[8:7]} + {1'b0, chrcnt[5:3]};

logic [9:0] txtva;
assign txtva = {vc[6:4], sum, chrcnt[2:0]};
	
// graphics enable (or text when disabled)
logic gr_en;
assign gr_en = ~text & hires & (~(mix & mixed_line));

// video address for memory
assign vid_addr[15:0] = {gr_en ? {1'b0, page2, ~page2, vc[3:1]} : {4'b0000, page2, ~page2}, txtva};
	
// character ROM
logic [4:0] osc = 0;
always_ff @(posedge vsync_i) begin // 2 Hz flash for characters
	osc <= (osc == 5'd30) ? 5'd1 : (osc + 1);
end

logic [7:0] grd;
logic invtxt;
always_comb begin
	case ({grd[7:6]})
		2'b00:   invtxt = 1;
		2'b01:   invtxt = osc[4];
		default: invtxt = 0;
	endcase
end

logic [6:0] charout;
logic [9:0] charaddr;
assign charaddr = {1'b0, data_in[5:0], vc[3:1]};

character_rom CHAR (.clk(clk_12mhz), .char_addr(charaddr), .char_data_out(charout));

logic [6:0] txtout;
assign txtout = invtxt ? (~charout[6:0]) : charout[6:0];

// graphics register
always_ff @(posedge clk_12mhz) begin
	if (read_mem) grd <= data_in;
end

// pixel display information
logic [6:0] pixd;
assign pixd = gr_en ? grd[6:0] : txtout;

logic ld1; 
assign ld1 = (pixcnt == 1) & de & (~clk_12mhz);

logic [6:0] sh;
logic rd7 = 0;

// shift register for display pixels
always_ff @(posedge clk_25mhz) begin
	sh <= (ld1) ? pixd : ((~clk_12mhz) ? {1'b0, sh[6:1]} : sh);
end

always_ff @(posedge clk_25mhz) begin
	if (ld1) rd7 <= (~text) & grd[7];
end

logic dpix;
always_ff @(posedge clk_25mhz) begin
	dpix <= sh[0];
end

logic hrpixel; // hires half pixel cycle
assign hrpixel = rd7 ? dpix : sh[0];

logic lrload = 0;
always_ff @(posedge clk_25mhz) begin 
	lrload <= (pixcnt == 1) & (clk_12mhz); // lores mode
end

logic [3:0] lrsh;
logic [3:0] lrdata;
assign lrdata = de ? (vc[3] ? grd[7:4] : grd[3:0]) : 0;
always_ff @(posedge clk_25mhz) begin 
	lrsh <= lrload ? lrdata : {lrsh[0], lrsh[3:1]};
end

logic lrdchr = 0;	// lores mode handling for even and odd bytes
always_ff @(posedge clk_12mhz) begin 
	if (pixcnt==1) lrdchr <= chrcnt[0];
end

logic lrpixel;
assign lrpixel = lrdchr ? lrsh[2] : lrsh[0];

// video output pixel (Apple 2 composite)
logic pixel;
assign pixel = (text | hires | (mix & mixed_line)) ? hrpixel : lrpixel; 

logic [1:0] cq = 0; // NTSC video
always_ff @(posedge clk_25mhz) begin
	cq <= nl ? 2'b00 : {~cq[0], cq[1]};
end

logic cp0, cp1, cp2, cp3;
assign {cp0, cp1, cp2, cp3} = {cq[0], ~cq[1], ~cq[0], cq[1]}; // phase 0, 90, 180, 270


logic [2:0] pix;
always_ff @(posedge clk_25mhz) begin
	pix <= {pix[1:0], pixel};
end

logic [3:0] cpix;
assign cpix = (text | ~color) ? 0 : {pix, pixel} ; // color pixels

// Apple 2 NTSC uses YIQ color space
logic Y;
logic [2:0] I;
logic [2:0] Q;
assign Y = pix[1];
assign I = {2'b0, cp3^cpix[0]} + {2'b0, cp0^cpix[1]} + {2'b0, cp1^cpix[2]} + {2'b0, cp2^cpix[3]};
assign Q = {2'b0, cp0^cpix[0]} + {2'b0, cp1^cpix[1]} + {2'b0, cp2^cpix[2]} + {2'b0, cp3^cpix[3]};

logic [11:0] apple_vid;

// convert YIQ color space to modern RGB for VGA
color_rom COLOR (.clk(clk_25mhz), .color_addr({Y, I, Q}), .color_data_out(apple_vid));
	
assign video = apple_vid;

endmodule