`timescale 1ns / 1ps

module keyboard (
    input  logic clk,     // Basys-3 100 MHz
    input  logic PS2Clk,       // PS/2 clock pin
    input  logic PS2Data,      // PS/2 data pin
    input  logic nrst,
    input logic clr_strb,
    output logic [3:0] an,      // seven-seg anodes (active-low)
    output logic [6:0] seg,     // CA..CG (active-low)
    output logic       dp,       // decimal point (active-low)
    output logic shift,
    output logic [7:0] decoded_out,
    output logic oflag
);
    logic [15:0] keycode;
    logic [7:0] next_decoded_out;
    logic        oflag_i, next_oflag;

    // PS/2 receiver
    rcv u_rcv (
        .clk     (clk),
        .kclk    (PS2Clk),
        .nrst    (nrst),
        .kdata   (PS2Data),
        .keycode (keycode),
        .oflag   (oflag_i)
    );

    always_comb begin
        next_oflag = oflag;
        next_decoded_out = decoded_out;
        if(clr_strb)
            next_oflag = 1'b0;
        else if (oflag_i)
            next_oflag = 1'b1;
            next_decoded_out = keycode[7:0];
    end

    always_ff @(posedge clk, negedge nrst) begin
        if(!nrst) begin
           oflag <= 1'b0;
           decoded_out <= 8'b0; 
        end
        else begin
            oflag <= next_oflag;
            decoded_out <= next_decoded_out;
        end
    end

//     Latch the most recent byte on oflag
    logic [7:0] last_byte;
    latch u_latch (
        .clk    (clk),
        .nrst   (nrst),
        .strobe (oflag),
        .din    (keycode[7:0]),  // current byte
        .dout   (last_byte)
    );
    
//    clock_divider #(.DIV(4)) divider(.CLK(clk) , .nRST(nrst) ,.enable(1'b1),.CPUCLK(clkdiv));
    //logic [7:0] decoded;

    // Build display value: [15:8] blank (0x00 shows "00"; to blank use 0x10..?), show right two digits
    // Easiest approach: left two "blank" isn't a hex digit. We can just display 0's or use a blank code.
    // Here we show "--" on left by mapping nibble 0xF to " " would require custom map.
    // Simpler: show only right two digits and keep left two 0.
    assign shift = ((keycode[15:8] == 8'h12) |(keycode[15:8] == 8'h59));
    logic [15:0] disp = {8'h00, last_byte};

    sevenseg u_ss (
        .clk   (clk),
        .value (disp),
        .dp_mask(1'b0),
        .an    (an),
        .seg   (seg),
        .dp    (dp)
    );
endmodule
