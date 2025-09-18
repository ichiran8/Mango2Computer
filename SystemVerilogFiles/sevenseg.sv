`timescale 1ns / 1ps

module sevenseg (
    input  logic        clk,          // 100 MHz
    input  logic [15:0] value,        // 4 hex nibbles {D3,D2,D1,D0}
    input  logic        dp_mask,      // 1=dp on for active digit? (we'll keep off)
    output logic [3:0]  an,           // active-low anodes
    output logic [6:0]  seg,          // {CA..CG} active-low
    output logic        dp            // active-low
);
    // Refresh divider
    localparam int unsigned TICKS_PER_DIGIT = 25_000; // ~1 kHz per digit
    logic [$clog2(TICKS_PER_DIGIT)-1:0] div = '0;
    logic [1:0] digit = 2'd0;

    always_ff @(posedge clk) begin
        if (div == TICKS_PER_DIGIT-1) begin
            div   <= '0;
            digit <= digit + 2'd1;
        end else begin
            div <= div + 1'b1;
        end
    end

    // Select nibble for current digit (rightmost is digit 0)
    logic [3:0] nibble;
    always_comb begin
        case (digit)
            2'd0: nibble = value[3:0];
            2'd1: nibble = value[7:4];
            2'd2: nibble = value[11:8];
            2'd3: nibble = value[15:12];
        endcase
    end

    // HEX to segments (active-low for Basys-3)
    function automatic logic [6:0] hex_to_segs_al (input logic [3:0] n);
        // seg order: {CA,CB,CC,CD,CE,CF,CG}, 0=ON
        case (n)
            4'h0: hex_to_segs_al = 7'b1000000;
            4'h1: hex_to_segs_al = 7'b1111001;
            4'h2: hex_to_segs_al = 7'b0100100;
            4'h3: hex_to_segs_al = 7'b0110000;
            4'h4: hex_to_segs_al = 7'b0011001;
            4'h5: hex_to_segs_al = 7'b0010010;
            4'h6: hex_to_segs_al = 7'b0000010;
            4'h7: hex_to_segs_al = 7'b1111000;
            4'h8: hex_to_segs_al = 7'b0000000;
            4'h9: hex_to_segs_al = 7'b0010000;
            4'hA: hex_to_segs_al = 7'b0001000;
            4'hB: hex_to_segs_al = 7'b0000011;
            4'hC: hex_to_segs_al = 7'b1000110;
            4'hD: hex_to_segs_al = 7'b0100001;
            4'hE: hex_to_segs_al = 7'b0000110;
            4'hF: hex_to_segs_al = 7'b0001110;
        endcase
    endfunction

    // Drive the active digit (active-low an)
    always_comb begin
        // default: all digits off
        an  = 4'b1111;
        seg = 7'b1111111;  // off
        dp  = 1'b1;        // off

        case (digit)
            2'd0: an = 4'b1110; // rightmost
            2'd1: an = 4'b1101;
            2'd2: an = 4'b1011;
            2'd3: an = 4'b0111; // leftmost
        endcase

        seg = hex_to_segs_al(nibble);
        // leave dp off; if you want to control, invert dp_mask here:
        // dp = ~dp_mask; // active-low
    end
endmodule