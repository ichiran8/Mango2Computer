module display (
    input logic clk, n_rst,
    output logic hsync,
    output logic vsync,
    output logic [3:0] r, g, b,
    output logic [1:0] led
);

// module clk_wiz_0 
//  (
//   // Clock out ports
//   output        clk_out1,
//   output        clk_out2,
//   // Status and control signals
//   input         reset,
//   output        locked,
//  // Clock in ports
//   input         clk_in1
//  );
    logic clk14, clk25MHz, cpuclk;
    clk_wiz_0 test(.clk_in1(clk), .reset(1'b0), .clk_out1(clk14), .clk_out2(clk25MHz));
    
    clock_divider CLOCK (.CLK(clk14), .nRST(~n_rst), .enable(1'b1), .CPUCLK(cpuclk));
    vga_controller VGA(.clk(clk25MHz), .n_rst(~n_rst), .hsync(hsync), .vsync(vsync), .r(r), .g(g), .b(b));

    assign led[0] = clk25MHz;
    assign led[1] = clk14;
endmodule