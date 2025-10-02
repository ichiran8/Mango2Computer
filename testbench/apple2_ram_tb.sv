module apple2_ram_tb ();

    parameter PERIOD = 10;

    logic clk = 1, keyclk = 1, reset = 0;
    logic keyboard_data, color, hsync, vsync;
    logic [11:0] video;
    
    // always #(1us) clk++; //1 MHz
    // always #(80) clk++; //12.5 MHz
    always #(5) clk++; //100 MHz
    // always #(40) keyclk++; //25 MHz

endmodule