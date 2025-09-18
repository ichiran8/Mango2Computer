`timescale 1ns/1ps
module audio_tb;
  string test_name;
  int freq;
  logic clk = 0;
  logic nRST, speaker_out;
  logic [15:0] addr; 

  
  parameter PERIOD = 1us;

  always #(PERIOD/2) clk = ~clk;               // 1 MHz clock

  audio DUT(.CLK(clk),
            .nRST(nRST),
            .speaker_out(speaker_out),
            .addr(addr));

  initial begin
    nRST = 1'b0;
    addr = 16'h0;
    #(10 * PERIOD);
    nRST = 1'b1;
    freq = 0;
    //*****************************
    // Trigger the signal toggling
    //*****************************
    test_name = "Single Toggle";
    addr = 16'hC030;
    #(PERIOD);
    addr = 16'h0;
    #(PERIOD);

    //*****************************
    // Generate different frequencies
    //*****************************
    test_name = "Different Frequencies";
    //220 Hz
    freq = 220;
    repeat(44) begin
      addr = 16'hC030;
      #(PERIOD);
      addr = 16'h0;
      #(2271 * PERIOD);
    end

    //1 kHz
    freq = 1000;
    repeat(200) begin
      addr = 16'hC030;
      #(PERIOD);
      addr = 16'h0;
      #(499 * PERIOD);
    end

    freq = 20000;
    repeat(4000) begin
      addr = 16'hC030;
      #(PERIOD);
      addr = 16'h0;
      #(24 * PERIOD);
    end

    $finish;

  end

endmodule