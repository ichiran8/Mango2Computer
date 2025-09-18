
//`include "r10k_pkg.vh"
//`include "cpu_types_pkg.vh"

`timescale 1 ns / 1 ns
module clock_divider_tb ();
    //import cpu_types_pkg::*;
    //import r10k_pkg::*;

    parameter PERIOD = 10;


  // signals
    //logic CLK = 1, nRST = 0, enable = 1;

    logic CLK = 1, nRST = 0;
    logic enable, CPUCLK;
  
  // clock
  always #(PERIOD/2) CLK++; 

  clock_divider fc(.CLK(CLK), .nRST(nRST), .enable(enable), .CPUCLK(CPUCLK));

  task reset_dut;
  begin
    nRST = 1'b1;
    @(posedge CLK);
	@(posedge CLK);
	nRST = 1'b0;
    @(posedge CLK);
    @(posedge CLK);
	@(negedge CLK);

	nRST = 1;
	
	@(posedge CLK);
	@(posedge CLK);
  end

endtask

// task push;
//     input logic [7:0] data, addr; 
//     begin
//         @(negedge CLK);
//          = 1;
//         write_data = data; 
//         write_addr = addr;
//          @(negedge CLK);
//         enqueue = 0;

//     end
//endtask


  integer i, j, k;
  initial begin
    enable = 1'b0;

    reset_dut;
    //pop;
    enable = 1'b1;
    @(posedge CLK);
    for(j = 0; j < 100; j++) begin
       // enable = 1'b1;
        @(posedge CLK);
    end
    for(i = 0; i < 100; i++)begin
      @(posedge CLK);
    end
    // cam(5);
    // cam(1);
    // edge_case(2, 9);
    // //for(k = 0; k <)
    
    $stop();
  end
    

endmodule