`timescale 1ns/1ns

module temp_top_tb ();

    parameter PERIOD = 10;

    logic clk = 1, reset = 0;
    logic write_enable;
    logic [7:0] data_in, data_out;
    logic [15:0] addr;


    // always #(PERIOD/(2*14.3181818e6)) clk++; 
    
    // always #(PERIOD/2) clk++;
    always #(34.9) clk++;
    // input wire clk,
    // input wire cs,
    // input wire [ADDR_WIDTH-1:0] addr, 
    // input wire we,
    // input wire [DATA_WIDTH-1:0] data_in,
    // output reg [DATA_WIDTH-1:0] data_out  
    // ram testmem(.clk(clk), .reset(reset), .cs(1'b1), .we(write_enable), .addr(addr), .data_in(data_in), .data_out(data_out));
    // cpu65xx #(
    //     .pipelineOpcode(1'b0),
    //     .pipelineAluMux(1'b0),
    //     .pipelineAluOut(1'b0)
    //     ) cpu(.clk(cpu_clk), .enable(1'b1), .reset(reset), .nmi_n(1'b1), .irq_n(1'b1), .data_in(cpu_data_in), .data_out(cpu_data_out), .addr(addr), .we(write_enable), .debugPc(),
    //     .debugOpcode(), .so_n(), .debugA(), .debugX(), .debugY(), .debugS());
    temp_top dut(.clk(clk), .reset(reset));
    task reset_dut;
        begin
            reset = 1'b0;
            @(posedge clk);
            @(posedge clk);
            @(posedge clk);
            reset = 1'b1;
            @(posedge clk);
            @(posedge clk);
            @(posedge clk);
            reset = 1'b0; // positive reset
            @(posedge clk);
            @(posedge clk);
            @(posedge clk);
        end

    endtask

    initial begin
        integer i;
        reset_dut();
        for (i = 0; i < 100000; i++) begin
            @(posedge clk);
        end
        // #(PERIOD * 100000);
        $finish();
    end

endmodule