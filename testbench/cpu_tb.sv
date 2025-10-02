`timescale 1ns/1ns
module cpu_tb;
  reg clk = 0;
  reg reset = 1;
  wire [15:0] AB;
  wire [7:0]  DO;
  reg  [7:0]  DI;
  wire WE;

  reg [7:0] mem [0:65535];

  cpu uut (
    .clk(clk), .reset(reset),
    .AB(AB), .DI(DI), .DO(DO), .WE(WE)
  );

  always #5 clk = ~clk;               // 100 MHz clock
  always @(*) DI = mem[AB];
  always @(posedge clk) if (WE) mem[AB] <= DO;

  initial begin
    $readmemh("test.hex", mem);
    mem[16'hFFFC] = 8'h00;            // reset vector -> $0400
    mem[16'hFFFD] = 8'h04;
    #50 reset = 0;
    #10000000 $finish;
  end
endmodule


// module ram (
//     input logic clk, rst,
//     input logic [31:0] data_address, // alu result to be read or written
//     input logic [31:0] instruction_address, // no brainer, it is the insturction address
//     input logic dm_read_en, dm_write_en, // enable ports for the read and enable
//     input logic [31:0] data_to_write, // data to be written into memory
//     output logic [31:0] instruction_read, data_read, // things we got from memory dude
//     output logic pc_enable
// );

// logic [31:0] memory [4095:0];

// initial begin
//         $readmemh("cpu.mem", memory);
// end


// typedef enum logic {IDLE, WAIT} StateType;

// StateType state, next_state;


// always_ff @(posedge clk, posedge rst) begin

//   if (rst) begin

//     state <= IDLE;

//   end else begin
//     state <= next_state;
//   end

// end


// // assign data_out = memory[address_DM];

// // assign instr_out = memory[address_IM];


// always_comb begin

//   pc_enable = 1'b1;

//   next_state = state;

//   case (state)

//   IDLE: begin

//     if (dm_read_en | dm_write_en) begin

//       pc_enable = 1'b0;

//       next_state = WAIT;

//     end

//   end

//   WAIT: begin

//      pc_enable = 1'b1;

//     next_state = IDLE;

//   end

//   endcase

// end

// always @(negedge clk) begin
//     if (dm_write_en) begin
//         memory[{4'b0, data_address[7:0]}] <= data_to_write;
//     end
//     data_read <= memory[{4'b0, data_address[7:0]}];
//     instruction_read <= memory[{4'b0, instruction_address[9:2]}];
// end
