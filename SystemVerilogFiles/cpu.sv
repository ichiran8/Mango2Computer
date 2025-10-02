
module top_level (

    input logic clk, reset,

    input logic [7:0] data_in,

    input logic IRQ, NMI, ready,

    output logic [15:0] address_bus,

    output logic [7:0] data_out,

    output logic write_enable,

    output logic[5:0] state

);

  import cpu_types_pkg::*;
 
  //Registers

  logic [15:0]pc, pc_temp;

  logic [7:0]sp, A, X, Y, instr_reg, rda, rdb;

  logic [7:0] next_sp, next_A, next_X, next_Y, next_rda, next_rdb;

  logic carry, zero, IRQ_dis, dec, brk, ovf, neg;

  logic next_carry, next_zero, next_irq_dis, next_dec, next_brk, next_ovf, next_neg;
 
  logic jump, branch, adtl_abs, jsr, mem_read, reg_write, pc_inc, transfer, alu_latch;

  logic [1:0]indexed, status_choice;

  logic [2:0]dest_reg, src_reg; //0: Acc, 1: Y, 2: X, 3:Status, 4:SP, 5:LPC, 6:UPC

  logic [3:0] branch_type;

  aluop_t aluop;

  state_t state_temp, next_state;

  assign state = 6'(state_temp);

always_ff @(posedge CLK, negedge reset) begin

    if(!reset) begin
        pc        <= 'h0;
        sp        <= 'h0;
        A         <= 'h0;
        X         <= 'h0;
        Y         <= 'h0;
        carry     <= 'h0;
        zero      <= 'h0;
        IRQ_dis   <= 'h0;
        dec       <= 'h0;
        brk       <= 'h0;
        ovf       <= 'h0;
        neg       <= 'h0;
        instr_reg <= 'h0;
        rda       <= 'h0;
        rdb       <= 'h0;
    end else begin
        if(ready) begin
            sp      <= next_sp;
            A       <= next_A;
            X       <= next_X;
            Y       <= next_Y;
            carry   <= next_carry;
            zero    <= next_zero;
            IRQ_dis <= next_irq_dis;
            dec     <= next_dec;
            brk     <= next_brk;
            ovf     <= next_ovf;
            neg     <= next_neg;
            pc      <= pc_temp + pc_inc; // set PC_INC to 1 during final state ; PC_TEMP will be PC in most cases 
            rda     <= next_rda;
            rdb     <= next_rdb;
            if(state_temp == IDLE)
              instr_reg <= data_in;
        end 
    end
end

// filter by operation --> then filter by register access --> filter by addressing mode


always_ff @(posedge CLK, negedge reset) begin
  if (!reset) begin
      state_temp <= IDLE;
  end else begin
      state_temp <= next_state;
  end
end

  endmodule