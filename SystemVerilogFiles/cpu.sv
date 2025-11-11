
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

// note difference in opcode for each memory address

// DECODE OPCODE --> DECODE BASED ON INSTR TYPE --> TRANSITION BASED ON ADDRESSING TYPE
always_comb begin
  address_bus = '0;
  branch_type = instr_reg[4:0] == 5'h10 ? instr_reg[7:5] : '0;
  transfer = 1'b0;
  next_state = state_temp;
  reg_write = 1'b0;
  dest_reg = 3'h0;
  pc_inc = 1'b1;
  jsr = instr_reg == 8'b0010_0000;
  jump = instr_reg ==? 8'b01?0_1100;
  adtl_abs = (instr_reg ==? 8'b0??1_0110) || (instr_reg ==? 8'b11?1_0110) || (instr_reg == 8'h0E);
  status_choice = (instr_reg == 8'b0001_1000) ? 2'h3 : 
                  (instr_reg == 8'b0101_1000) ? 2'h2 :
                  (instr_reg == 8'b1011_1000) ? 2'h1 : 2'h0;
  mem_read = 1'b0;
  write_enable = 1'b0;
  branch = 1'b0;
  alu_latch = 1'b1;
  casez(instr_reg)
    8'b???0_0001, 8'b10?1_0100, 8'b???1_?101, 8'b0??1_?110, 8'b11?1_?110, 1011_1100 : indexed = 2'h2;
    8'b???1_?001, 8'b10?1_0110, 8'b1011_1110 : indexed = 2'h1;
    default: indexed = 2'h0;
  endcase
  //ALU OP since this can't be dependent on IDLE
  casez (instr_reg)
    8'b100?_??00                                           : aluop = ALU_OR;
    8'b11?0_0?00, 8'b11?0_1100, 8'b11??_??01               : aluop = ALU_SUB;
    8'b001?_??01, 8'b0010_?100                             : aluop = ALU_AND;
    8'b010?_??01                                           : aluop = ALU_XOR;
    8'b011?_??01                                           : aluop = ALU_ADD;
    8'b000?_?110, 8'b0000_1010                             : aluop = ALU_SLA;
    8'b001?_?110, 8'b0010_1010                             : aluop = ALU_ROL;
    8'b010?_?110, 8'b0100_1010                             : aluop = ALU_SRL;
    8'b011?_?110, 8'b0110_1010                             : aluop = ALU_ROR;
    8'b110?_?110, 8'b1000_1000, 8'b1100_1010, 8'b0?00_1000 : aluop = ALU_DEC;
    8'b111?_?110, 8'b11?0_1000, 8'b0?10_1000               : aluop = ALU_INC;
  endcase
  casez(state_temp)
    IDLE : begin
      mem_read = 1'b1;
      address_bus = pc;
        casez(instr_reg)
          8'b0000_0000 : begin
            next_state = IMPLIED_BREAK_FETCH; // break instruction ; 7 cycles
          end

          8'b000?_??01 : begin // all OR instructions
              casez(instr_reg[4:2])
              // 010 is OR IMMEDIATE 2 
              // 001 is OR ZERO PAGE 1 
              // 101 is OR ZERO PAGE X 5 
              // 011 is OR ABSOLUTE 3
              // 111 is OR BSOLUTE X 7 
              // 110 is OR ABSOLUTE Y 6 
              // 000 is OR ZERO PAGE INDIRECT X 0 
              // 100 is OR ZERO PAGE INDIRECT Y 4 
                  3'd0 : next_state = INDIRECT_FETCH_LOWER; // fetch tghe lower immediate addr
                  3'd1 : next_state = ZERO_PAGE_FETCH_PAGE;
                  3'd2 : next_state = IMMEDIATE_FETCH;
                  3'd3 : next_state = ABSOLUTE_FETCH_LOWER;
                  3'd4 : next_state = ZERO_PAGE_Y_FETCH_IMM;
                  3'd5 : next_state = ZERO_PAGE_X_FETCH_IMM;
                  3'd6 : next_state = ABSOLUTE_FETCH_LOWER;
                  3'd7 : next_state = ABSOLUTE_FETCH_LOWER;
              endcase
          end

          8'b???1_0000 : begin 
              next_state = BRANCH_CHECK; // all branch instructions ; increment every odd number
          end

          8'b0010_0000 : begin
              next_state = ABSOLUTE_FETCH_LOWER; // jump subroutine instruction
          end

          8'b01?0_0000 : begin
            next_state = instr_reg[5] ? IMPLIED_RTS_READ : IMPLIED_RTI_INCREMENT; // return subroutine and return from interrupt
            pc_inc = instr_reg[5];
          end

          8'b101?_??01 : begin // all load A instructions (depending on addressing mode)
          // LOAD ACCUMULATOR 
          // imm - 010
          // zero page - 001 
          // zero page X - 101
          // absolute
          // absolute x
          // absolute y
          // indirect x
          // indirect y
              casez(instr_reg[4:2])
                  3'd0 : next_state = INDIRECT_FETCH_LOWER; // fetch tghe lower immediate addr
                  3'd1 : next_state = ZERO_PAGE_FETCH_PAGE;
                  3'd2 : next_state = IMMEDIATE_FETCH;
                  3'd3 : next_state = ABSOLUTE_FETCH_LOWER;
                  3'd4 : next_state = ZERO_PAGE_Y_FETCH_IMM;
                  3'd5 : next_state = ZERO_PAGE_X_FETCH_IMM;
                  3'd6 : next_state = ABSOLUTE_FETCH_LOWER;
                  3'd7 : next_state = ABSOLUTE_FETCH_LOWER;
              endcase
          end

          // LOAD Y
          8'b1010_0000, 8'b101?_?100 : begin // all load Y instructions (depending on addressing mode)
              if(instr_reg[2]) begin
                  next_state = IMMEDIATE_FETCH;
              end else begin
                  casez(instr_reg[4:3])
                      2'd0 : next_state = ZERO_PAGE_FETCH_PAGE;
                      2'd1 : next_state = ABSOLUTE_FETCH_LOWER;
                      2'd2 : next_state = ZERO_PAGE_X_FETCH_IMM;
                      2'd3 : next_state = ABSOLUTE_FETCH_LOWER;
                  endcase
              end
          end

          // LOAD X
          8'b1010_0010, 8'b101?_?110 : begin
              if(instr_reg[3]) begin
                  next_state = IMMEDIATE_FETCH;
              end else begin
                  casez(instr_reg[4:3]) 
                      2'd0 : next_state = ZERO_PAGE_FETCH_PAGE; 
                      2'd1 : next_state = ABSOLUTE_FETCH_LOWER;
                      2'd2 : next_state = ZERO_PAGE_X_FETCH_IMM;
                      2'd3 : next_state = ABSOLUTE_FETCH_LOWER;
                  endcase
              end
          end

          8'b01?0_1100 : begin
              next_state = instr_reg[5] ? INDIRECT_FETCH_LOWER : ABSOLUTE_FETCH_LOWER;                        
          end

          8'b1100_0?00, 8'b1100_1100 : begin // Compare Y with register
              casez(instr_reg[3:2])
                  2'b00 : next_state = IMMEDIATE_FETCH;//Immediate (2 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //Zeropage (3 cycles)
                  // 2'b10 : //NOP
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER;//Absolute (4 cycles)
                  default :;
              endcase
          end

          8'b1110_0?00, 8'b1110_1100 : begin // Compare X with register
              casez(instr_reg[3:2])
                  2'b00 : next_state = IMMEDIATE_FETCH; //Immediate (2 cycles)
                  2'b01 : next_state =ZERO_PAGE_FETCH_PAGE; //Zeropage (3 cycles)
                  //1'b10 : //NOP
                  2'b11 : next_state =  ABSOLUTE_FETCH_LOWER; //Absolute (4 cycles)
                  default :;
              endcase
          end
          
          // AND
          8'b001?_??01 : begin
              if(instr_reg[4]) begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect Y (takes 6/7 cycles depending on pages)
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; //zeropage X (takes 4 cycles)
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; //absolute Y (takes 5/6 cycles depending on pages)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute X (takes 5/6 cycles depending on pages)
                  endcase
              end else begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect X (takes 6 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                  2'b10 : next_state = IMMEDIATE_FETCH;//immediate (takes 2 cycles)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end
          end

          // XOR 
          8'b010?_??01 : begin
              if(instr_reg[4]) begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect Y (takes 6/7 cycles depending on pages)
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; //zeropage X (takes 4 cycles)
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; //absolute Y (takes 5/6 cycles depending on pages)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute X (takes 5/6 cycles depending on pages)
                  endcase
              end else begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect X (takes 6 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                  2'b10 : next_state = IMMEDIATE_FETCH;//immediate (takes 2 cycles)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end

          end

          // ADD 
          8'b011?_??01 : begin
              if(instr_reg[4]) begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect Y (takes 6/7 cycles depending on pages)
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; //zeropage X (takes 4 cycles)
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; //absolute Y (takes 5/6 cycles depending on pages)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute X (takes 5/6 cycles depending on pages)
                  endcase
              end else begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER;//indirect X (takes 6 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                  2'b10 : next_state = IMMEDIATE_FETCH;//immediate (takes 2 cycles)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end
          end

          // STORE_ACC
          8'b100?_0?01, 8'b1001_1001, 8'b100?_1101 : begin
              if(instr_reg[3]) begin
                      next_state = ABSOLUTE_FETCH_LOWER; 
              end else begin
                  casez({instr_reg[4], instr_reg[2]})
                      2'd0 : next_state = INDIRECT_FETCH_LOWER; // x
                      2'd1 : next_state = ZERO_PAGE_FETCH_PAGE; // zero page
                      2'd2 : next_state = INDIRECT_FETCH_LOWER; // y
                      2'd3 : next_state = ZERO_PAGE_X_FETCH_IMM; // zero page x
                  endcase
              end
          end


          8'b110?_??01 : begin //next_state = COMPARE_A;
              if(instr_reg[4]) begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER; //indirect Y (takes 6/7 cycles depending on pages)
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; //zeropage X (takes 4 cycles)
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; //absolute Y (takes 5/6 cycles depending on pages)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute X (takes 5/6 cycles depending on pages)
                  endcase
              end else begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER; //indirect X (takes 6 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                  2'b10 : next_state = IMMEDIATE_FETCH; //immediate (takes 2 cycles)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end
          end

          // SUB
          8'b111?_??01 : begin
              if(instr_reg[4]) begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER; //indirect Y (takes 6/7 cycles depending on pages)
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; //zeropage X (takes 4 cycles)
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; //absolute Y (takes 5/6 cycles depending on pages)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute X (takes 5/6 cycles depending on pages)
                  endcase
              end else begin
                  casez(instr_reg[3:2])
                  2'b00 : next_state = INDIRECT_FETCH_LOWER; //indirect X (takes 6 cycles)
                  2'b01 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                  2'b10 : next_state = IMMEDIATE_FETCH; //immediate (takes 2 cycles)
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end
          end

          // BIT TEST
          8'b0010_?100 : begin
              next_state = instr_reg[3] ? ABSOLUTE_FETCH_LOWER : ZERO_PAGE_FETCH_PAGE;
          end
          
          // STORE Y
          8'b100?_0100, 8'b1000_1100 : begin
              if(!instr_reg[4]) begin
                  case(instr_reg[3])
                      1'b0 : next_state = ZERO_PAGE_FETCH_PAGE; //zeropage (takes 3 cycles)
                      1'b1 : next_state = ABSOLUTE_FETCH_LOWER; //absolute (takes 4 cycles)
                  endcase
              end else begin
                  //Zeropage X (takes 4 cycles)
                  next_state = ZERO_PAGE_X_FETCH_IMM;
              end
          end


          8'b000?_?110, 8'b0000_1010 :  begin // shift left arithmetic 
              if(instr_reg[2]) begin
                  casez(instr_reg[4:3])
                      2'd0 : next_state = ZERO_PAGE_FETCH_PAGE;
                      2'd1 : next_state = ABSOLUTE_FETCH_LOWER; // just normal
                      2'd2 : next_state = ZERO_PAGE_X_FETCH_IMM;
                      2'd3 : next_state = ABSOLUTE_FETCH_LOWER; // x indexed
                  endcase
              end else begin
                  next_state = IMPLIED_EXECUTE;
                  pc_inc = 0;
              end
          end

          8'b001?_?110, 8'b0010_1010 : begin//ROTATE_LEFT; // shift carry out into LSB
              if(instr_reg[2]) begin
                  casez(instr_reg[4:3]) 
                      2'b00 : next_state = ZERO_PAGE_FETCH_PAGE;// rotate left zero page ; should take 5 cycles
                      2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM;// rotate left zero page x indexed ; should take 6 cycles
                      2'b10 : next_state = ABSOLUTE_FETCH_LOWER;// rotate left absolute ; should take 6 cycles
                      2'b11 : next_state = ABSOLUTE_FETCH_LOWER; // rotate left absolute x indexed ; should take 7 cycles
                  endcase
              end else begin
                  next_state = IMMEDIATE_FETCH;
                  // rotate left accumulator ; should take 2 cycles
              end
          end

          8'b010?_?110, 8'b0100_1010 : begin//next_state = SRL; // shift right logical
              if(instr_reg[2]) begin
                  casez(instr_reg[4:3]) 
                      2'b00 : next_state = ZERO_PAGE_FETCH_PAGE; // logical right shift zero page ; should take 5 cycles
                      2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; // logical right shift zero page x indexed ; should take 6 cycles
                      2'b10 : next_state = ABSOLUTE_FETCH_LOWER;// logica right shift absolute ; should take 6 cyclces
                      2'b11 : next_state = ABSOLUTE_FETCH_LOWER; // logical right shift absolute x indefxed ; should take 7 cycles
                  endcase
              end else begin
                  // logical right shift accumulator ; should take 2 cycles only
                  next_state = IMMEDIATE_FETCH;
              end
          end

          8'b011?_?110, 8'b0110_1010 : begin//next_state = ROTATE_RIGHT;
          if(instr_reg[2]) begin
                  casez(instr_reg[4:3]) 
                      2'b00 : next_state = ZERO_PAGE_FETCH_PAGE; // logical right shift zero page ; should take 5 cycles
                      2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; // logical right shift zero page x indexed ; should take 6 cycles
                      2'b10 : next_state = ABSOLUTE_FETCH_LOWER; // logica right shift absolute ; should take 6 cyclces
                      2'b11 : next_state = ABSOLUTE_FETCH_LOWER; // logical right shift absolute x indefxed ; should take 7 cycles
                  endcase
              end else begin
                  // logical right shift accumulator ; should take 2 cycles only
                  next_state = ABSOLUTE_FETCH_LOWER;
              end
          end

          8'b100?_0110, 8'b1000_1110 : begin
              //next_state = STORE_X;
              if(instr_reg[3]) begin
                  next_state = ABSOLUTE_FETCH_LOWER; // store X absolute
              end else begin
                  next_state = instr_reg[4] ? ZERO_PAGE_Y_FETCH_IMM : ZERO_PAGE_FETCH_PAGE;
              end
          end

          8'b110?_?110 : begin//next_state = DECREMENT_ACC;
              // there are page crossing penalties 
              casez(instr_reg[4:3])
                  2'b00 : next_state = ZERO_PAGE_FETCH_PAGE; // decrement zero page ; should take 5 cycles
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; // decrement zero page x indexed ; should take 6 cycles
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; // decrement absolute ; should take 6 cycles
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; // decrement absolute x indexed ; should take 7 cycles
              endcase
          end

          8'b111?_?110 : begin //next_state = INCREMENT_ACC;
              casez(instr_reg[4:3])
                  2'b00 : next_state = ZERO_PAGE_FETCH_PAGE; // decrement zero page ; should take 5 cycles
                  2'b01 : next_state = ZERO_PAGE_X_FETCH_IMM; // decrement zero page x indexed ; should take 6 cycles
                  2'b10 : next_state = ABSOLUTE_FETCH_LOWER; // decrement absolute ; should take 6 cycles
                  2'b11 : next_state = ABSOLUTE_FETCH_LOWER; // decrement absolute x indexed ; should take 7 cycles
              endcase
          end


          8'b0?00_1000 : begin//next_state = PUSH;
          // each of these take 3 cycles
          // so the only thing we need to differentiate is the register that is being pushed
            pc_inc = 0;
            next_state = IMPLIED_PUSH_DECREMENT;
          end

          8'b0?10_1000 : //next_state = PULL;
          // These each take 4 cycles
            next_state = IMPLIED_PULL_READ;

          8'b0?01_1000, 8'b1?01_1000 : begin
            next_state = IMPLIED_EXECUTE;
            pc_inc = 0;
          end

          8'b?111_1000, 8'b0011_1000 : begin
            next_state = IMPLIED_EXECUTE;
            pc_inc = 0;
          end

          8'b1000_1000, 8'b1100_1010 : begin//next_state = DECREMENT; // Y and X
            next_state = IMPLIED_EXECUTE;
            pc_inc = 0;
          end

          8'b11?0_1000 : begin//next_state = INCREMENT; // Y and X
          // each are 2 cycles
            next_state = IMPLIED_EXECUTE;
            pc_inc = 0;
          end

          8'b1010_10?0 : //next_state = TRANSFER_ACC; // transfer accumulator to register
            next_state = IMPLIED_EXECUTE;

          8'b100?_1010 : //next_state = TRANSFER_X; // transfer x to register
            next_state = IMPLIED_EXECUTE;

          8'b1001_1000 : begin 
            next_state = IMPLIED_EXECUTE; //next_state = next_state = IMMEDIATE_FETCH; //TRANSFER_Y;
            pc_inc = 0;
          end
          //each are 2 cycles

          8'b1011_1010 : begin
            next_state = IMPLIED_EXECUTE;//next_state = TRANSFER_STACK;
            pc_inc = 0;
          end
          // each are 2 cycles
          
          default : begin
            pc_inc = 0; 
            next_state = IMPLIED_EXECUTE; //NOP;
          end
        endcase
      end
      BRANCH_CHECK: begin
          next_state = branch ? BRANCH_CHECK_PAGE : IDLE;
          mem_read = 1'b1;
          branch = (branch_type == 4'h8) ? ~neg   :
                  (branch_type == 4'h9) ? neg    :
                  (branch_type == 4'hA) ? ~ovf   :
                  (branch_type == 4'hB) ? ovf    :
                  (branch_type == 4'hC) ? ~carry :
                  (branch_type == 4'hD) ? carry  :
                  (branch_type == 4'hE) ? ~zero  : zero;
          src_reg = 3'h5;
      end
      BRANCH_CHECK_PAGE: begin
          next_state = ~carry ? IDLE : BRANCH_PAGE_FAULT;
          src_reg = 3'h6;
          dest_reg = 3'h5;
          reg_write = 1'b1;
      end
      BRANCH_PAGE_FAULT: begin
        next_state = IDLE;
        dest_reg = 3'h6;
        reg_write = 1'b1;
      end
      IMMEDIATE_FETCH: begin
          next_state = IDLE;
          mem_read = 1'b1;
          alu_latch = 1'b0;
      end
      ABSOLUTE_FETCH_LOWER: begin
          next_state = ABSOLUTE_FETCH_UPPER;
          mem_read = 1'b1;
          dest_reg = 3'h5;
          reg_write = 1'b1;
      end 
      ABSOLUTE_FETCH_UPPER: begin
          mem_read = 1'b1;
          dest_reg = 3'h6;
          reg_write = 1'b1;
          if(jsr == 0 ) begin
                              next_state = ABSOLUTE_VALUE_FETCH;
          end else if(indexed[1] | indexed[0]) begin
                              next_state = ABSOLUTE_INDEXED_ADDR;
          end else begin
                              next_state = ABSOLUTE_FETCH_ADDR2;
          end
      end
      ABSOLUTE_VALUE_FETCH:   begin
        mem_read = 1'b1;
        next_state = IDLE;
      end
      ABSOLUTE_INDEXED_ADDR:  next_state = carry ? ABSOLUTE_PAGE_FAULT : IDLE;
      ABSOLUTE_FETCH_ADDR2:   begin
        next_state = ABSOLUTE_WRITE1;
        mem_read = 1'b1;
      end
      ABSOLUTE_WRITE1:        begin
        next_state = ABSOLUTE_WRITE2;
        write_enable = 1'b1;
      end
      ABSOLUTE_WRITE2:        begin
        next_state = IDLE;
        write_enable = 1'b1;
      end
      ZERO_PAGE_FETCH_PAGE: begin
          mem_read = 1'b1;
          if(indexed[1] | indexed[0]) begin
                              next_state = ZERO_PAGE_INDEXED;
          end else begin
                              next_state = ZERO_PAGE_READ;
          end
      end
      ZERO_PAGE_INDEXED: next_state = ZERO_PAGE_READ;
      ZERO_PAGE_READ: begin
          mem_read = 1'b1;
          next_state = IDLE;
      end
      ZERO_PAGE_Y_FETCH_IMM: begin
          next_state = ZERO_PAGE_Y_LOAD1;
          mem_read = 1'b1;
      end
      ZERO_PAGE_Y_LOAD1: begin
          mem_read = 1'b1;
          next_state = ZERO_PAGE_Y_LOAD2;
      end
      ZERO_PAGE_Y_LOAD2: begin
          mem_read = 1'b1;
          if(carry) begin
              next_state = ZERO_PAGE_Y_LOAD;
          end else begin
              next_state = ZERO_PAGE_FAULT;
          end
      end
      ZERO_PAGE_FAULT: next_state = ZERO_PAGE_Y_LOAD;
      ZERO_PAGE_Y_LOAD: begin
          next_state = IDLE;
          mem_read = 1'b1;
      end
      ZERO_PAGE_X_FETCH_IMM: begin
          next_state = ZERO_PAGE_X_LOAD1;
          mem_read = 1'b1;
      end
      ZERO_PAGE_X_LOAD1: begin
        next_state = ZERO_PAGE_X_LOAD2;
        mem_read = 1'b1;
      end
      ZERO_PAGE_X_LOAD2: begin
      mem_read = 1'b1;
      if(carry) begin
              next_state = ZERO_PAGE_X_LOAD;
          end else begin
              next_state = ZERO_PAGE_X_FAULT;
          end
      end
      ZERO_PAGE_X_FAULT: begin 
        next_state = ZERO_PAGE_X_LOAD;
      end
      ZERO_PAGE_X_LOAD: begin
        mem_read = 1'b1;
        next_state = IDLE;
      end
      INDIRECT_FETCH_LOWER: begin
        mem_read = 1'b1;
        next_state = INDIRECT_FETCH_UPPER;
      end
      INDIRECT_FETCH_UPPER: begin 
        next_state = INDIRECT_LOAD_IMM1;
        mem_read = 1'b1;
      end
      INDIRECT_LOAD_IMM1: begin
        mem_read = 1'b1;
        next_state = INDIRECT_LOAD_IMM2;
      end 
      INDIRECT_LOAD_IMM2: begin
        mem_read = 1'b1;
        next_state = IDLE;
      end 

      IMPLIED_EXECUTE: begin 
        next_state = IDLE;
      end

      IMPLIED_PUSH_DECREMENT: begin 
        next_state = IMPLIED_PUSH_STACK;
        write_enable = 1'b1;
        src_reg = 3'h3;
        dest_reg = 3'h4;
        aluop = ALU_DEC;
        alu_latch = 1'b0;
      end
      IMPLIED_PUSH_STACK: begin 
        next_state = IDLE;
        write_enable = 1'b1;
        src_reg = 3'h4;
        dest_reg = 3'h4;
        aluop = ALU_DEC;
        alu_latch = 1'b0;
      end
      IMPLIED_PULL_READ: begin 
        next_state = IMPLIED_INCREMENT;
        mem_read = 1'b1;
        dest_reg = 3'h3;
      end
      IMPLIED_INCREMENT: begin
        next_state = IMPLIED_PULL_STACK;
        dest_reg = 3'h4;
        aluop = ALU_DEC;
        src_reg = 3'h4;
      end
      IMPLIED_PULL_STACK: begin 
        next_state = IDLE;
        mem_read = 1'b1;
        dest_reg = 3'h4;
      end

      IMPLIED_RTS_READ: begin
        next_state = IMPLIED_RTS_INCREMENT;
        mem_read = 1'b1;
      end
      IMPLIED_RTS_INCREMENT: begin
        next_state = IMPLIED_RTS_PULL_LOWER;
        src_reg = 3'h4;
        dest_reg = 3'h4;
        aluop = ALU_DEC;
      end
      IMPLIED_RTS_PULL_LOWER: begin
        next_state = IMPLIED_RTS_PULL_UPPER;
        mem_read = 1'b1;
      end
      IMPLIED_RTS_PULL_UPPER: begin
        next_state = IMPLIED_RTS_ADD_PC;
        mem_read = 1'b1;
      end
      IMPLIED_RTS_ADD_PC: begin
        next_state = IDLE;
        src_reg = 3'h6;
        dest_reg = 3'h6;
      end

      IMPLIED_RTI_INCREMENT: begin
        src_reg = 3'h4;
        next_state = IMPLIED_RTI_STATUS;
        dest_reg = 3'h4;
        aluop = ALU_DEC;
      end
      IMPLIED_RTI_STATUS: begin
        next_state = IMPLIED_RTI_PULL_LOWER;
        dest_reg = 3'h3;
        write_enable = 1'b1;
      end
      IMPLIED_RTI_PULL_LOWER: begin 
        next_state = IMPLIED_RTI_PULL_UPPER;
        mem_read = 1'b1;
        dest_reg = 3'h5;
      end
      IMPLIED_RTI_PULL_UPPER: begin
        next_state = IMPLIED_RTI_STORE;
        mem_read = 1'b1;
        dest_reg = 3'h6;
      end
      IMPLIED_RTI_STORE: begin
        next_state = IDLE;
        write_enable = 1'b1;
        src_reg = 3'h4;
      end

      IMPLIED_BREAK_FETCH: begin
        next_state = IMPLIED_BREAK_PUSH_UPPER;
        mem_read = 1'b1;
      end
      IMPLIED_BREAK_PUSH_UPPER: begin
        next_state = IMPLIED_BREAK_PUSH_LOWER;
        aluop = ALU_DEC;
        src_reg = 3'h4;
        dest_reg = 3'h4;
        write_enable = 1'b1;
      end
      IMPLIED_BREAK_PUSH_LOWER: begin
        next_state = IMPLIED_BREAK_PUSH_P;
        aluop = ALU_DEC;
        src_reg = 3'h4;
        dest_reg = 3'h4;
        write_enable = 1'b1;
      end
      IMPLIED_BREAK_PUSH_P: begin 
        next_state = IMPLIED_BREAK_READ_LOWER;
        aluop = ALU_DEC;
        src_reg = 3'h4;
        dest_reg = 3'h4;
        write_enable = 1'b1;
      end
      IMPLIED_BREAK_READ_LOWER: begin 
        next_state = IMPLIED_BREAK_READ_HIGHER;
        mem_read = 1'b1;
        dest_reg = 3'h5;
      end
      IMPLIED_BREAK_READ_HIGHER: begin
        next_state = IDLE;
        mem_read = 1'b1;
        dest_reg = 3'h6;
      end

      endcase
    end
  endmodule