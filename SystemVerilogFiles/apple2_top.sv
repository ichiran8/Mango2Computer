module apple2_top (
    // top level input logic
    input logic clk, reset, 
    
    // keyboard input signals
    input logic keyboard_clk, key_rst, 
    input logic keyboard_data,

    //audio output signal
    output logic speaker,

    // vga input signals
    input logic color,

    // VGA output signals
    output logic hsync, vsync,
    output logic [11:0] video,
    
     output logic [7:0] an,
     output logic [6:0] seg,
     output logic dp,
     output logic keyflag
);
    assign keyflag = oflag;
    // clock definitions
    logic VGACLK, CPUCLK,CLK12;

    // video signals
    logic [15:0] VGAADDR; // input VGA address to RAM
    logic [7:0] VGA_DOUT; // output video data that we need
    logic read_mem; // request to read the data from mem
    logic [7:0] video_data;

    // cpu data signals
    logic [15:0] CPUADDR;
    logic [7:0] cpu_data_in, cpu_data_out, CPU_RAM_DOUT, CPU_ROM_DOUT;
    logic write_enable, write_enable_int;

    // keyboard signals
    logic [7:0] decoded;
    logic oflag,locked;

    clk_wiz_0 test(
        .clk_in1(clk), 
        .reset(1'b0), 
        .clk_out1(VGACLK),
        .clk_out2(CPUCLK),
        .clk_out3(CLK12),
        .locked(locked)
    );

    vga_test VID (
        .clk_25mhz(VGACLK),
        .clk_12mhz(CLK12),
        .data_in(video_data), // white on black characters
        .vid_addr(VGAADDR),
        .read_mem(read_mem),
        .hsync(hsync),
        .vsync(vsync),
        .text(txt_enable),
        .mix(mix_enable),
        .page2(pg_enable),
        .hires(hires_enable),
        .color(color),
        .video(video)
    );

    assign video_data = (read_mem) ? VGA_DOUT : '0;

    apple2_ram testmem(
        .CPUCLK(clk2), 
        .VGACLK(VGACLK), 
        .CPUADDR(CPUADDR), 
        .VGAADDR(VGAADDR), 
        .CPU_WEN(write_enable),
        .DSTORE(cpu_data_out), 
        .CPU_DOUT(CPU_RAM_DOUT), 
        .VGA_DOUT(VGA_DOUT),
        .decoded(decoded),
        .oflag(oflag));

    assign write_enable = write_enable_int && (CPUADDR < 16'hD000); // preventing writing to ROM

    // TODO
    // leaving this here in case we need to swap cores out
    // not sure if arlet's core will work in this case
    logic clk2, cpuclk_out;
    flex_counter ct(.clkx(CPUCLK), .n_rst(!reset || locked), .count_enable(1'b1), .rollover_val(4'd5), .rollover_flag(clk2));
    // BUFG clk_1mhz (
    //     .I(clk2),
    //     .O(cpuclk_out)
    // );

	// port (
	// 	clk : in std_logic;
	// 	enable : in std_logic;
	// 	reset : in std_logic;
	// 	nmi_n : in std_logic;
	// 	irq_n : in std_logic;
	// 	so_n : in std_logic := '1';

	// 	data_in : in unsigned(7 downto 0);
	// 	data_out : out unsigned(7 downto 0);
	// 	addr : out unsigned(15 downto 0);
	// 	we : out std_logic;
		
	// 	debugOpcode : out unsigned(7 downto 0);
	// 	debugPc : out unsigned(15 downto 0);
	// 	debugA : out unsigned(7 downto 0);
	// 	debugX : out unsigned(7 downto 0);
	// 	debugY : out unsigned(7 downto 0);
	// 	debugS : out u
	//12.5875
    

    cpu comp(
        .clk(clk2), 
        .reset(reset||!locked), 
        .address_bus(CPUADDR), 
        .data_in(cpu_data_in), 
        .data_out(cpu_data_out), 
        .write_enable(write_enable_int), 
        .IRQ(1'b0), 
        .NMI(1'b0), // not sure if this needs to be 0 or 1; it is meant for vertical blanking nonmaskable interrupt that halts the cpu apparently?
        .RDY(1'b1), 
        .PC());
    
    logic [15:0] testaddr;
    always_ff @(posedge clk2) begin
        testaddr <= CPUADDR;
    end

    assign cpu_data_in = (testaddr == 16'hC000) ? {oflag, decoded[6:0]} : (testaddr == 16'hC010) ? decoded : CPU_RAM_DOUT;
    logic of, de, sp, txt1, txt2, mix1, mix2, pg1, pg2, hires1, hires2;

    logic txt_enable, mix_enable, hires_enable, pg_enable;

    logic latch_txt = 1'b0;
    logic latch_mix = 1'b0;
    logic latch_pg = 1'b0;
    logic latch_hires = 1'b1;

    assign an[7:4] = '0;
    always_ff @(posedge VGACLK) begin
        latch_txt <= txt_enable;
        latch_mix <= mix_enable;
        latch_pg <= pg_enable;
        latch_hires <= hires_enable;
    end

    assign of = (testaddr == 16'hC000);
    assign de = (testaddr == 16'hC010);
    assign sp = (testaddr == 16'hC040);

    assign txt1 = (testaddr == 16'hC050);
    assign txt2 = (testaddr == 16'hc051);

    assign txt_enable = (txt2) ? 1'b1 : (txt1) ? 1'b0 : latch_txt;
    assign mix1 = (testaddr == 16'hc052);
    assign mix2 = (testaddr == 16'hc053);


    assign mix_enable = (mix2) ? 1'b1 : (mix1) ? 1'b0 : latch_mix;

    assign pg1 = (testaddr == 16'hc054);
    assign pg2 = (testaddr == 16'hc055);

    
    assign pg_enable = (pg2) ? 1'b1 : (pg1) ? 1'b0 : latch_pg;

    assign hires1 = (testaddr == 16'hc056);
    assign hires2 = (testaddr == 16'hc057);

    // 1598 123
    assign hires_enable = 1'b1;//(hires2) ? 1'b1 : (hires1) ? 1'b0 : latch_hires;
  
    keyboard KEY (
        .clk(VGACLK),
        .nrst(!key_rst),
        .PS2Clk(keyboard_clk),
        .PS2Data(keyboard_data),
        .clr_strb(testaddr == 16'hC010),
        .an(an[3:0]),
        .seg(seg),
        .dp(dp),
        .shift(),
        .decoded_out(decoded),
        .oflag(oflag)
    );

    audio SOUND (
        .CLK(clk2),
        .nRST(!reset),
        .addr(testaddr),
        .speaker_out(speaker)
    );

endmodule

 
module flex_counter #(
    parameter NUM_CNT_BITS = 4
)(
    input logic clk, n_rst, clear, count_enable,
    input logic [NUM_CNT_BITS - 1:0] rollover_val,
    output logic [NUM_CNT_BITS - 1:0] count_out,
    output logic rollover_flag
);
 
logic [NUM_CNT_BITS - 1:0] count_next;
logic roll_check;
 
always_ff @(posedge clk, negedge n_rst) begin
    if(!n_rst) begin
        count_out <= 4'd1;
        rollover_flag <= 1'b1;
    end else begin
        count_out <= count_next;
        rollover_flag <= roll_check;
    end
end
 
always_comb begin
    count_next = count_out;
    if(clear) begin
        count_next = '0;
    end else if(count_enable) begin
        if(count_next == rollover_val) begin
            count_next = 1;
        end else begin
            count_next = count_out + 1'b1;
        end
    end
end
 
always_comb begin
    roll_check = rollover_flag;
    if(clear) begin
        roll_check = 1'b0;
    end else if(count_next == rollover_val) begin
        roll_check = !rollover_flag;
    end
end
 
endmodule
`include "cpu_types_pkg.vh"

module cpu (

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

  module audio (
  input logic CLK, nRST,
  input logic [15:0] addr,
  output logic speaker_out
);

  logic speaker_toggle, next_speaker;

  always_ff @(posedge CLK, negedge nRST)
  if(!nRST)
    speaker_out <= 1'b0;
  else
      speaker_out <= next_speaker;

  assign speaker_toggle = (addr == 16'hC030);
  assign next_speaker = (speaker_toggle) ? ~speaker_out : speaker_out;

endmodule

`timescale 1ns / 10ps

module character_rom (
    input logic clk,
    input logic [9:0] char_addr,
    output logic [6:0] char_data_out
);
    logic [6:0] rom [0:1023];

    initial begin
        rom[0] = 7'h1C;
        rom[1] = 7'h22;
        rom[2] = 7'h2A;
        rom[3] = 7'h3A;
        rom[4] = 7'h1A;
        rom[5] = 7'h02;
        rom[6] = 7'h3C;
        rom[7] = 7'h00;
        rom[8] = 7'h08;
        rom[9] = 7'h14;
        rom[10] = 7'h22;
        rom[11] = 7'h22;
        rom[12] = 7'h3E;
        rom[13] = 7'h22;
        rom[14] = 7'h22;
        rom[15] = 7'h00;
        rom[16] = 7'h1E;
        rom[17] = 7'h22;
        rom[18] = 7'h22;
        rom[19] = 7'h1E;
        rom[20] = 7'h22;
        rom[21] = 7'h22;
        rom[22] = 7'h1E;
        rom[23] = 7'h00;
        rom[24] = 7'h1C;
        rom[25] = 7'h22;
        rom[26] = 7'h02;
        rom[27] = 7'h02;
        rom[28] = 7'h02;
        rom[29] = 7'h22;
        rom[30] = 7'h1C;
        rom[31] = 7'h00;
        rom[32] = 7'h1E;
        rom[33] = 7'h22;
        rom[34] = 7'h22;
        rom[35] = 7'h22;
        rom[36] = 7'h22;
        rom[37] = 7'h22;
        rom[38] = 7'h1E;
        rom[39] = 7'h00;
        rom[40] = 7'h3E;
        rom[41] = 7'h02;
        rom[42] = 7'h02;
        rom[43] = 7'h1E;
        rom[44] = 7'h02;
        rom[45] = 7'h02;
        rom[46] = 7'h3E;
        rom[47] = 7'h00;
        rom[48] = 7'h3E;
        rom[49] = 7'h02;
        rom[50] = 7'h02;
        rom[51] = 7'h1E;
        rom[52] = 7'h02;
        rom[53] = 7'h02;
        rom[54] = 7'h02;
        rom[55] = 7'h00;
        rom[56] = 7'h3C;
        rom[57] = 7'h02;
        rom[58] = 7'h02;
        rom[59] = 7'h02;
        rom[60] = 7'h32;
        rom[61] = 7'h22;
        rom[62] = 7'h3C;
        rom[63] = 7'h00;
        rom[64] = 7'h22;
        rom[65] = 7'h22;
        rom[66] = 7'h22;
        rom[67] = 7'h3E;
        rom[68] = 7'h22;
        rom[69] = 7'h22;
        rom[70] = 7'h22;
        rom[71] = 7'h00;
        rom[72] = 7'h1C;
        rom[73] = 7'h08;
        rom[74] = 7'h08;
        rom[75] = 7'h08;
        rom[76] = 7'h08;
        rom[77] = 7'h08;
        rom[78] = 7'h1C;
        rom[79] = 7'h00;
        rom[80] = 7'h20;
        rom[81] = 7'h20;
        rom[82] = 7'h20;
        rom[83] = 7'h20;
        rom[84] = 7'h20;
        rom[85] = 7'h22;
        rom[86] = 7'h1C;
        rom[87] = 7'h00;
        rom[88] = 7'h22;
        rom[89] = 7'h12;
        rom[90] = 7'h0A;
        rom[91] = 7'h06;
        rom[92] = 7'h0A;
        rom[93] = 7'h12;
        rom[94] = 7'h22;
        rom[95] = 7'h00;
        rom[96] = 7'h02;
        rom[97] = 7'h02;
        rom[98] = 7'h02;
        rom[99] = 7'h02;
        rom[100] = 7'h02;
        rom[101] = 7'h02;
        rom[102] = 7'h3E;
        rom[103] = 7'h00;
        rom[104] = 7'h22;
        rom[105] = 7'h36;
        rom[106] = 7'h2A;
        rom[107] = 7'h2A;
        rom[108] = 7'h22;
        rom[109] = 7'h22;
        rom[110] = 7'h22;
        rom[111] = 7'h00;
        rom[112] = 7'h22;
        rom[113] = 7'h22;
        rom[114] = 7'h26;
        rom[115] = 7'h2A;
        rom[116] = 7'h32;
        rom[117] = 7'h22;
        rom[118] = 7'h22;
        rom[119] = 7'h00;
        rom[120] = 7'h1C;
        rom[121] = 7'h22;
        rom[122] = 7'h22;
        rom[123] = 7'h22;
        rom[124] = 7'h22;
        rom[125] = 7'h22;
        rom[126] = 7'h1C;
        rom[127] = 7'h00;
        rom[128] = 7'h1E;
        rom[129] = 7'h22;
        rom[130] = 7'h22;
        rom[131] = 7'h1E;
        rom[132] = 7'h02;
        rom[133] = 7'h02;
        rom[134] = 7'h02;
        rom[135] = 7'h00;
        rom[136] = 7'h1C;
        rom[137] = 7'h22;
        rom[138] = 7'h22;
        rom[139] = 7'h22;
        rom[140] = 7'h2A;
        rom[141] = 7'h12;
        rom[142] = 7'h2C;
        rom[143] = 7'h00;
        rom[144] = 7'h1E;
        rom[145] = 7'h22;
        rom[146] = 7'h22;
        rom[147] = 7'h1E;
        rom[148] = 7'h0A;
        rom[149] = 7'h12;
        rom[150] = 7'h22;
        rom[151] = 7'h00;
        rom[152] = 7'h1C;
        rom[153] = 7'h22;
        rom[154] = 7'h02;
        rom[155] = 7'h1C;
        rom[156] = 7'h20;
        rom[157] = 7'h22;
        rom[158] = 7'h1C;
        rom[159] = 7'h00;
        rom[160] = 7'h3E;
        rom[161] = 7'h08;
        rom[162] = 7'h08;
        rom[163] = 7'h08;
        rom[164] = 7'h08;
        rom[165] = 7'h08;
        rom[166] = 7'h08;
        rom[167] = 7'h00;
        rom[168] = 7'h22;
        rom[169] = 7'h22;
        rom[170] = 7'h22;
        rom[171] = 7'h22;
        rom[172] = 7'h22;
        rom[173] = 7'h22;
        rom[174] = 7'h1C;
        rom[175] = 7'h00;
        rom[176] = 7'h22;
        rom[177] = 7'h22;
        rom[178] = 7'h22;
        rom[179] = 7'h22;
        rom[180] = 7'h22;
        rom[181] = 7'h14;
        rom[182] = 7'h08;
        rom[183] = 7'h00;
        rom[184] = 7'h22;
        rom[185] = 7'h22;
        rom[186] = 7'h22;
        rom[187] = 7'h2A;
        rom[188] = 7'h2A;
        rom[189] = 7'h36;
        rom[190] = 7'h22;
        rom[191] = 7'h00;
        rom[192] = 7'h22;
        rom[193] = 7'h22;
        rom[194] = 7'h14;
        rom[195] = 7'h08;
        rom[196] = 7'h14;
        rom[197] = 7'h22;
        rom[198] = 7'h22;
        rom[199] = 7'h00;
        rom[200] = 7'h22;
        rom[201] = 7'h22;
        rom[202] = 7'h14;
        rom[203] = 7'h08;
        rom[204] = 7'h08;
        rom[205] = 7'h08;
        rom[206] = 7'h08;
        rom[207] = 7'h00;
        rom[208] = 7'h3E;
        rom[209] = 7'h20;
        rom[210] = 7'h10;
        rom[211] = 7'h08;
        rom[212] = 7'h04;
        rom[213] = 7'h02;
        rom[214] = 7'h3E;
        rom[215] = 7'h00;
        rom[216] = 7'h3E;
        rom[217] = 7'h06;
        rom[218] = 7'h06;
        rom[219] = 7'h06;
        rom[220] = 7'h06;
        rom[221] = 7'h06;
        rom[222] = 7'h3E;
        rom[223] = 7'h00;
        rom[224] = 7'h00;
        rom[225] = 7'h02;
        rom[226] = 7'h04;
        rom[227] = 7'h08;
        rom[228] = 7'h10;
        rom[229] = 7'h20;
        rom[230] = 7'h00;
        rom[231] = 7'h00;
        rom[232] = 7'h3E;
        rom[233] = 7'h30;
        rom[234] = 7'h30;
        rom[235] = 7'h30;
        rom[236] = 7'h30;
        rom[237] = 7'h30;
        rom[238] = 7'h3E;
        rom[239] = 7'h00;
        rom[240] = 7'h00;
        rom[241] = 7'h00;
        rom[242] = 7'h08;
        rom[243] = 7'h14;
        rom[244] = 7'h22;
        rom[245] = 7'h00;
        rom[246] = 7'h00;
        rom[247] = 7'h00;
        rom[248] = 7'h00;
        rom[249] = 7'h00;
        rom[250] = 7'h00;
        rom[251] = 7'h00;
        rom[252] = 7'h00;
        rom[253] = 7'h00;
        rom[254] = 7'h00;
        rom[255] = 7'h7F;
        rom[256] = 7'h00;
        rom[257] = 7'h00;
        rom[258] = 7'h00;
        rom[259] = 7'h00;
        rom[260] = 7'h00;
        rom[261] = 7'h00;
        rom[262] = 7'h00;
        rom[263] = 7'h00;
        rom[264] = 7'h08;
        rom[265] = 7'h08;
        rom[266] = 7'h08;
        rom[267] = 7'h08;
        rom[268] = 7'h08;
        rom[269] = 7'h00;
        rom[270] = 7'h08;
        rom[271] = 7'h00;
        rom[272] = 7'h14;
        rom[273] = 7'h14;
        rom[274] = 7'h14;
        rom[275] = 7'h00;
        rom[276] = 7'h00;
        rom[277] = 7'h00;
        rom[278] = 7'h00;
        rom[279] = 7'h00;
        rom[280] = 7'h14;
        rom[281] = 7'h14;
        rom[282] = 7'h3E;
        rom[283] = 7'h14;
        rom[284] = 7'h3E;
        rom[285] = 7'h14;
        rom[286] = 7'h14;
        rom[287] = 7'h00;
        rom[288] = 7'h08;
        rom[289] = 7'h3C;
        rom[290] = 7'h0A;
        rom[291] = 7'h1C;
        rom[292] = 7'h28;
        rom[293] = 7'h1E;
        rom[294] = 7'h08;
        rom[295] = 7'h00;
        rom[296] = 7'h06;
        rom[297] = 7'h26;
        rom[298] = 7'h10;
        rom[299] = 7'h08;
        rom[300] = 7'h04;
        rom[301] = 7'h32;
        rom[302] = 7'h30;
        rom[303] = 7'h00;
        rom[304] = 7'h04;
        rom[305] = 7'h0A;
        rom[306] = 7'h0A;
        rom[307] = 7'h04;
        rom[308] = 7'h2A;
        rom[309] = 7'h12;
        rom[310] = 7'h2C;
        rom[311] = 7'h00;
        rom[312] = 7'h08;
        rom[313] = 7'h08;
        rom[314] = 7'h08;
        rom[315] = 7'h00;
        rom[316] = 7'h00;
        rom[317] = 7'h00;
        rom[318] = 7'h00;
        rom[319] = 7'h00;
        rom[320] = 7'h08;
        rom[321] = 7'h04;
        rom[322] = 7'h02;
        rom[323] = 7'h02;
        rom[324] = 7'h02;
        rom[325] = 7'h04;
        rom[326] = 7'h08;
        rom[327] = 7'h00;
        rom[328] = 7'h08;
        rom[329] = 7'h10;
        rom[330] = 7'h20;
        rom[331] = 7'h20;
        rom[332] = 7'h20;
        rom[333] = 7'h10;
        rom[334] = 7'h08;
        rom[335] = 7'h00;
        rom[336] = 7'h08;
        rom[337] = 7'h2A;
        rom[338] = 7'h1C;
        rom[339] = 7'h08;
        rom[340] = 7'h1C;
        rom[341] = 7'h2A;
        rom[342] = 7'h08;
        rom[343] = 7'h00;
        rom[344] = 7'h00;
        rom[345] = 7'h08;
        rom[346] = 7'h08;
        rom[347] = 7'h3E;
        rom[348] = 7'h08;
        rom[349] = 7'h08;
        rom[350] = 7'h00;
        rom[351] = 7'h00;
        rom[352] = 7'h00;
        rom[353] = 7'h00;
        rom[354] = 7'h00;
        rom[355] = 7'h00;
        rom[356] = 7'h08;
        rom[357] = 7'h08;
        rom[358] = 7'h04;
        rom[359] = 7'h00;
        rom[360] = 7'h00;
        rom[361] = 7'h00;
        rom[362] = 7'h00;
        rom[363] = 7'h3E;
        rom[364] = 7'h00;
        rom[365] = 7'h00;
        rom[366] = 7'h00;
        rom[367] = 7'h00;
        rom[368] = 7'h00;
        rom[369] = 7'h00;
        rom[370] = 7'h00;
        rom[371] = 7'h00;
        rom[372] = 7'h00;
        rom[373] = 7'h00;
        rom[374] = 7'h08;
        rom[375] = 7'h00;
        rom[376] = 7'h00;
        rom[377] = 7'h20;
        rom[378] = 7'h10;
        rom[379] = 7'h08;
        rom[380] = 7'h04;
        rom[381] = 7'h02;
        rom[382] = 7'h00;
        rom[383] = 7'h00;
        rom[384] = 7'h1C;
        rom[385] = 7'h22;
        rom[386] = 7'h32;
        rom[387] = 7'h2A;
        rom[388] = 7'h26;
        rom[389] = 7'h22;
        rom[390] = 7'h1C;
        rom[391] = 7'h00;
        rom[392] = 7'h08;
        rom[393] = 7'h0C;
        rom[394] = 7'h08;
        rom[395] = 7'h08;
        rom[396] = 7'h08;
        rom[397] = 7'h08;
        rom[398] = 7'h1C;
        rom[399] = 7'h00;
        rom[400] = 7'h1C;
        rom[401] = 7'h22;
        rom[402] = 7'h20;
        rom[403] = 7'h18;
        rom[404] = 7'h04;
        rom[405] = 7'h02;
        rom[406] = 7'h3E;
        rom[407] = 7'h00;
        rom[408] = 7'h3E;
        rom[409] = 7'h20;
        rom[410] = 7'h10;
        rom[411] = 7'h18;
        rom[412] = 7'h20;
        rom[413] = 7'h22;
        rom[414] = 7'h1C;
        rom[415] = 7'h00;
        rom[416] = 7'h10;
        rom[417] = 7'h18;
        rom[418] = 7'h14;
        rom[419] = 7'h12;
        rom[420] = 7'h3E;
        rom[421] = 7'h10;
        rom[422] = 7'h10;
        rom[423] = 7'h00;
        rom[424] = 7'h3E;
        rom[425] = 7'h02;
        rom[426] = 7'h1E;
        rom[427] = 7'h20;
        rom[428] = 7'h20;
        rom[429] = 7'h22;
        rom[430] = 7'h1C;
        rom[431] = 7'h00;
        rom[432] = 7'h38;
        rom[433] = 7'h04;
        rom[434] = 7'h02;
        rom[435] = 7'h1E;
        rom[436] = 7'h22;
        rom[437] = 7'h22;
        rom[438] = 7'h1C;
        rom[439] = 7'h00;
        rom[440] = 7'h3E;
        rom[441] = 7'h20;
        rom[442] = 7'h10;
        rom[443] = 7'h08;
        rom[444] = 7'h04;
        rom[445] = 7'h04;
        rom[446] = 7'h04;
        rom[447] = 7'h00;
        rom[448] = 7'h1C;
        rom[449] = 7'h22;
        rom[450] = 7'h22;
        rom[451] = 7'h1C;
        rom[452] = 7'h22;
        rom[453] = 7'h22;
        rom[454] = 7'h1C;
        rom[455] = 7'h00;
        rom[456] = 7'h1C;
        rom[457] = 7'h22;
        rom[458] = 7'h22;
        rom[459] = 7'h3C;
        rom[460] = 7'h20;
        rom[461] = 7'h10;
        rom[462] = 7'h0E;
        rom[463] = 7'h00;
        rom[464] = 7'h00;
        rom[465] = 7'h00;
        rom[466] = 7'h08;
        rom[467] = 7'h00;
        rom[468] = 7'h08;
        rom[469] = 7'h00;
        rom[470] = 7'h00;
        rom[471] = 7'h00;
        rom[472] = 7'h00;
        rom[473] = 7'h00;
        rom[474] = 7'h08;
        rom[475] = 7'h00;
        rom[476] = 7'h08;
        rom[477] = 7'h08;
        rom[478] = 7'h04;
        rom[479] = 7'h00;
        rom[480] = 7'h10;
        rom[481] = 7'h08;
        rom[482] = 7'h04;
        rom[483] = 7'h02;
        rom[484] = 7'h04;
        rom[485] = 7'h08;
        rom[486] = 7'h10;
        rom[487] = 7'h00;
        rom[488] = 7'h00;
        rom[489] = 7'h00;
        rom[490] = 7'h3E;
        rom[491] = 7'h00;
        rom[492] = 7'h3E;
        rom[493] = 7'h00;
        rom[494] = 7'h00;
        rom[495] = 7'h00;
        rom[496] = 7'h04;
        rom[497] = 7'h08;
        rom[498] = 7'h10;
        rom[499] = 7'h20;
        rom[500] = 7'h10;
        rom[501] = 7'h08;
        rom[502] = 7'h04;
        rom[503] = 7'h00;
        rom[504] = 7'h1C;
        rom[505] = 7'h22;
        rom[506] = 7'h10;
        rom[507] = 7'h08;
        rom[508] = 7'h08;
        rom[509] = 7'h00;
        rom[510] = 7'h08;
        rom[511] = 7'h00;
        rom[512] = 7'h1C;
        rom[513] = 7'h22;
        rom[514] = 7'h2A;
        rom[515] = 7'h3A;
        rom[516] = 7'h1A;
        rom[517] = 7'h02;
        rom[518] = 7'h3C;
        rom[519] = 7'h00;
        rom[520] = 7'h08;
        rom[521] = 7'h14;
        rom[522] = 7'h22;
        rom[523] = 7'h22;
        rom[524] = 7'h3E;
        rom[525] = 7'h22;
        rom[526] = 7'h22;
        rom[527] = 7'h00;
        rom[528] = 7'h1E;
        rom[529] = 7'h22;
        rom[530] = 7'h22;
        rom[531] = 7'h1E;
        rom[532] = 7'h22;
        rom[533] = 7'h22;
        rom[534] = 7'h1E;
        rom[535] = 7'h00;
        rom[536] = 7'h1C;
        rom[537] = 7'h22;
        rom[538] = 7'h02;
        rom[539] = 7'h02;
        rom[540] = 7'h02;
        rom[541] = 7'h22;
        rom[542] = 7'h1C;
        rom[543] = 7'h00;
        rom[544] = 7'h1E;
        rom[545] = 7'h22;
        rom[546] = 7'h22;
        rom[547] = 7'h22;
        rom[548] = 7'h22;
        rom[549] = 7'h22;
        rom[550] = 7'h1E;
        rom[551] = 7'h00;
        rom[552] = 7'h3E;
        rom[553] = 7'h02;
        rom[554] = 7'h02;
        rom[555] = 7'h1E;
        rom[556] = 7'h02;
        rom[557] = 7'h02;
        rom[558] = 7'h3E;
        rom[559] = 7'h00;
        rom[560] = 7'h3E;
        rom[561] = 7'h02;
        rom[562] = 7'h02;
        rom[563] = 7'h1E;
        rom[564] = 7'h02;
        rom[565] = 7'h02;
        rom[566] = 7'h02;
        rom[567] = 7'h00;
        rom[568] = 7'h3C;
        rom[569] = 7'h02;
        rom[570] = 7'h02;
        rom[571] = 7'h02;
        rom[572] = 7'h32;
        rom[573] = 7'h22;
        rom[574] = 7'h3C;
        rom[575] = 7'h00;
        rom[576] = 7'h22;
        rom[577] = 7'h22;
        rom[578] = 7'h22;
        rom[579] = 7'h3E;
        rom[580] = 7'h22;
        rom[581] = 7'h22;
        rom[582] = 7'h22;
        rom[583] = 7'h00;
        rom[584] = 7'h1C;
        rom[585] = 7'h08;
        rom[586] = 7'h08;
        rom[587] = 7'h08;
        rom[588] = 7'h08;
        rom[589] = 7'h08;
        rom[590] = 7'h1C;
        rom[591] = 7'h00;
        rom[592] = 7'h20;
        rom[593] = 7'h20;
        rom[594] = 7'h20;
        rom[595] = 7'h20;
        rom[596] = 7'h20;
        rom[597] = 7'h22;
        rom[598] = 7'h1C;
        rom[599] = 7'h00;
        rom[600] = 7'h22;
        rom[601] = 7'h12;
        rom[602] = 7'h0A;
        rom[603] = 7'h06;
        rom[604] = 7'h0A;
        rom[605] = 7'h12;
        rom[606] = 7'h22;
        rom[607] = 7'h00;
        rom[608] = 7'h02;
        rom[609] = 7'h02;
        rom[610] = 7'h02;
        rom[611] = 7'h02;
        rom[612] = 7'h02;
        rom[613] = 7'h02;
        rom[614] = 7'h3E;
        rom[615] = 7'h00;
        rom[616] = 7'h22;
        rom[617] = 7'h36;
        rom[618] = 7'h2A;
        rom[619] = 7'h2A;
        rom[620] = 7'h22;
        rom[621] = 7'h22;
        rom[622] = 7'h22;
        rom[623] = 7'h00;
        rom[624] = 7'h22;
        rom[625] = 7'h22;
        rom[626] = 7'h26;
        rom[627] = 7'h2A;
        rom[628] = 7'h32;
        rom[629] = 7'h22;
        rom[630] = 7'h22;
        rom[631] = 7'h00;
        rom[632] = 7'h1C;
        rom[633] = 7'h22;
        rom[634] = 7'h22;
        rom[635] = 7'h22;
        rom[636] = 7'h22;
        rom[637] = 7'h22;
        rom[638] = 7'h1C;
        rom[639] = 7'h00;
        rom[640] = 7'h1E;
        rom[641] = 7'h22;
        rom[642] = 7'h22;
        rom[643] = 7'h1E;
        rom[644] = 7'h02;
        rom[645] = 7'h02;
        rom[646] = 7'h02;
        rom[647] = 7'h00;
        rom[648] = 7'h1C;
        rom[649] = 7'h22;
        rom[650] = 7'h22;
        rom[651] = 7'h22;
        rom[652] = 7'h2A;
        rom[653] = 7'h12;
        rom[654] = 7'h2C;
        rom[655] = 7'h00;
        rom[656] = 7'h1E;
        rom[657] = 7'h22;
        rom[658] = 7'h22;
        rom[659] = 7'h1E;
        rom[660] = 7'h0A;
        rom[661] = 7'h12;
        rom[662] = 7'h22;
        rom[663] = 7'h00;
        rom[664] = 7'h1C;
        rom[665] = 7'h22;
        rom[666] = 7'h02;
        rom[667] = 7'h1C;
        rom[668] = 7'h20;
        rom[669] = 7'h22;
        rom[670] = 7'h1C;
        rom[671] = 7'h00;
        rom[672] = 7'h3E;
        rom[673] = 7'h08;
        rom[674] = 7'h08;
        rom[675] = 7'h08;
        rom[676] = 7'h08;
        rom[677] = 7'h08;
        rom[678] = 7'h08;
        rom[679] = 7'h00;
        rom[680] = 7'h22;
        rom[681] = 7'h22;
        rom[682] = 7'h22;
        rom[683] = 7'h22;
        rom[684] = 7'h22;
        rom[685] = 7'h22;
        rom[686] = 7'h1C;
        rom[687] = 7'h00;
        rom[688] = 7'h22;
        rom[689] = 7'h22;
        rom[690] = 7'h22;
        rom[691] = 7'h22;
        rom[692] = 7'h22;
        rom[693] = 7'h14;
        rom[694] = 7'h08;
        rom[695] = 7'h00;
        rom[696] = 7'h22;
        rom[697] = 7'h22;
        rom[698] = 7'h22;
        rom[699] = 7'h2A;
        rom[700] = 7'h2A;
        rom[701] = 7'h36;
        rom[702] = 7'h22;
        rom[703] = 7'h00;
        rom[704] = 7'h22;
        rom[705] = 7'h22;
        rom[706] = 7'h14;
        rom[707] = 7'h08;
        rom[708] = 7'h14;
        rom[709] = 7'h22;
        rom[710] = 7'h22;
        rom[711] = 7'h00;
        rom[712] = 7'h22;
        rom[713] = 7'h22;
        rom[714] = 7'h14;
        rom[715] = 7'h08;
        rom[716] = 7'h08;
        rom[717] = 7'h08;
        rom[718] = 7'h08;
        rom[719] = 7'h00;
        rom[720] = 7'h3E;
        rom[721] = 7'h20;
        rom[722] = 7'h10;
        rom[723] = 7'h08;
        rom[724] = 7'h04;
        rom[725] = 7'h02;
        rom[726] = 7'h3E;
        rom[727] = 7'h00;
        rom[728] = 7'h3E;
        rom[729] = 7'h06;
        rom[730] = 7'h06;
        rom[731] = 7'h06;
        rom[732] = 7'h06;
        rom[733] = 7'h06;
        rom[734] = 7'h3E;
        rom[735] = 7'h00;
        rom[736] = 7'h00;
        rom[737] = 7'h02;
        rom[738] = 7'h04;
        rom[739] = 7'h08;
        rom[740] = 7'h10;
        rom[741] = 7'h20;
        rom[742] = 7'h00;
        rom[743] = 7'h00;
        rom[744] = 7'h3E;
        rom[745] = 7'h30;
        rom[746] = 7'h30;
        rom[747] = 7'h30;
        rom[748] = 7'h30;
        rom[749] = 7'h30;
        rom[750] = 7'h3E;
        rom[751] = 7'h00;
        rom[752] = 7'h00;
        rom[753] = 7'h00;
        rom[754] = 7'h08;
        rom[755] = 7'h14;
        rom[756] = 7'h22;
        rom[757] = 7'h00;
        rom[758] = 7'h00;
        rom[759] = 7'h00;
        rom[760] = 7'h00;
        rom[761] = 7'h00;
        rom[762] = 7'h00;
        rom[763] = 7'h00;
        rom[764] = 7'h00;
        rom[765] = 7'h00;
        rom[766] = 7'h00;
        rom[767] = 7'h7F;
        rom[768] = 7'h04;
        rom[769] = 7'h08;
        rom[770] = 7'h10;
        rom[771] = 7'h00;
        rom[772] = 7'h00;
        rom[773] = 7'h00;
        rom[774] = 7'h00;
        rom[775] = 7'h00;
        rom[776] = 7'h00;
        rom[777] = 7'h00;
        rom[778] = 7'h1C;
        rom[779] = 7'h20;
        rom[780] = 7'h3C;
        rom[781] = 7'h22;
        rom[782] = 7'h3C;
        rom[783] = 7'h00;
        rom[784] = 7'h02;
        rom[785] = 7'h02;
        rom[786] = 7'h1E;
        rom[787] = 7'h22;
        rom[788] = 7'h22;
        rom[789] = 7'h22;
        rom[790] = 7'h1E;
        rom[791] = 7'h00;
        rom[792] = 7'h00;
        rom[793] = 7'h00;
        rom[794] = 7'h3C;
        rom[795] = 7'h02;
        rom[796] = 7'h02;
        rom[797] = 7'h02;
        rom[798] = 7'h3C;
        rom[799] = 7'h00;
        rom[800] = 7'h20;
        rom[801] = 7'h20;
        rom[802] = 7'h3C;
        rom[803] = 7'h22;
        rom[804] = 7'h22;
        rom[805] = 7'h22;
        rom[806] = 7'h3C;
        rom[807] = 7'h00;
        rom[808] = 7'h00;
        rom[809] = 7'h00;
        rom[810] = 7'h1C;
        rom[811] = 7'h22;
        rom[812] = 7'h3E;
        rom[813] = 7'h02;
        rom[814] = 7'h3C;
        rom[815] = 7'h00;
        rom[816] = 7'h18;
        rom[817] = 7'h24;
        rom[818] = 7'h04;
        rom[819] = 7'h1E;
        rom[820] = 7'h04;
        rom[821] = 7'h04;
        rom[822] = 7'h04;
        rom[823] = 7'h00;
        rom[824] = 7'h00;
        rom[825] = 7'h00;
        rom[826] = 7'h1C;
        rom[827] = 7'h22;
        rom[828] = 7'h22;
        rom[829] = 7'h3C;
        rom[830] = 7'h20;
        rom[831] = 7'h1C;
        rom[832] = 7'h02;
        rom[833] = 7'h02;
        rom[834] = 7'h1E;
        rom[835] = 7'h22;
        rom[836] = 7'h22;
        rom[837] = 7'h22;
        rom[838] = 7'h22;
        rom[839] = 7'h00;
        rom[840] = 7'h08;
        rom[841] = 7'h00;
        rom[842] = 7'h0C;
        rom[843] = 7'h08;
        rom[844] = 7'h08;
        rom[845] = 7'h08;
        rom[846] = 7'h1C;
        rom[847] = 7'h00;
        rom[848] = 7'h10;
        rom[849] = 7'h00;
        rom[850] = 7'h18;
        rom[851] = 7'h10;
        rom[852] = 7'h10;
        rom[853] = 7'h10;
        rom[854] = 7'h12;
        rom[855] = 7'h0C;
        rom[856] = 7'h02;
        rom[857] = 7'h02;
        rom[858] = 7'h22;
        rom[859] = 7'h12;
        rom[860] = 7'h0E;
        rom[861] = 7'h12;
        rom[862] = 7'h22;
        rom[863] = 7'h00;
        rom[864] = 7'h0C;
        rom[865] = 7'h08;
        rom[866] = 7'h08;
        rom[867] = 7'h08;
        rom[868] = 7'h08;
        rom[869] = 7'h08;
        rom[870] = 7'h1C;
        rom[871] = 7'h00;
        rom[872] = 7'h00;
        rom[873] = 7'h00;
        rom[874] = 7'h36;
        rom[875] = 7'h2A;
        rom[876] = 7'h2A;
        rom[877] = 7'h2A;
        rom[878] = 7'h22;
        rom[879] = 7'h00;
        rom[880] = 7'h00;
        rom[881] = 7'h00;
        rom[882] = 7'h1E;
        rom[883] = 7'h22;
        rom[884] = 7'h22;
        rom[885] = 7'h22;
        rom[886] = 7'h22;
        rom[887] = 7'h00;
        rom[888] = 7'h00;
        rom[889] = 7'h00;
        rom[890] = 7'h1C;
        rom[891] = 7'h22;
        rom[892] = 7'h22;
        rom[893] = 7'h22;
        rom[894] = 7'h1C;
        rom[895] = 7'h00;
        rom[896] = 7'h00;
        rom[897] = 7'h00;
        rom[898] = 7'h1E;
        rom[899] = 7'h22;
        rom[900] = 7'h22;
        rom[901] = 7'h1E;
        rom[902] = 7'h02;
        rom[903] = 7'h02;
        rom[904] = 7'h00;
        rom[905] = 7'h00;
        rom[906] = 7'h3C;
        rom[907] = 7'h22;
        rom[908] = 7'h22;
        rom[909] = 7'h3C;
        rom[910] = 7'h20;
        rom[911] = 7'h20;
        rom[912] = 7'h00;
        rom[913] = 7'h00;
        rom[914] = 7'h3A;
        rom[915] = 7'h06;
        rom[916] = 7'h02;
        rom[917] = 7'h02;
        rom[918] = 7'h02;
        rom[919] = 7'h00;
        rom[920] = 7'h00;
        rom[921] = 7'h00;
        rom[922] = 7'h3C;
        rom[923] = 7'h02;
        rom[924] = 7'h1C;
        rom[925] = 7'h20;
        rom[926] = 7'h1E;
        rom[927] = 7'h00;
        rom[928] = 7'h04;
        rom[929] = 7'h04;
        rom[930] = 7'h1E;
        rom[931] = 7'h04;
        rom[932] = 7'h04;
        rom[933] = 7'h24;
        rom[934] = 7'h18;
        rom[935] = 7'h00;
        rom[936] = 7'h00;
        rom[937] = 7'h00;
        rom[938] = 7'h22;
        rom[939] = 7'h22;
        rom[940] = 7'h22;
        rom[941] = 7'h32;
        rom[942] = 7'h2C;
        rom[943] = 7'h00;
        rom[944] = 7'h00;
        rom[945] = 7'h00;
        rom[946] = 7'h22;
        rom[947] = 7'h22;
        rom[948] = 7'h22;
        rom[949] = 7'h14;
        rom[950] = 7'h08;
        rom[951] = 7'h00;
        rom[952] = 7'h00;
        rom[953] = 7'h00;
        rom[954] = 7'h22;
        rom[955] = 7'h22;
        rom[956] = 7'h2A;
        rom[957] = 7'h2A;
        rom[958] = 7'h36;
        rom[959] = 7'h00;
        rom[960] = 7'h00;
        rom[961] = 7'h00;
        rom[962] = 7'h22;
        rom[963] = 7'h14;
        rom[964] = 7'h08;
        rom[965] = 7'h14;
        rom[966] = 7'h22;
        rom[967] = 7'h00;
        rom[968] = 7'h00;
        rom[969] = 7'h00;
        rom[970] = 7'h22;
        rom[971] = 7'h22;
        rom[972] = 7'h22;
        rom[973] = 7'h3C;
        rom[974] = 7'h20;
        rom[975] = 7'h1C;
        rom[976] = 7'h00;
        rom[977] = 7'h00;
        rom[978] = 7'h3E;
        rom[979] = 7'h10;
        rom[980] = 7'h08;
        rom[981] = 7'h04;
        rom[982] = 7'h3E;
        rom[983] = 7'h00;
        rom[984] = 7'h38;
        rom[985] = 7'h0C;
        rom[986] = 7'h0C;
        rom[987] = 7'h06;
        rom[988] = 7'h0C;
        rom[989] = 7'h0C;
        rom[990] = 7'h38;
        rom[991] = 7'h00;
        rom[992] = 7'h08;
        rom[993] = 7'h08;
        rom[994] = 7'h08;
        rom[995] = 7'h08;
        rom[996] = 7'h08;
        rom[997] = 7'h08;
        rom[998] = 7'h08;
        rom[999] = 7'h08;
        rom[1000] = 7'h0E;
        rom[1001] = 7'h18;
        rom[1002] = 7'h18;
        rom[1003] = 7'h30;
        rom[1004] = 7'h18;
        rom[1005] = 7'h18;
        rom[1006] = 7'h0E;
        rom[1007] = 7'h00;
        rom[1008] = 7'h2C;
        rom[1009] = 7'h1A;
        rom[1010] = 7'h00;
        rom[1011] = 7'h00;
        rom[1012] = 7'h00;
        rom[1013] = 7'h00;
        rom[1014] = 7'h00;
        rom[1015] = 7'h00;
        rom[1016] = 7'h00;
        rom[1017] = 7'h2A;
        rom[1018] = 7'h14;
        rom[1019] = 7'h2A;
        rom[1020] = 7'h14;
        rom[1021] = 7'h2A;
        rom[1022] = 7'h00;
        rom[1023] = 7'h00;
    end

    always_ff @(posedge clk) begin
        char_data_out <= rom[char_addr];        
    end

endmodule


`timescale 1ns / 10ps

module color_rom (
    input logic clk,
    input logic [6:0] color_addr,
    output logic [11:0] color_data_out
);
    logic [11:0] rom [0:127];

    initial begin
        rom[0] = 12'h0F4;
        rom[1] = 12'h0B9;
        rom[2] = 12'h06E;
        rom[3] = 12'h06F;
        rom[4] = 12'h45F;
        rom[5] = 12'h000;
        rom[6] = 12'h000;
        rom[7] = 12'h000;
        rom[8] = 12'h0C0;
        rom[9] = 12'h082;
        rom[10] = 12'h037;
        rom[11] = 12'h22F;
        rom[12] = 12'h62F;
        rom[13] = 12'h000;
        rom[14] = 12'h000;
        rom[15] = 12'h000;
        rom[16] = 12'h090;
        rom[17] = 12'h040;
        rom[18] = 12'h000;
        rom[19] = 12'h409;
        rom[20] = 12'h90F;
        rom[21] = 12'h000;
        rom[22] = 12'h000;
        rom[23] = 12'h000;
        rom[24] = 12'h5A0;
        rom[25] = 12'h650;
        rom[26] = 12'h610;
        rom[27] = 12'hB06;
        rom[28] = 12'hF0E;
        rom[29] = 12'h000;
        rom[30] = 12'h000;
        rom[31] = 12'h000;
        rom[32] = 12'hBA0;
        rom[33] = 12'hC60;
        rom[34] = 12'hC10;
        rom[35] = 12'hF12;
        rom[36] = 12'hF0B;
        rom[37] = 12'h000;
        rom[38] = 12'h000;
        rom[39] = 12'h000;
        rom[40] = 12'h000;
        rom[41] = 12'h000;
        rom[42] = 12'h000;
        rom[43] = 12'h000;
        rom[44] = 12'h000;
        rom[45] = 12'h000;
        rom[46] = 12'h000;
        rom[47] = 12'h000;
        rom[48] = 12'h000;
        rom[49] = 12'h000;
        rom[50] = 12'h000;
        rom[51] = 12'h000;
        rom[52] = 12'h000;
        rom[53] = 12'h000;
        rom[54] = 12'h000;
        rom[55] = 12'h000;
        rom[56] = 12'h000;
        rom[57] = 12'h000;
        rom[58] = 12'h000;
        rom[59] = 12'h000;
        rom[60] = 12'h000;
        rom[61] = 12'h000;
        rom[62] = 12'h000;
        rom[63] = 12'h000;
        rom[64] = 12'h2FC;
        rom[65] = 12'h5FF;
        rom[66] = 12'h7FF;
        rom[67] = 12'hAFF;
        rom[68] = 12'hCDF;
        rom[69] = 12'h000;
        rom[70] = 12'h000;
        rom[71] = 12'h000;
        rom[72] = 12'h7F7;
        rom[73] = 12'h9FE;
        rom[74] = 12'hCFF;
        rom[75] = 12'hEFF;
        rom[76] = 12'hFCF;
        rom[77] = 12'h000;
        rom[78] = 12'h000;
        rom[79] = 12'h000;
        rom[80] = 12'hBF2;
        rom[81] = 12'hEF9;
        rom[82] = 12'hFFF;
        rom[83] = 12'hFDF;
        rom[84] = 12'hFBF;
        rom[85] = 12'h000;
        rom[86] = 12'h000;
        rom[87] = 12'h000;
        rom[88] = 12'hFF0;
        rom[89] = 12'hFF4;
        rom[90] = 12'hFFB;
        rom[91] = 12'hFCF;
        rom[92] = 12'hFAF;
        rom[93] = 12'h000;
        rom[94] = 12'h000;
        rom[95] = 12'h000;
        rom[96] = 12'hFF0;
        rom[97] = 12'hFF0;
        rom[98] = 12'hFE6;
        rom[99] = 12'hFBD;
        rom[100] = 12'hF8F;
        rom[101] = 12'h000;
        rom[102] = 12'h000;
        rom[103] = 12'h000;
        rom[104] = 12'h000;
        rom[105] = 12'h000;
        rom[106] = 12'h000;
        rom[107] = 12'h000;
        rom[108] = 12'h000;
        rom[109] = 12'h000;
        rom[110] = 12'h000;
        rom[111] = 12'h000;
        rom[112] = 12'h000;
        rom[113] = 12'h000;
        rom[114] = 12'h000;
        rom[115] = 12'h000;
        rom[116] = 12'h000;
        rom[117] = 12'h000;
        rom[118] = 12'h000;
        rom[119] = 12'h000;
        rom[120] = 12'h000;
        rom[121] = 12'h000;
        rom[122] = 12'h000;
        rom[123] = 12'h000;
        rom[124] = 12'h000;
        rom[125] = 12'h000;
        rom[126] = 12'h000;
        rom[127] = 12'h000;
    end

    always_ff @(posedge clk) begin
        color_data_out <= rom[color_addr];        
    end

endmodule


`timescale 1ns/1ps
//------------------------------------------------------------------------------
// debouncer (SystemVerilog)
// Samples input I; after COUNT_MAX stable cycles, drives O to I
//------------------------------------------------------------------------------

module debouncer #(
    parameter int unsigned COUNT_MAX   = 255,
    parameter int unsigned COUNT_WIDTH = 8
) (
    input  logic clk,
    input  logic I,
    input logic nrst,
    output logic O
);

    // logic [COUNT_WIDTH-1:0] count = '0;
    // logic                   Iv    = 1'b0;

    // always_ff @(posedge clk) begin

    //     if (I == Iv) begin
    //         if (count == COUNT_MAX[COUNT_WIDTH-1:0]) begin
    //             O <= I;
    //         end else begin
    //             count <= count + 1'b1;
    //         end
    //     end else begin
    //         count <= '0;
    //         Iv    <= I;
    //     end
    // end

    logic [COUNT_WIDTH-1:0] count, count_next;
    logic O_next, Iv, Iv_next;

    always_ff @(posedge clk, negedge nrst) begin
        if(nrst == 0) begin
            Iv <= 0;
            O <= 0;
            count <= '0;
        end
        else begin
            Iv <= Iv_next;
            O <= O_next;
            count <= count_next;
        end
    end
    always_comb begin
    O_next = O;
    Iv_next = Iv;
    count_next = count;
    if(Iv == I) begin
        if (count == COUNT_MAX[COUNT_WIDTH-1:0]) begin
            O_next = I;
        end else begin
            count_next = count + 1'b1;
        end
    end
    else begin 
        Iv_next = I;
        count_next = '0;
    end
    end
endmodule
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

module latch(
    input  logic       clk,
    input logic nrst,
    input  logic       strobe,       // connect oflag
    input  logic [7:0] din,          // connect keycode[7:0]
    output logic [7:0] dout = 8'h00  // last received byte
);
    always_ff @(posedge clk, negedge nrst) begin
        if(nrst == 0) begin
            dout <= '0;
        end else begin
            if(strobe) begin 
                dout <= din;
            end
            else begin
                dout <= din;
            end
        end
    end
endmodule

`timescale 1ns / 10ps

module vga_controller (
    input logic clk, // 25 MHz
    input logic n_rst,
    output logic hsync,
    output logic vsync,
    output logic [3:0] r, g, b
);
    localparam H_DISPLAY     = 640; 
    localparam H_SYNC        = 96;
    localparam H_FRONT_PORCH = 16;
    localparam H_BACK_PORCH  = 48;
    localparam H_LINE        = H_DISPLAY + H_SYNC + H_FRONT_PORCH + H_BACK_PORCH - 1;

    localparam V_DISPLAY     = 480;
    localparam V_SYNC        = 2;
    localparam V_FRONT_PORCH = 10;
    localparam V_BACK_PORCH  = 33;
    localparam V_LINE        = V_DISPLAY + V_SYNC + V_FRONT_PORCH + V_BACK_PORCH - 1;
    
    logic [9:0] hcount, vcount;
    logic [9:0] next_hcount, next_vcount;
    logic next_hsync, next_vsync;
    logic video_active;

    always_ff @(posedge clk, negedge n_rst) begin : COUNTER_REG
        if (!n_rst) begin
            hcount <= 10'b0; // pixel x 
            vcount <= 10'b0; // pixel y
            hsync  <= 1'b1;
            vsync  <= 1'b1;
        end else begin
            hcount <= next_hcount;
            vcount <= next_vcount;
            hsync  <= next_hsync;
            vsync  <= next_vsync;
        end
    end

    always_comb begin : OUTPUT_LOGIC
        next_vcount = vcount;
        if (hcount < H_LINE) begin
            next_hcount = hcount + 1;
        end else begin
            next_hcount = 0;
            if (vcount < V_LINE) begin
                next_vcount = vcount + 1;
            end else begin
                next_vcount = 0;
            end
        end

        next_hsync = !(hcount >= H_DISPLAY + H_FRONT_PORCH && hcount < H_DISPLAY + H_FRONT_PORCH + H_SYNC);
        next_vsync = !(vcount >= V_DISPLAY + V_FRONT_PORCH && vcount < V_DISPLAY + V_FRONT_PORCH + V_SYNC);
        video_active = (hcount < H_DISPLAY && vcount < V_DISPLAY);
    end

    always_comb begin : TV_BAR_TEST // for TESTING monitor
        {r,g,b} = 12'h0; // black
        if (video_active) begin
            case (hcount[9:6])
                4'd0:    {r,g,b} = 12'hF00; // red
                4'd1:    {r,g,b} = 12'h0F0; // green
                4'd2:    {r,g,b} = 12'h00F; // blue
                4'd3:    {r,g,b} = 12'hFF0; // yellow
                4'd4:    {r,g,b} = 12'hF0F; // magenta
                4'd5:    {r,g,b} = 12'h0FF; // cyan
                4'd6:    {r,g,b} = 12'hFFF; // white
                4'd7:    {r,g,b} = 12'h888; // gray
                default: {r,g,b} = 12'h000; // black
            endcase
        end
    end
    
endmodule

`timescale 1ns / 10ps

module vga_test (
	input logic clk_25mhz,	// clock 25MHz
	input logic clk_12mhz,  // clock 12.5MHz
	input logic [7:0] data_in,	    // data from RAM
	output logic [15:0] vid_addr,	// video memory addr
	output logic read_mem,  // read memory when high
	output logic hsync,	    // horizontal sync
	output logic vsync,	    // vertical sync
	
	input logic text,		// text mode
	input logic hires,	    // hires/lores graphics mode
	input logic mix,		// mix mode
	input logic page2,	    // page2/page1
	
	input logic color,	// color/blackwhite
	
	output logic [11:0] video	// RGB444 video output
);
// internal signals
logic hsync_i = 1;
logic hde = 0;
logic [2:0]pixcnt = 0;
logic pixtc;
logic [5:0]chrcnt = 0;
logic nl;
logic [9:0]vc = 523;	
logic vsync_i = 1;
logic vde = 0;
logic mixed_line = 0;

// horizontal timing
assign hsync = hsync_i;
// pixel counter
always_ff @(posedge clk_12mhz) begin
	if (pixtc | nl) pixcnt <= 0;
	else pixcnt <= (pixcnt == 6) ? 0 : pixcnt + 1;
end

assign pixtc = (pixcnt == 6);

// count 40 characters per line
always_ff @(posedge clk_12mhz) begin
	if (nl) chrcnt <= 0;
	else if (pixtc) chrcnt <= chrcnt + 1;
end

assign nl = (chrcnt == 57);

// horizontal display enable / hsync for VGA
always_ff @(posedge clk_12mhz) begin
	hde <= nl ? 1 : ((pixtc & (chrcnt == 39)) ? 0 : hde);
	if (pixtc) begin
		if (chrcnt == 43) hsync_i <= 0;
		if (chrcnt == 51) hsync_i <= 1;
	end
end

// vertical timing
assign vsync = vsync_i;

always_ff @(posedge hde) begin
	if (vc == 10'd524) begin vc <= 0; vde <= 1; mixed_line <= 0; end 
	else vc <= vc + 1;
	if (vc == 10'd319) mixed_line <= 1;
	if (vc == 10'd383) vde <= 0;
	if (vc == 10'd441) vsync_i <= 0;
	if (vc == 10'd443) vsync_i <= 1;
end

// display enable and data read from memory
logic de;
assign de = hde & vde;
assign read_mem = de & (pixcnt == 0);

// video intermediate sum for addressing
logic [3:0] sum;
assign sum = {vc[8:7], vc[8:7]} + {1'b0, chrcnt[5:3]};

logic [9:0] txtva;
assign txtva = {vc[6:4], sum, chrcnt[2:0]};
	
// graphics enable (or text when disabled)
logic gr_en;
assign gr_en = ~text & hires & (~(mix & mixed_line));

// video address for memory
assign vid_addr[15:0] = {gr_en ? {1'b0, page2, ~page2, vc[3:1]} : {4'b0000, page2, ~page2}, txtva};
	
// character ROM
logic [4:0] osc = 0;
always_ff @(posedge vsync_i) begin // 2 Hz flash for characters
	osc <= (osc == 5'd30) ? 5'd1 : (osc + 1);
end

logic [7:0] grd;
logic invtxt;
always_comb begin
	case ({grd[7:6]})
		2'b00:   invtxt = 1;
		2'b01:   invtxt = osc[4];
		default: invtxt = 0;
	endcase
end

logic [6:0] charout;
logic [9:0] charaddr;
assign charaddr = {1'b0, data_in[5:0], vc[3:1]};

character_rom CHAR (.clk(clk_12mhz), .char_addr(charaddr), .char_data_out(charout));

logic [6:0] txtout;
assign txtout = invtxt ? (~charout[6:0]) : charout[6:0];

// graphics register
always_ff @(posedge clk_12mhz) begin
	if (read_mem) grd <= data_in;
end

// pixel display information
logic [6:0] pixd;
assign pixd = gr_en ? grd[6:0] : txtout;

logic ld1; 
assign ld1 = (pixcnt == 1) & de & (~clk_12mhz);

logic [6:0] sh;
logic rd7 = 0;

// shift register for display pixels
always_ff @(posedge clk_25mhz) begin
	sh <= (ld1) ? pixd : ((~clk_12mhz) ? {1'b0, sh[6:1]} : sh);
end

always_ff @(posedge clk_25mhz) begin
	if (ld1) rd7 <= (~text) & grd[7];
end

logic dpix;
always_ff @(posedge clk_25mhz) begin
	dpix <= sh[0];
end

logic hrpixel; // hires half pixel cycle
assign hrpixel = rd7 ? dpix : sh[0];

logic lrload = 0;
always_ff @(posedge clk_25mhz) begin 
	lrload <= (pixcnt == 1) & (clk_12mhz); // lores mode
end

logic [3:0] lrsh;
logic [3:0] lrdata;
assign lrdata = de ? (vc[3] ? grd[7:4] : grd[3:0]) : 0;
always_ff @(posedge clk_25mhz) begin 
	lrsh <= lrload ? lrdata : {lrsh[0], lrsh[3:1]};
end

logic lrdchr = 0;	// lores mode handling for even and odd bytes
always_ff @(posedge clk_12mhz) begin 
	if (pixcnt==1) lrdchr <= chrcnt[0];
end

logic lrpixel;
assign lrpixel = lrdchr ? lrsh[2] : lrsh[0];

// video output pixel (Apple 2 composite)
logic pixel;
assign pixel = (text | hires | (mix & mixed_line)) ? hrpixel : lrpixel; 

logic [1:0] cq = 0; // NTSC video
always_ff @(posedge clk_25mhz) begin
	cq <= nl ? 2'b00 : {~cq[0], cq[1]};
end

logic cp0, cp1, cp2, cp3;
assign {cp0, cp1, cp2, cp3} = {cq[0], ~cq[1], ~cq[0], cq[1]}; // phase 0, 90, 180, 270


logic [2:0] pix;
always_ff @(posedge clk_25mhz) begin
	pix <= {pix[1:0], pixel};
end

logic [3:0] cpix;
assign cpix = (text | ~color) ? 0 : {pix, pixel} ; // color pixels

// Apple 2 NTSC uses YIQ color space
logic Y;
logic [2:0] I;
logic [2:0] Q;
assign Y = pix[1];
assign I = {2'b0, cp3^cpix[0]} + {2'b0, cp0^cpix[1]} + {2'b0, cp1^cpix[2]} + {2'b0, cp2^cpix[3]};
assign Q = {2'b0, cp0^cpix[0]} + {2'b0, cp1^cpix[1]} + {2'b0, cp2^cpix[2]} + {2'b0, cp3^cpix[3]};

logic [11:0] apple_vid;

// convert YIQ color space to modern RGB for VGA
color_rom COLOR (.clk(clk_25mhz), .color_addr({Y, I, Q}), .color_data_out(apple_vid));
	
assign video = apple_vid;

endmodule