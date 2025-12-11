// `timescale 1ns / 10ps
// ////////////////////////////////////////////////////////////////////////////////// 
// //////////////////////////////////////////////////////////////////////////////////

// module rcv(
//     input  logic        clk,
//     input  logic        kclk,
//     input  logic        nrst,
//     input  logic        kdata,
//     output logic [15:0] keycode,
//     output logic        oflag
// );

//   // Debounced PS/2 clock/data
//   logic kclkf, kdataf;
//   logic [15:0] keycode_next;

//   // Shifted bytes and control
//   logic [7:0] datacur  = '0;
//   logic [7:0] dataprev = '0;
//   logic [3:0] cnt      = '0;   // counts 0..10
//   logic       flag     = 1'b0; // strobes high when a full byte is ready
//   logic       pflag    = 1'b0; // previous flag (for edge detect)

//   // NEW: remember last low byte seen with Shift prefix, to detect change
//   logic [7:0] last_low_after_shift = 8'h00;

//   // Debouncers (assumes debouncer is available as a separate module)
//   debouncer #(.COUNT_MAX(19), .COUNT_WIDTH(5)) db_clk  (.clk(clk), .nrst(nrst), .I(kclk),  .O(kclkf));
//   debouncer #(.COUNT_MAX(19), .COUNT_WIDTH(5)) db_data (.clk(clk), .nrst(nrst), .I(kdata), .O(kdataf));

//   // Sample bits on the falling edge of the debounced PS/2 clock
//   // cnt: 0 = start, 1..8 = data bits, 9 = parity (use as 'flag'), 10 = stop
// always_ff @(negedge kclkf or negedge nrst) begin
//   if (!nrst) begin
//     cnt     <= 4'd0;
//     flag    <= 1'b0;
//     datacur <= '0;
//   end else begin
//     unique case (cnt)
//       4'd0:  /* start bit */ ;
//       4'd1:  datacur[0] <= kdataf;
//       4'd2:  datacur[1] <= kdataf;
//       4'd3:  datacur[2] <= kdataf;
//       4'd4:  datacur[3] <= kdataf;
//       4'd5:  datacur[4] <= kdataf;
//       4'd6:  datacur[5] <= kdataf;
//       4'd7:  datacur[6] <= kdataf;
//       4'd8:  datacur[7] <= kdataf;
//       4'd9:  flag       <= 1'b1; // byte ready
//       4'd10: flag       <= 1'b0; // clear strobe
//     endcase

//     if (cnt <= 4'd9) cnt <= cnt + 4'd1;
//     else             cnt <= 4'd0;
//   end
// end
// //  // Pair the previous and current data bytes
// //  always_comb begin
// //    keycode_next = {dataprev, datacur};
// //  end

// //  // Latch on rising edge of 'flag'.
// //  // If keycode[15:8] == 0x12 (Shift), only assert oflag when keycode[7:0] changed.
// //  always_ff @(posedge clk) begin
// //    if (flag && !pflag) begin
// //      // Check if the prefix/high byte is Shift (0x12), as per your logic
// //      if ((dataprev == 8'h12)| (dataprev == 8'h59)) begin
// //        if (datacur != last_low_after_shift) begin
// //          keycode              <= keycode_next;
// //          oflag                <= 1'b1;               // fire only on change
// //          last_low_after_shift <= datacur;            // remember last low
// //        end else begin
// //          oflag <= 1'b0;                              // suppress duplicate
// //        end
// //      end else begin
// //        // Normal path (no Shift prefix): behave as before
// //        keycode  <= keycode_next;
// //        oflag    <= 1'b1;
// //      end

// //      // Keep your rolling previous byte
// //      dataprev <= datacur;
// //    end else begin
// //      oflag <= 1'b0;
// //    end
// //    pflag <= flag;
// //  end

//   logic pflag_next, oflag_next;
//   logic [15:0] keycode1;
//   logic [7:0] dataprev_next, todecode, todecode_next;
  
//    typedef enum logic[3:0] {IDLE,DECODE,NORMAL,SHIFT,WAITS,DECODESHIFT,SEND,KEYUP, KEYUP2,EXT} state_type;
//    state_type state, next_state;
//    logic oflag1;

//   always_ff @(posedge clk or negedge nrst) begin
//     if(nrst == 0) begin
//       oflag1 <= '0;
//       pflag <= '0;
//       keycode1 <= '0;
//       state <= IDLE;
//       todecode <= '0;
//       dataprev <= '0;
//     end else begin
//       oflag1 <= oflag_next;
//       pflag <= pflag_next;
//       keycode1 <= keycode_next;
//       state <= next_state;
//       dataprev <= dataprev_next;
//       todecode <= todecode_next;
//     end
//   end


// logic flag_sync1, flag_sync2;
// always_ff @(posedge clk or negedge nrst) begin
//   if (!nrst) begin
//     flag_sync1 <= 0; flag_sync2 <= 0;
//   end else begin
//     flag_sync1 <= flag;         // from kclkf domain
//     flag_sync2 <= flag_sync1;
//   end
// end

// wire flag_rise = flag_sync1 & ~flag_sync2;  // use this instead of (flag && !pflag)

//   always_comb begin
//     oflag_next   = oflag;
//     pflag_next   = flag;
//     keycode_next = keycode1;
//     dataprev_next = dataprev;
//     if(flag_rise) begin
//       keycode_next  =  {dataprev, datacur};
//       dataprev_next =  datacur;
//       oflag_next = (dataprev != 8'hf0 && datacur != 8'hf0);
//     end else begin
//       oflag_next = 1'b0;
//     end
//   end
  
//   //FSM IMPLEMENTATION (NO BREAK SEQUENCES JUST ONE FLAG ONCE WE RECIEVED OUR DATA)
//   logic upkey;
//   assign upkey = (keycode1[15:8] == 8'hf0); //aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
  
//   logic shiftany;
//   logic extended;
  
  
//   always_comb begin
//   next_state = state;
//   case(state) 
//   IDLE: next_state = oflag1? DECODE : IDLE;
//   DECODE: begin
//     if(shift) begin
//         next_state = WAITS;
//     end else if (extended) begin
//         next_state = EXT;
//     end else begin
//         next_state = NORMAL;
//     end
//   end
//   EXT: begin
//     if(flag) begin
//         next_state = DECODE;
//     end
//   end
//   NORMAL: next_state = KEYUP;
//   KEYUP: begin
//     if(upkey) begin
//         next_state = IDLE;
//     end
//   end
//   WAITS:begin
//     if(upseq) begin
//       next_state = IDLE;
//     end else if(oflag1 && !upseq && shiftany) begin
//       next_state = DECODESHIFT;
//     end
//   end
//   DECODESHIFT: next_state = SEND;
//   SEND: next_state = KEYUP2;
//   KEYUP2: begin
//     if(upkey && shiftany) begin
//         next_state = KEYUP;
//     end
//   end
//   default: next_state = state;
//   endcase
// end

// assign extended = (keycode1[7:0] == 8'he0);
// assign shiftany = (keycode1[7:0] != 8'h12 && keycode1[7:0] != 8'h59);
// assign shift = (keycode1[7:0] == 8'h12 || keycode1[7:0] == 8'h59) &&(keycode1[15:8] != 8'hf0)? 1'b1: 1'b0;
// assign upseq = (keycode1 == 16'hf012 || keycode1 == 16'hf059)? 1'b1: 1'b0;
// logic codeflag;
// logic decodeshift;
// always_comb begin
//   todecode_next = todecode;
//   codeflag = 0;
//   decodeshift = 0;
//   case(state)
//   IDLE: begin
//   end
//   DECODE: begin
//     todecode_next = decoded;
//   end
//   NORMAL: begin
//     codeflag = 1;
//   end
//   WAITS: begin
//   end
//   KEYUP: begin
//   end
//   KEYUP2: begin
//   end
//   EXT: begin
//   end
//   DECODESHIFT: begin
//     decodeshift = 1;
//     todecode_next = decoded;
//   end
//   SEND: begin
//     codeflag = 1;
//   end
//   endcase
// end
    
  
  
  
  
// logic [7:0] decoded;
  
  
  
  
  
  
  
//   always_comb begin
//   case(keycode1[7:0])
//     8'h1c: decoded = 8'h41; // A
//     8'h32: decoded = 8'h42; // B
//     8'h21: decoded = 8'h43; // C
//     8'h23: decoded = 8'h44; // D
//     8'h24: decoded = 8'h45; // E
//     8'h2b: decoded = 8'h46; // F
//     8'h34: decoded = 8'h47; // G
//     8'h33: decoded = 8'h48; // H
//     8'h43: decoded = 8'h49; // I
//     8'h3b: decoded = 8'h4A; // J //to fix
//     8'h42: decoded = 8'h4B; // K
//     8'h4b: decoded = 8'h4C; // L
//     8'h3a: decoded = 8'h4D; // M
//     8'h31: decoded = 8'h4E; // N
//     8'h44: decoded = 8'h4F; // O
//     8'h4d: decoded = 8'h50; // P
//     8'h15: decoded = 8'h51; // Q
//     8'h2d: decoded = 8'h52; // R
//     8'h1b: decoded = 8'h53; // S
//     8'h2c: decoded = 8'h54; // T
//     8'h3c: decoded = 8'h55; // U
//     8'h2a: decoded = 8'h56; // V
//     8'h1d: decoded = 8'h57; // W
//     8'h22: decoded = 8'h58; // X
//     8'h35: decoded = 8'h59; // Y
//     8'h1a: decoded = 8'h5A; // Z
//     8'h54: decoded = 8'h5B; // [
//     8'h5d: decoded = 8'h5C; // \
//     8'h5b: decoded = 8'h5D; // ]
//     8'h36: begin
//       if (decodeshift) decoded = 8'h5E; // ^ (6 with shift)
//       else       decoded = 8'h36; // 6
//     end
//     8'h4e: begin
//       if (decodeshift) decoded = 8'h5F; // _
//       else       decoded = 8'h2D; // -
//     end
//     8'h29: decoded = 8'h20; // space
//     8'h16: begin
//       if (decodeshift) decoded = 8'h21; // !
//       else       decoded = 8'h31; // 1
//     end
//     8'h1E: begin
//       if (decodeshift) decoded = 8'h40; // @
//       else       decoded = 8'h32; // 2
//     end
    
//     8'h52: begin
//       if (decodeshift) decoded = 8'h22; // "
//       else       decoded = 8'h27; // '
//     end
//     8'h26: begin
//       if (decodeshift) decoded = 8'h23; // #
//       else       decoded = 8'h33; // 3
//     end
//     8'h25: begin
//       if (decodeshift) decoded = 8'h24; // $
//       else       decoded = 8'h34; // 4
//     end
//     8'h2e: begin
//       if (decodeshift) decoded = 8'h25; // %
//       else       decoded = 8'h35; // 5
//     end
//     8'h3d: begin
//       if (decodeshift) decoded = 8'h26; // &
//       else       decoded = 8'h37; // 7
//     end
//     8'h46: begin
//       if (decodeshift) decoded = 8'h28; // (
//       else       decoded = 8'h39; // 9
//     end
//     8'h45: begin
//       if (decodeshift) decoded = 8'h29; // )
//       else       decoded = 8'h30; // 0
//     end
//     8'h3e: begin
//       if (decodeshift) decoded = 8'h2A; // *
//       else       decoded = 8'h38; // 8
//     end
//     8'h55: begin
//       if (decodeshift) decoded = 8'h2B; // +
//       else       decoded = 8'h3D; // =
//     end
//     8'h41: begin
//       if (decodeshift) decoded = 8'h3C; // <
//       else       decoded = 8'h2C; // ,
//     end
//     8'h49: begin
//       if (decodeshift) decoded = 8'h3E; // >
//       else       decoded = 8'h2E; // .
//     end
//     8'h4a: begin
//       if (decodeshift) decoded = 8'h3F; // ?
//       else       decoded = 8'h2F; // /
//     end
//     8'h4c: begin
//       if (decodeshift) decoded = 8'h3A; // :
//       else       decoded = 8'h3B; // ;
//     end
//     8'h66: decoded = 8'h7F; // del
//     8'h11: decoded = 8'h01; // alt (no ASCII)
//     8'h5a: decoded = 8'h0D; // enter
//     8'h76: decoded = 8'h1B; // esc
//     8'h59: decoded = 8'h02; // shift (no ASCII)
//     8'h12: decoded = 8'h02; // shift (no ASCII)
//     8'h6b: decoded = 8'h08; //left 
//     //8'he0: decoded = 8'haa;
//     8'h74: decoded = 8'h15;
//     default: decoded = 8'h00;
//   endcase
// end

// assign keycode = {8'b0, todecode};
// assign oflag = codeflag;


// endmodule

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
 
module rcv(
    input  logic        clk,
    input  logic        kclk,
    input  logic        nrst,
    input  logic        kdata,
    output logic [15:0] keycode,
    output logic        oflag
);
 
  // Debounced Ps/2 clock/data
  logic kclkf, kdataf;
  logic [15:0] keycode_next;
 
  logic [7:0] datacur  = '0;
  logic [7:0] dataprev = '0;
  logic [3:0] cnt      = '0;
  logic       flag     = 1'b0;
  logic       pflag    = 1'b0;
 
  logic [7:0] last_low_after_shift = 8'h00;
 
  // Debouncers (assumes debouncer is available as a separate module)
  debouncer #(.COUNT_MAX(19), .COUNT_WIDTH(5)) db_clk  (.clk(clk), .nrst(nrst), .I(kclk),  .O(kclkf));
  debouncer #(.COUNT_MAX(19), .COUNT_WIDTH(5)) db_data (.clk(clk), .nrst(nrst), .I(kdata), .O(kdataf));
 
always_ff @(negedge kclkf or negedge nrst) begin
  if (!nrst) begin
    cnt     <= 4'd0;
    flag    <= 1'b0;
    datacur <= '0;
  end else begin
    unique case (cnt)
      4'd0: ;
      4'd1:  datacur[0] <= kdataf;
      4'd2:  datacur[1] <= kdataf;
      4'd3:  datacur[2] <= kdataf;
      4'd4:  datacur[3] <= kdataf;
      4'd5:  datacur[4] <= kdataf;
      4'd6:  datacur[5] <= kdataf;
      4'd7:  datacur[6] <= kdataf;
      4'd8:  datacur[7] <= kdataf;
      4'd9:  flag       <= 1'b1; // byte ready
      4'd10: flag       <= 1'b0; // clear
    endcase
 
    if (cnt <= 4'd9) cnt <= cnt + 4'd1;
    else             cnt <= 4'd0;
  end
end
 
  logic pflag_next, oflag_next;
  logic [15:0] keycode1;
  logic [7:0] dataprev_next, todecode, todecode_next;
 
   typedef enum logic[3:0] {IDLE,DECODE,NORMAL,SHIFT,WAITS,DECODESHIFT,SEND,KEYUP, KEYUP2,EXT,CWAIT,DECODECNTRL} state_type;
   state_type state, next_state;
   logic oflag1;
 
  always_ff @(posedge clk or negedge nrst) begin
    if(nrst == 0) begin
      oflag1 <= '0;
      pflag <= '0;
      keycode1 <= '0;
      state <= IDLE;
      todecode <= '0;
      dataprev <= '0;
    end else begin
      oflag1 <= oflag_next;
      pflag <= pflag_next;
      keycode1 <= keycode_next;
      state <= next_state;
      dataprev <= dataprev_next;
      todecode <= todecode_next;
    end
  end
 
 
logic flag_sync1, flag_sync2;
always_ff @(posedge clk or negedge nrst) begin
  if (!nrst) begin
    flag_sync1 <= 0; flag_sync2 <= 0;
  end else begin
    flag_sync1 <= flag;         // from kclkf domain
    flag_sync2 <= flag_sync1;
  end
end
 
wire flag_rise = flag_sync1 & ~flag_sync2;  // use this instead of (flag && !pflag)
 
  always_comb begin
    oflag_next   = oflag;
    pflag_next   = flag;
    keycode_next = keycode1;
    dataprev_next = dataprev;
    if(flag_rise) begin
      keycode_next  =  {dataprev, datacur};
      dataprev_next =  datacur;
      oflag_next = (dataprev != 8'hf0 && datacur != 8'hf0);
    end else begin
      oflag_next = 1'b0;
    end
  end
 
  logic upkey;
  assign upkey = (keycode1[15:8] == 8'hf0); //aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 
  logic shiftany;
  logic extended;
  logic ctrl,ctrlany,cntrlup;
 
 
  always_comb begin
  next_state = state;
  case(state)
  IDLE: next_state = oflag1? DECODE : IDLE;
  DECODE: begin
    if(shift) begin
        next_state = WAITS;
    end else if (extended) begin
        next_state = EXT;
    end else if(ctrl) begin
        next_state = CWAIT;
    end else begin
        next_state = NORMAL;
    end
  end
  EXT: begin
    if(flag) begin
        next_state = DECODE;
    end
  end
  NORMAL: next_state = IDLE;
  KEYUP: begin
    if(upkey) begin
        next_state = IDLE;
    end
  end
  WAITS:begin
    if(upseq) begin
      next_state = IDLE;
    end else if(oflag1 && !upseq && shiftany) begin
      next_state = DECODESHIFT;
    end
  end
  CWAIT:begin
    if(ctrlup) begin
      next_state = IDLE;
    end else if(oflag1 && !ctrlup && ctrlany) begin
      next_state = DECODECNTRL;
    end
  end
  DECODECNTRL:begin
    next_state = SEND;
  end
  DECODESHIFT: next_state = SEND;
  SEND: next_state = KEYUP2;
  KEYUP2: begin
    if(upkey && (shiftany||ctrlany)) begin
        next_state = KEYUP;
    end
  end
  default: next_state = state;
  endcase
end
 
assign extended = (keycode1[7:0] == 8'he0);
assign shiftany = (keycode1[7:0] != 8'h12 && keycode1[7:0] != 8'h59);
assign shift = (keycode1[7:0] == 8'h12 || keycode1[7:0] == 8'h59) &&(keycode1[15:8] != 8'hf0)? 1'b1: 1'b0;
assign upseq = (keycode1 == 16'hf012 || keycode1 == 16'hf059)? 1'b1: 1'b0;
assign ctrl = ((keycode1[7:0] == 8'h14) && (keycode1[15:8] != 8'hf0))? 1'b1:1'b0;
assign ctrlany = (keycode1[7:0] != 8'h14);
assign ctrlup = (keycode1[7:0] == 16'hf014);
 
 
logic codeflag;
logic decodeshift;
logic decodectrl;
always_comb begin
  todecode_next = todecode;
  codeflag = 0;
  decodeshift = 0;
  decodectrl = 0;
  case(state)
  IDLE: begin
  end
  DECODE: begin
    todecode_next = decoded;
  end
  NORMAL: begin
    codeflag = 1;
  end
  WAITS: begin
  end
  KEYUP: begin
  end
  KEYUP2: begin
  end
  EXT: begin
  end
  CWAIT:begin
  end
  DECODECNTRL:begin
    decodectrl = 1;
    todecode_next = ctrldecoded;
  end
  DECODESHIFT: begin
    decodeshift = 1;
    todecode_next = decoded;
  end
  SEND: begin
    codeflag = 1;
  end
  endcase
end
   
 
 
 
 
logic [7:0] decoded;
logic [7:0] ctrldecoded;
 
 
 
 
 
 
 
  always_comb begin
  ctrldecoded = '0;
  case(keycode1[7:0])
    8'h1c: decoded = 8'h41; // A
    8'h32: decoded = 8'h42; // B
    8'h21:decoded = 8'h43; //
    8'h23:decoded = 8'h44; // D
    8'h24:decoded = 8'h45; // E
    8'h2b:decoded = 8'h46; // F
    8'h34:decoded = 8'h47; // G
    8'h33:decoded = 8'h48; // H
    8'h43: decoded = 8'h49; // I
    8'h3b: decoded = 8'h4A; // J 
    8'h42: decoded = 8'h4B; // K
    8'h4b: decoded = 8'h4C; // L
    8'h3a: decoded = 8'h4D; // M
    8'h31: decoded = 8'h4E; // N
    8'h44: decoded = 8'h4F; // O
    8'h4d: decoded = 8'h50; // P
    8'h15: decoded = 8'h51; // Q
    8'h2d: decoded = 8'h52; // R
    8'h1b: decoded = 8'h53; // S
    8'h2c: decoded = 8'h54; // T
    8'h3c: decoded = 8'h55; // U
    8'h2a: decoded = 8'h56; // V
    8'h1d: decoded = 8'h57; // W
    8'h22:decoded = 8'h58; // X
    8'h35: decoded = 8'h59; // Y
    8'h1a: decoded = 8'h5A; // Z
    8'h54: decoded = 8'h5B; // [
    8'h5d: decoded = 8'h5C; // \
    8'h5b: decoded = 8'h5D; // ]
    8'h36: begin
      if (decodeshift) decoded = 8'h5E; // ^ (6 with shift)
      else       decoded = 8'h36; // 6
    end
    8'h4e: begin
      if (decodeshift) decoded = 8'h5F; // _
      else       decoded = 8'h2D; // -
    end
    8'h29: decoded = 8'h20; // space
    8'h16: begin
      if (decodeshift) decoded = 8'h21; // !
      else       decoded = 8'h31; // 1
    end
    8'h1E: begin
      if (decodeshift) decoded = 8'h40; // @
      else       decoded = 8'h32; // 2
    end
   
    8'h52: begin
      if (decodeshift) decoded = 8'h22; // "
      else       decoded = 8'h27; // '
    end
    8'h26: begin
      if (decodeshift) decoded = 8'h23; // #
      else       decoded = 8'h33; // 3
    end
    8'h25: begin
      if (decodeshift) decoded = 8'h24; // $
      else       decoded = 8'h34; // 4
    end
    8'h2e: begin
      if (decodeshift) decoded = 8'h25; // %
      else       decoded = 8'h35; // 5
    end
    8'h3d: begin
      if (decodeshift) decoded = 8'h26; // &
      else       decoded = 8'h37; // 7
    end
    8'h46: begin
      if (decodeshift) decoded = 8'h28; // (
      else       decoded = 8'h39; // 9
    end
    8'h45: begin
      if (decodeshift) decoded = 8'h29; // )
      else       decoded = 8'h30; // 0
    end
    8'h3e: begin
      if (decodeshift) decoded = 8'h2A; // *
      else       decoded = 8'h38; // 8
    end
    8'h55: begin
      if (decodeshift) decoded = 8'h2B; // +
      else       decoded = 8'h3D; // =
    end
    8'h41: begin
      if (decodeshift) decoded = 8'h3C; // <
      else       decoded = 8'h2C; // ,
    end
    8'h49: begin
      if (decodeshift) decoded = 8'h3E; // >
      else       decoded = 8'h2E; // .
    end
    8'h4a: begin
      if (decodeshift) decoded = 8'h3F; // ?
      else       decoded = 8'h2F; // /
    end
    8'h4c: begin
      if (decodeshift) decoded = 8'h3A; // :
      else       decoded = 8'h3B; // ;
    end
    8'h66: decoded = 8'h7F; // del
    8'h11: decoded = 8'h01; // alt (no ASCII)
    8'h5a: decoded = 8'h0D; // enter
    8'h76: decoded = 8'h1B; // esc
    8'h59: decoded = 8'h02; // shift (no ASCII)
    8'h12: decoded = 8'h02; // shift (no ASCII)
    8'h6b: decoded = 8'h08; //left
    //8'he0: decoded = 8'haa;
    8'h74: decoded = 8'h15;
    default: decoded = 8'h00;
  endcase
  if(decodectrl) begin
  ctrldecoded = decoded & 8'h1f;
  end
end
 
assign keycode = {8'b0, todecode};
assign oflag = codeflag;
 
 
endmodule