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
