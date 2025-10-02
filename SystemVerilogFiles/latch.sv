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