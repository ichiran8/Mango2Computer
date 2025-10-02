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