module clock_divider #(parameter DIV = 14) (
    input logic CLK, enable,
    output logic CPUCLK
);

logic [3:0] count = 0;
logic [3:0] next_count;
logic flag, next_cpuclk;
logic cpu_clk_int = 0;

assign CPUCLK = cpu_clk_int;

always_ff @(posedge CLK) begin
    count <= next_count;
    cpu_clk_int <= (flag) ? next_cpuclk : cpu_clk_int;//(flag) ? !cpu_clk_int : cpu_clk_int;
end

always_comb begin
    next_count = count;
    flag = 1'b0;
    next_cpuclk = cpu_clk_int;
    if(enable) begin
        if(count + 1 == DIV / 2) begin
            next_count = '0;
            flag = 1'b1;
            next_cpuclk = !cpu_clk_int;
        end else begin
            next_count = count + 1;
            flag = 1'b0;
            next_cpuclk = cpu_clk_int;
        end
    end

end

endmodule