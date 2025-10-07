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