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