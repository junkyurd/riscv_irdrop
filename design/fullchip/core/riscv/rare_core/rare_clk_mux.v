`timescale 1ns / 1ps

module rare_clk_mux (
    input clk_select,
    input clk_in_dco,
    input clk_in_fpga,
    output wire clk_out_temp,

    input freq_out_cnt_clk_select,
    input clk_in_divided,
    output wire clk_out_freq_out_cnt
);

    CKMUX2D4BWP30P140 main_clk_select_mux(
        .S(clk_select),
        .I0(clk_in_dco),
        .I1(clk_in_fpga),
        .Z(clk_out_temp)
    );

    CKMUX2D4BWP30P140 freq_out_clk_select_mux(
        .S(freq_out_cnt_clk_select),
        .I0(clk_in_dco),
        .I1(clk_in_divided),
        .Z(clk_out_freq_out_cnt)
    );

endmodule