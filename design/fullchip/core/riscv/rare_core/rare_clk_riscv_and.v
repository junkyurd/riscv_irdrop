`timescale 1ns/1ps

module rare_clk_riscv_and(
    input wire clk_out_riscv_en,
    input wire clk_in,
    output wire clk_out
);

    CKAN2D4BWP30P140 clk_riscv_and(
        .A1(clk_in),
        .A2(clk_out_riscv_en),
        .Z(clk_out)
    );

endmodule