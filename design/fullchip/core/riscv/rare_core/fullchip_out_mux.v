`timescale 1ns / 1ps

module fullchip_out_mux (
    input a_write_read_sel, // if 0 AW selected, 1 AR selected
    input awready,
    input arready,
    output wire aready,

    input freq_out_sel, // 0 for clk_out_freq_out, 1 for odometer_vco_pad_out
    input clk_out_freq_out,
    input odometer_vco_pad_out,
    output wire freq_out,

    input debug_out_sel, // 0 for primary debug out, 1 for secondary debug out
    input riscv_debug_out_primary,
    input riscv_debug_out_secondary,
    output wire riscv_debug_out

);

    MUX2D2BWP30P140 aready_mux(
        .S(a_write_read_sel),
        .I0(awready),
        .I1(arready),
        .Z(aready)
    );

    CKMUX2D4BWP30P140 freq_out_mux(
        .S(freq_out_sel),
        .I0(clk_out_freq_out),
        .I1(odometer_vco_pad_out),
        .Z(freq_out)
    );

    MUX2D2BWP30P140 riscv_debug_mux(
        .S(debug_out_sel),
        .I0(riscv_debug_out_primary),
        .I1(riscv_debug_out_secondary),
        .Z(riscv_debug_out)
    );

endmodule