`timescale 1ns / 1ps

module rare_sram_mux # (
    parameter data_width = 32
)(
    // From stream
    input wire stream_clk,
    input wire stream_enable,
    input wire [31:0] stream_addr,
    input wire [data_width - 1 : 0] stream_data_in,
    output wire [data_width - 1 : 0] stream_data_out,
    input wire stream_we,
    input wire stream_en,

    // From RISC-V
    input wire core_clk,
    input wire [31:0] core_addr,
    input wire [data_width - 1 : 0] core_data_in,
    output wire [data_width - 1 : 0] core_data_out,
    input wire core_we,
    input wire core_en,
    input wire [31:0] core_beb,

    // To Memory
    output wire mem_clk,
    output wire [31:0] mem_addr,
    output wire [data_width - 1 : 0] mem_data_in,
    input wire [data_width - 1 : 0] mem_data_out,
    output wire mem_we,
    output wire mem_en,
    output wire [31:0] mem_beb
);
    // CLK MUX
    CKMUX2D2BWP30P140 mem_clk_mux(
        .S(stream_enable),
        .I0(core_clk),
        .I1(stream_clk),
        .Z(mem_clk)
    );

    // General MUX
    MUX2D2BWP30P140 mem_we_mux(
        .S(stream_enable),
        .I0(core_we),
        .I1(stream_we),
        .Z(mem_we)
    );

    MUX2D2BWP30P140 mem_en_mux(
        .S(stream_enable),
        .I0(core_en),
        .I1(stream_en),
        .Z(mem_en)
    );

    genvar i;
    generate
        for (i = 0; i < 32; i = i+1) begin
            MUX2D2BWP30P140 mem_addr_mux(
                .S(stream_enable),
                .I0(core_addr[i]),
                .I1(stream_addr[i]),
                .Z(mem_addr[i])
            );            

            MUX2D2BWP30P140 mem_be_mux(
                .S(stream_enable),
                .I0(core_beb[i]),
                .I1(1'b0),
                .Z(mem_beb[i])
            );
        end
    endgenerate

    generate
        for (i = 0; i < data_width; i = i+1) begin
            MUX2D2BWP30P140 mem_data_in_mux(
                .S(stream_enable),
                .I0(core_data_in[i]),
                .I1(stream_data_in[i]),
                .Z(mem_data_in[i])
            );
            
            MUX2D2BWP30P140 stream_data_out_mux(
                .S(stream_enable),
                .I0(1'b0),
                .I1(mem_data_out[i]),
                .Z(stream_data_out[i])
            );

            MUX2D2BWP30P140 core_data_out_mux(
                .S(stream_enable),
                .I0(mem_data_out[i]),
                .I1(1'b0),
                .Z(core_data_out[i])
            );
        end
    endgenerate
    
endmodule