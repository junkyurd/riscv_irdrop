`timescale 1ns / 1ps

module ddls_decoder # (
    parameter INPUTSIZE = 2,
    parameter BUFFERSIZE = 4
)(
    //input clk,
    input [INPUTSIZE - 1 : 0] delay_sel_in,
    output reg [BUFFERSIZE - 1 : 0] delay_sel_out
);

    integer i;

    always @ (*) begin
        for (i = 0; i < BUFFERSIZE; i = i+1) begin
            if (delay_sel_in == i) begin
                delay_sel_out[i] <= 1'b1;
            end else begin
                delay_sel_out[i] <= 1'b0;
            end
        end
    end

endmodule