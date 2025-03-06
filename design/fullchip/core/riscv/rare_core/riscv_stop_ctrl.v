`timescale 1ns / 1ps

module riscv_stop_ctrl (
    input clk,
    input resetb,
    input ce,
    input we,
    input [31:0] addr,
    input [3:0] data,
    output wire riscv_ready_out,
    output wire riscv_debug_out    
);

    reg riscv_ready;
    reg riscv_debug;
    reg end_q1, end_q2, end_q3;
    assign riscv_debug_out = riscv_debug;
    
    always @ (posedge clk or negedge resetb) begin
        if (!resetb) begin
            riscv_ready <= 1'b0;
            riscv_debug <= 1'b0;
        end else if (ce) begin
            if (we) begin
                if (addr[29:0] == 30'b000000000000000000000111111111) begin
                    if (data == 4'b0101) begin
                        riscv_ready <= 1'b1;
                    end else if (data == 4'b1010) begin
                        riscv_debug <= ~riscv_debug;
                    end
                end
            end
        end
    end
    
    assign riscv_ready_out = end_q3;

    always @ (posedge clk or negedge resetb) begin
        if (!resetb) begin
            end_q1 <= 1'b0;
            end_q2 <= 1'b0;
            end_q3 <= 1'b0;
        end else begin
            end_q1 <= riscv_ready;
            end_q2 <= end_q1;
            end_q3 <= end_q2;
        end
    end

endmodule;