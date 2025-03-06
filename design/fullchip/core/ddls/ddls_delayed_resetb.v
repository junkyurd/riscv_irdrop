`timescale 1ns / 1ps

module ddls_delayed_resetb # (
    parameter BUFFERSIZE = 4
)(
    input clk,
    input resetb,
    input [BUFFERSIZE - 1 : 0] delay_sel,
    output reg primary_resetb,
    output reg secondary_resetb,

    input clk_cnt_resetb,
    input riscv_clk_en,
    output reg riscv_clk_en_primary,
    output reg riscv_clk_en_secondary

    //input riscv_stop_ctrl_resetb,
    //output reg riscv_stop_ctrl_resetb_primary,
    //output reg riscv_stop_ctrl_resetb_secondary
);

    reg resetb_reg [BUFFERSIZE - 1 : 0];
    reg clk_en_reg [BUFFERSIZE - 1 : 0];
    //reg riscv_stop_ctrl_resetb_reg [BUFFERSIZE - 1 : 0];
    integer i;

    always @ (posedge clk or negedge resetb) begin
        if (!resetb) begin
            for (i = 0; i < BUFFERSIZE; i = i+1) begin
                resetb_reg[i] <= resetb;
                //clk_en_reg[i] <= resetb;
                //riscv_stop_ctrl_resetb_reg[i] <= resetb;
            end
            primary_resetb <= resetb;
            //riscv_clk_en_primary <= 1'b0;
            //riscv_stop_ctrl_resetb_primary <= 1'b0;
            secondary_resetb <= resetb;
            //riscv_clk_en_secondary <= 1'b0;
            //riscv_stop_ctrl_resetb_secondary <= 1'b0;

        end else begin
            for (i = 1; i < BUFFERSIZE; i = i+1) begin
                resetb_reg[i] <= resetb_reg[i-1];
                //clk_en_reg[i] <= clk_en_reg[i-1];
                //riscv_stop_ctrl_resetb_reg[i] <= riscv_stop_ctrl_resetb_reg[i-1];
            end
            resetb_reg[0] <= resetb;
            //clk_en_reg[0] <= riscv_clk_en;
            primary_resetb <= resetb;
            //riscv_clk_en_primary <= riscv_clk_en;
            //riscv_stop_ctrl_resetb_primary <= riscv_stop_ctrl_resetb;
            //slave_resetb <= resetb;
            if (delay_sel == 1) begin
                secondary_resetb <= resetb;
                //riscv_clk_en_secondary <= riscv_clk_en;
                //riscv_stop_ctrl_resetb_secondary <= riscv_stop_ctrl_resetb;
            end else begin
                for (i = 1; i < BUFFERSIZE; i = i+1) begin
                    if (delay_sel == (1<<i)) begin
                        secondary_resetb <= resetb_reg[i-1];
                        //riscv_clk_en_secondary <= clk_en_reg[i-1];
                        //riscv_stop_ctrl_resetb_secondary <= riscv_stop_ctrl_resetb_reg[i-1];
                    end
                end
            end
        end
    end


    always @ (posedge clk or negedge clk_cnt_resetb) begin
        if (!clk_cnt_resetb) begin
            for (i = 0; i < BUFFERSIZE; i = i+1) begin
                //resetb_reg[i] <= resetb;
                clk_en_reg[i] <= clk_cnt_resetb;
                //riscv_stop_ctrl_resetb_reg[i] <= resetb;
            end
            //primary_resetb <= resetb;
            riscv_clk_en_primary <= 1'b0;
            //riscv_stop_ctrl_resetb_primary <= 1'b0;
            //secondary_resetb <= resetb;
            riscv_clk_en_secondary <= 1'b0;
            //riscv_stop_ctrl_resetb_secondary <= 1'b0;

        end else begin
            for (i = 1; i < BUFFERSIZE; i = i+1) begin
                //resetb_reg[i] <= resetb_reg[i-1];
                clk_en_reg[i] <= clk_en_reg[i-1];
                //riscv_stop_ctrl_resetb_reg[i] <= riscv_stop_ctrl_resetb_reg[i-1];
            end
            //resetb_reg[0] <= resetb;
            clk_en_reg[0] <= riscv_clk_en;
            //primary_resetb <= resetb;
            riscv_clk_en_primary <= riscv_clk_en;
            //riscv_stop_ctrl_resetb_primary <= riscv_stop_ctrl_resetb;
            //slave_resetb <= resetb;
            if (delay_sel == 1) begin
                //secondary_resetb <= resetb;
                riscv_clk_en_secondary <= riscv_clk_en;
                //riscv_stop_ctrl_resetb_secondary <= riscv_stop_ctrl_resetb;
            end else begin
                for (i = 1; i < BUFFERSIZE; i = i+1) begin
                    if (delay_sel == (1<<i)) begin
                        //secondary_resetb <= resetb_reg[i-1];
                        riscv_clk_en_secondary <= clk_en_reg[i-1];
                        //riscv_stop_ctrl_resetb_secondary <= riscv_stop_ctrl_resetb_reg[i-1];
                    end
                end
            end
        end
    end



endmodule