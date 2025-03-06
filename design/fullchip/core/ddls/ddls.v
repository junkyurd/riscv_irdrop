`timescale 1ns / 1ps

module ddls # (
    parameter BUFFERSIZE = 4,
    parameter BUFFERWIDTH = 256
)(
    input clk,
    input resetb,
    
    input start,

    input [BUFFERWIDTH - 1 : 0] primary_data,
    input [BUFFERWIDTH - 1 : 0] secondary_data,
    input [BUFFERSIZE - 1 : 0] delay_sel,
    output reg result_flag,
    output reg [BUFFERWIDTH - 1 : 0] result
);
    integer i;

    reg [BUFFERWIDTH - 1 : 0] delayed_data [BUFFERSIZE - 1 : 0];
    reg [BUFFERWIDTH - 1 : 0] secondary_data_r; // register for slave_data to meet timing criteria
    reg [BUFFERWIDTH - 1 : 0] primary_data_r; // master_data from delayed_data ffs (based on delay_sel) to be compared with slave_data_r
    

    always @ (posedge clk) begin
        if (!resetb) begin
            for (i = 0; i < BUFFERSIZE; i = i+1) begin
                delayed_data[i] <= 'b0;
            end
        end else begin
            for (i = 1; i < BUFFERSIZE; i = i+1) begin
                delayed_data[i] <= delayed_data[i-1];
            end
            delayed_data[0] <= primary_data;
        end
    end

    always @ (posedge clk) begin
        if (!resetb) begin
            secondary_data_r <= 'b0;
            primary_data_r <= 'b0;
        end else begin
            secondary_data_r <= secondary_data;
            //master_data_r <= master_data;
            if (delay_sel == 1) begin
                primary_data_r <= primary_data;
            end else begin
                for (i = 1; i < BUFFERSIZE; i = i+1) begin
                    if (delay_sel == (1<<i)) begin
                        primary_data_r <= delayed_data[i-1];
                    end
                end
            end
        end
    end

    always @ (posedge clk) begin
        if (!resetb) begin
            result_flag <= 1'b0;
            result <= {BUFFERWIDTH{1'b0}};
        end else if (start) begin
            result_flag <= |({primary_data_r[180:0]} 
                            ^ {secondary_data_r[180:0]});
            result[180:0] <= ({primary_data_r[180:0]} 
                            ^ {secondary_data_r[180:0]});
            result[BUFFERWIDTH-1:181] <= primary_data_r[BUFFERWIDTH-1:181];
        end else begin
            result_flag <= 1'b0;
            result <= {BUFFERWIDTH{1'b0}};
        end
    end

endmodule