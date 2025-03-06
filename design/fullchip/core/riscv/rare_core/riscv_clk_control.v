`timescale 1ns / 1ps

module riscv_clk_control (
    input clk_in_dco,
    input clk_in_fpga,

    input resetb,
    input clk_cnt_resetb,
    input primary_resetb,
    input secondary_resetb,

    input clk_select, // select between dco generated and fpga
                      // 0: DCO 1: FPGA
    
    input freq_out_en,

    input freq_out_cnt_clk_select,
    input freq_out_cnt_resetb,
    input freq_out_cnt_start,
    output reg freq_out_cnt_done,

    input clk_en_in_primary,
    input clk_en_in_secondary,
    input riscv_ready_primary,
    input riscv_ready_secondary,


    input [63:0] riscv_run_cycle,
    input riscv_run_cycle_enable,
    output wire riscv_run_done,
    output reg riscv_run_done_primary,
    output reg riscv_run_done_secondary,

    output reg clk_out_riscv_en_primary,
    output reg clk_out_riscv_en_secondary,

    output wire clk_out_system,
    //output wire clk_out_riscv_primary,
    //output wire clk_out_riscv_secondary,

    output wire clk_out_freq_out,

    output reg [63:0] core_cycle_count_primary, // total number of cycles core ran since reset
    output reg [63:0] core_cycle_count_secondary 
);
    
    reg [63:0] run_cnt_primary; // number of cycles core ran for this clk_en run
    reg [63:0] run_cnt_secondary;

    reg riscv_ready_primary_reg;
    reg riscv_ready_secondary_reg;
    //reg riscv_run_done_primary;
    //reg riscv_run_done_secondary;
    
    assign riscv_run_done = riscv_run_done_primary && riscv_run_done_secondary;

    wire clk_out_temp; // muxed signal select between dco and fpga clock
    wire clk_in_divided;
    wire clk_out_freq_out_cnt;

    always @ (posedge clk_out_temp or negedge clk_cnt_resetb) begin
        if (!clk_cnt_resetb) begin
            run_cnt_primary <= 0;
            riscv_run_done_primary <= 1'b0;
            clk_out_riscv_en_primary <= 1'b0;
            core_cycle_count_primary <= 0;
            riscv_ready_primary_reg <= 1'b0;
        end else begin
            if (clk_en_in_primary) begin
                if (riscv_ready_primary) begin
                    riscv_ready_primary_reg <= 1'b1;
                    clk_out_riscv_en_primary <= 1'b0;
                    riscv_run_done_primary <= 1'b1;
                    core_cycle_count_primary <= core_cycle_count_primary;
                end else if (riscv_ready_primary_reg) begin
                    clk_out_riscv_en_primary <= 1'b0;
                    riscv_run_done_primary <= 1'b1;
                    core_cycle_count_primary <= core_cycle_count_primary;
                end else if (riscv_run_cycle_enable) begin
                    if (run_cnt_primary == riscv_run_cycle) begin
                        clk_out_riscv_en_primary <= 1'b0;
                        riscv_run_done_primary <= 1'b1;
                        core_cycle_count_primary <= core_cycle_count_primary;
                    end else if (primary_resetb) begin
                        run_cnt_primary <= run_cnt_primary + 1'b1;
                        clk_out_riscv_en_primary <= 1'b1;
                        core_cycle_count_primary <= core_cycle_count_primary + 1'b1;
                    end else begin
                        clk_out_riscv_en_primary <= 1'b1;
                    end
                end else begin
                    if (primary_resetb) begin
                        clk_out_riscv_en_primary <= 1'b1;
                        core_cycle_count_primary <= core_cycle_count_primary + 1'b1;
                    end else begin
                        clk_out_riscv_en_primary <= 1'b1;
                        core_cycle_count_primary <= core_cycle_count_primary;
                    end
                end
            end else begin
                run_cnt_primary <= 0;
                clk_out_riscv_en_primary <= 1'b0;
                riscv_run_done_primary <= 1'b0;
                core_cycle_count_primary <= core_cycle_count_primary;
                riscv_ready_primary_reg <= 1'b0;
            end
        end
    end

    always @ (posedge clk_out_temp or negedge clk_cnt_resetb) begin
        if (!clk_cnt_resetb) begin
            run_cnt_secondary <= 0;
            riscv_run_done_secondary <= 1'b0;
            clk_out_riscv_en_secondary <= 1'b0;
            core_cycle_count_secondary <= 0;
            riscv_ready_secondary_reg <= 1'b0;
        end else begin
            if (clk_en_in_secondary) begin
                if (riscv_ready_secondary) begin                
                    riscv_ready_secondary_reg <= 1'b1;
                    clk_out_riscv_en_secondary <= 1'b0;
                    riscv_run_done_secondary <= 1'b1;
                    core_cycle_count_secondary <= core_cycle_count_secondary;
                end else if (riscv_ready_secondary_reg) begin
                    clk_out_riscv_en_secondary <= 1'b0;
                    riscv_run_done_secondary <= 1'b1;
                    core_cycle_count_secondary <= core_cycle_count_secondary;
                end else if (riscv_run_cycle_enable) begin
                    if (run_cnt_secondary == riscv_run_cycle) begin
                        clk_out_riscv_en_secondary <= 1'b0;
                        riscv_run_done_secondary <= 1'b1;
                        core_cycle_count_secondary <= core_cycle_count_secondary;
                    end else if (secondary_resetb) begin
                        run_cnt_secondary <= run_cnt_secondary + 1'b1;
                        clk_out_riscv_en_secondary <= 1'b1;
                        core_cycle_count_secondary <= core_cycle_count_secondary + 1'b1;
                    end else begin
                        clk_out_riscv_en_secondary <= 1'b1;
                    end
                end else begin
                    if (secondary_resetb) begin
                        clk_out_riscv_en_secondary <= 1'b1;
                        core_cycle_count_secondary <= core_cycle_count_secondary + 1'b1;
                    end else begin
                        clk_out_riscv_en_secondary <= 1'b1;
                        core_cycle_count_secondary <= core_cycle_count_secondary;
                    end
                end
            end else begin
                run_cnt_secondary <= 0;
                clk_out_riscv_en_secondary <= 1'b0;
                riscv_run_done_secondary <= 1'b0;
                core_cycle_count_secondary <= core_cycle_count_secondary;
                riscv_ready_secondary_reg <= 1'b0;
            end
        end
    end

    /*
    rare_clk_riscv_and clk_riscv_and_primary (
        .clk_out_riscv_en(clk_out_riscv_en_primary),
        .clk_in(clk_out_temp),
        .clk_out(clk_out_riscv_primary)
    );

    rare_clk_riscv_and clk_riscv_and_secondary (
        .clk_out_riscv_en(clk_out_riscv_en_secondary),
        .clk_in(clk_out_temp),
        .clk_out(clk_out_riscv_secondary)
    );
    */

    // CLK MUX, selecting between external FPGA clk or generated DCO clk
    rare_clk_mux clk_select_mux_inst(
        .clk_select(clk_select),
        .clk_in_dco(clk_in_dco),
        .clk_in_fpga(clk_in_fpga),
        .clk_out_temp(clk_out_temp),

        .freq_out_cnt_clk_select(freq_out_cnt_clk_select),
        .clk_in_divided(clk_in_divided),
        .clk_out_freq_out_cnt(clk_out_freq_out_cnt)
    );


    
    

// --------------------------------------------------------------------
// 00000 0000  00000  000         000  0   0 00000 
// 0     0   0 0     0   0       0   0 0   0   0   
// 00000 0000  00000 0 0 0       0   0 0   0   0         
// 0     0   0 0     0  00       0   0 0   0   0
// 0     0   0 00000  0000        000   000    0
// --------------------------------------------------------------------
    reg [15:0] freq_out_cnt;

    reg [1:0] freq_out_cnt_start_reg;

    always @ (posedge clk_out_freq_out_cnt or negedge resetb) begin
        if (!resetb) begin
            freq_out_cnt_start_reg <= 0;
        end else begin
            freq_out_cnt_start_reg <= {freq_out_cnt_start_reg[0], freq_out_cnt_start};
        end
    end // synchronizer

    always @ (posedge clk_out_freq_out_cnt or negedge freq_out_cnt_resetb) begin
        if (!freq_out_cnt_resetb) begin
            freq_out_cnt <= 0;
            freq_out_cnt_done <= 1'b0;
        end else if (freq_out_cnt_start_reg[1] && !freq_out_cnt_done) begin
            if (freq_out_cnt == 16'b1111111111111111) begin
                freq_out_cnt_done <= 1'b1;
            end else begin
                freq_out_cnt <= freq_out_cnt + 1'b1;
            end
        end
    end   
    
    reg clk_dco_x2;
    // 1st stage X2
    always @ (posedge clk_in_dco or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x2 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x2 <= ~clk_dco_x2;
        end
    end

    reg clk_dco_x4;
    // 2nd stage x4
    always @ (posedge clk_dco_x2 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x4 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x4 <= ~clk_dco_x4;
        end
    end

    reg clk_dco_x8;
    // 3rd stage x8
    always @ (posedge clk_dco_x4 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x8 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x8 <= ~clk_dco_x8;
        end
    end

    reg clk_dco_x16;
    // 4th stage x16
    always @ (posedge clk_dco_x8 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x16 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x16 <= ~clk_dco_x16;
        end
    end

    reg clk_dco_x32;
    // 5th stage x32
    always @ (posedge clk_dco_x16 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x32 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x32 <= ~clk_dco_x32;
        end
    end

    reg clk_dco_x64;
    // 6th stage x64
    always @ (posedge clk_dco_x32 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x64 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x64 <= ~clk_dco_x64;
        end
    end

    reg clk_dco_x128;
    // 7th stage x128
    always @ (posedge clk_dco_x64 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x128 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x128 <= ~clk_dco_x128;
        end
    end

    reg clk_dco_x256;
    // 8th stage x256
    always @ (posedge clk_dco_x128 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x256 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x256 <= ~clk_dco_x256;
        end
    end

    reg clk_dco_x512;
    // 9th stage x512
    always @ (posedge clk_dco_x256 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x512 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x512 <= ~clk_dco_x512;
        end
    end

    reg clk_dco_x1024;
    // 10th stage x1024
    always @ (posedge clk_dco_x512 or negedge resetb) begin
        if (!resetb) begin
            clk_dco_x1024 <= 1'b0;
        end else if (freq_out_en) begin
            clk_dco_x1024 <= ~clk_dco_x1024;
        end
    end

    assign clk_out_freq_out = clk_dco_x1024;
    
    assign clk_in_divided = clk_dco_x32;

    assign clk_out_system = clk_out_temp;




endmodule