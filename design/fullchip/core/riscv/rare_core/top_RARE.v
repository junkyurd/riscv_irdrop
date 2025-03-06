`timescale 1ns / 1ps

module top_RARE # (
    // RISC-V
    parameter COREV_PULP = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1,

    // delayed dual lockstep
    parameter INPUTSIZE = 4,
    parameter BUFFERSIZE = 16,
    parameter BUFFERWIDTH = 256, // want to make BUFFERWIDTH multiple of 32

    // Heater paramters
    //parameter NUM_HEATER_TOP = 0, // actual heater count will be 8 * C_S_AXI_DATA_WIDTH
    parameter NUM_HEATER_P = 40,
    parameter NUM_HEATER_S = 40,

    //parameter NUM_TEMP_SENSOR_TOP = 0,
    parameter NUM_TEMP_SENSOR_PRIMARY = 32,
    parameter NUM_TEMP_SENSOR_SECONDARY = 32,

    // Odometer parameters
    parameter NUM_ODOMETER_P = 100,
    parameter NUM_ODOMETER_S = 100,
    parameter NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER_P+NUM_ODOMETER_S+1)
)(
    input stream_clk,    
    input resetb,
    input riscv_resetb,

    // Clock control
    input clk_select,
    input freq_out_en,

    input dco_en,
    input [3:0] dco_coarse,
    input [3:0] dco_fine,

    input freq_out_cnt_clk_select,
    input freq_out_cnt_resetb,
    input freq_out_cnt_start,
    output wire freq_out_cnt_done,
    output wire clk_out_freq_out,

    // DDLS stream
    input [31:0] stream_addr_ddls,
    input [BUFFERWIDTH - 1 : 0] stream_data_in_ddls,
    output reg [BUFFERWIDTH - 1 : 0] stream_data_out_ddls,
    input stream_we_ddls,
    input stream_en_ddls,
    
    input [1:0] stream_select,  // decide which one to stream
                                // 01: primary only
                                // 10: secondary only
                                // 11: write to both, but no read
    input fetch_enable,

    input riscv_clk_en,
    input [63:0] riscv_run_cycle,
    input wire riscv_run_cycle_enable,
    output wire riscv_run_done,
    output wire riscv_run_done_primary,
    output wire riscv_run_done_secondary,

    output reg [63:0] core_cycle_count_primary_reg,
    output reg [63:0] core_cycle_count_secondary_reg,

    output wire core_sleep_o_primary,
    output wire core_sleep_o_secondary,

    output wire riscv_err_overflow,
    output wire riscv_64_cnt_overflow,
    output wire [63:0] riscv_errcnt,

    // Instruction stream
    input [31:0] stream_addr_instr,
    input [31:0] stream_data_in_instr,
    output reg [31:0] stream_data_out_instr,
    input stream_we_instr,
    input stream_en_instr,

    // Data stream
    input [31:0] stream_addr_data,
    input [31:0] stream_data_in_data,
    output reg [31:0] stream_data_out_data,
    input stream_we_data,
    input stream_en_data,

    // RISC-V boot
    input [31:0] boot_addr_i,
    input [31:0] mtvec_addr_i,
    //input [31:0] dm_halt_addr_i,
    //input [31:0] hart_id_i,
    //input [31:0] dm_exception_addr_i,

    // delayed dual lockstep
    input [INPUTSIZE - 1 : 0] delay_sel_in,
    input ddls_resetb,

    // HEATER ports
    input wire heater_pwm_a,
    input wire heater_pwm_b,
    input wire heater_global_enable,
    //input wire [NUM_HEATER_TOP - 1 : 0] heater_enable_top,
    input wire [NUM_HEATER_P - 1 : 0] heater_enable_primary,
    input wire [NUM_HEATER_S - 1 : 0] heater_enable_secondary,
    input wire [NUM_TEMP_SENSOR_PRIMARY + NUM_TEMP_SENSOR_SECONDARY - 1 : 0] temp_sensor_decode,
    input wire temp_sensor_en,

    inout wire sense_hi,
    inout wire sense_lo,
    inout wire source_hi,
    inout wire source_lo,

    // Odometer ports
    input wire odometer_resetb,
    // number of odometers are NUM_ODOMETER_P + NUM_ODOMETER_S + 1 (DCO_odo)
    input wire [NUM_ODOMETER_LOG2 - 1 : 0] odometer_sel,
    input wire odometer_enable_primary,
    input wire odometer_enable_secondary,
    input wire odometer_enable_dco,


    input wire stress,
    input wire ac_dc,
    input wire sel_inv,
    input wire sel_nand,
    input wire sel_nor,
    input wire odometer_meas_trig,
    input wire odometer_load,
    output reg [15:0] bit_count,
    output wire odometer_vco_pad_out,

    // Debug ports
    output wire riscv_debug_out_primary,
    output wire riscv_debug_out_secondary,

    output reg [535:0] scan_out
    
);

// --------------------------------------------------------------------
//  000  0   0 0   0  000
// 0      0 0  00  0 0   0
//  000    0   0 0 0 0
//     0   0   0  00 0   0
//  000    0   0   0  000
// --------------------------------------------------------------------

    reg [1:0] riscv_resetb_reg;
    reg [1:0] ddls_resetb_reg;
    reg [1:0] riscv_clk_en_reg;
    reg [1:0] freq_out_cnt_start_reg;
    reg [1:0] freq_out_cnt_resetb_reg;

    wire clk_out_system;

    wire [63:0] core_cycle_count_primary;
    wire [63:0] core_cycle_count_secondary;

    always @ (posedge clk_out_system or negedge resetb) begin
        if (!resetb) begin
            riscv_resetb_reg <= 0;
            ddls_resetb_reg <= 0;
            riscv_clk_en_reg <= 0;
            freq_out_cnt_start_reg <= 0;
            freq_out_cnt_resetb_reg <= 0;
        end else begin
            riscv_resetb_reg <= {riscv_resetb_reg[0], riscv_resetb};
            ddls_resetb_reg <= {ddls_resetb_reg[0], ddls_resetb};
            riscv_clk_en_reg <= {riscv_clk_en_reg[0], riscv_clk_en};
            freq_out_cnt_start_reg <= {freq_out_cnt_start_reg[0], freq_out_cnt_start};
            freq_out_cnt_resetb_reg <= {freq_out_cnt_resetb_reg[0], freq_out_cnt_resetb};
        end
    end  

    // DCO
    // Generates internal clock for core and heater
    wire clk_i; // is a dco generated clock replace to actual
    wire rst_ni;

    // delayed dual lockstep signals
    wire [BUFFERSIZE - 1 : 0] delay_sel_out;
    wire primary_resetb;
    wire secondary_resetb;
    wire clk_en_in_primary;
    wire clk_en_in_secondary;
    wire clk_out_riscv_en_primary;
    wire clk_out_riscv_en_secondary;
    wire riscv_ready_out_primary;
    wire riscv_ready_out_secondary;

    // For now    
    assign rst_ni = resetb;

    riscv_clk_control riscv_clk_control_inst(
        .clk_in_dco(clk_i),
        .clk_in_fpga(stream_clk),

        .resetb(resetb),
        .primary_resetb(primary_resetb),
        .secondary_resetb(secondary_resetb),
        
        .clk_select(clk_select),
        .freq_out_en(freq_out_en),

        .freq_out_cnt_clk_select(freq_out_cnt_clk_select),
        .freq_out_cnt_resetb(freq_out_cnt_resetb_reg[1]),
        .freq_out_cnt_start(freq_out_cnt_start_reg[1]),
        .freq_out_cnt_done(freq_out_cnt_done),

        .clk_en_in_primary(clk_en_in_primary),
        .clk_en_in_secondary(clk_en_in_secondary),

        .riscv_run_cycle(riscv_run_cycle),
        .riscv_run_cycle_enable(riscv_run_cycle_enable),
        .riscv_run_done(riscv_run_done),
        .riscv_run_done_primary(riscv_run_done_primary),
        .riscv_run_done_secondary(riscv_run_done_secondary),
        .clk_out_riscv_en_primary(clk_out_riscv_en_primary),
        .clk_out_riscv_en_secondary(clk_out_riscv_en_secondary),
        
        .clk_out_system(clk_out_system),
        .clk_out_freq_out(clk_out_freq_out),

        .core_cycle_count_primary(core_cycle_count_primary),
        .core_cycle_count_secondary(core_cycle_count_secondary),

        .riscv_ready_primary(riscv_ready_out_primary),
        .riscv_ready_secondary(riscv_ready_out_secondary)
    );
 

    
    
    // heater_wrapper * () heater_inst (); // heater unit to be placed in its own power domain

// --------------------------------------------------------------------
// 0000   0000  000
// 0   0 0     0   0
// 0   0 0     0   0
// 0   0 0     0   0
// 0000   0000  000
// --------------------------------------------------------------------
// Odometer
// DCO control for odometer comes from the most significant bits of
// odometer control signals
    wire ac_stress_clk;
    reg [4:0] odometer_dco_con;
    always @ (*) begin
        if (!resetb) begin
            odometer_dco_con <= 0;
        end else if (odometer_enable_dco) begin
            odometer_dco_con[4] <= stress;
            odometer_dco_con[3] <= ac_dc;
            odometer_dco_con[2] <= sel_inv;
            odometer_dco_con[1] <= sel_nand;
            odometer_dco_con[0] <= sel_nor;            
        end

    end
    /*
    assign odometer_dco_con =   {stress[NUM_ODOMETER_P+NUM_ODOMETER_S],
                                ac_dc[NUM_ODOMETER_P+NUM_ODOMETER_S],
                                sel_inv[NUM_ODOMETER_P+NUM_ODOMETER_S],
                                sel_nand[NUM_ODOMETER_P+NUM_ODOMETER_S],
                                sel_nor[NUM_ODOMETER_P+NUM_ODOMETER_S]};
    */
    VCO_full odometer_vco_full_inst(
        // INPUT
        .RESETB(odometer_resetb),
        .LOAD(odometer_load),
        .EN_VCO(odometer_dco_con[4]),
        .CLK_KILL(odometer_dco_con[3]),
        .VCO_DIV_SEL(odometer_dco_con[1:0]),
        // OUTPUT
        .PAD_OUT(odometer_vco_pad_out),
        .AC_STRESS_CLK(ac_stress_clk),
	.VDD(VDD),
	.VSS(VSS)
    );

// RISC-V
    wire clk_out;
    DCO_TOP dco_top_inst(
        .COARSE(dco_coarse),
        .FINE(dco_fine),
        .ROSC_EN(dco_en),
        .CLKOUT(clk_out),
	.VDD(VDD),
	.VSS(VSS)
    );


    //reg core_clk;
    //initial begin
    //    core_clk = 1'b0;
    //end
    //always begin
    //    #5;
    //    core_clk = ~core_clk;
    //end
    assign clk_i = clk_out;
// --------------------------------------------------------------------
// 0000  0000  0      000
// 0   0 0   0 0     0
// 0   0 0   0 0      000
// 0   0 0   0 0         0
// 0000  0000  00000  000
// --------------------------------------------------------------------
    
    wire fetch_enable_i;
    assign fetch_enable_i = fetch_enable;

    // DDLS control signals
    reg [63:0] ddls_result_cnt;
    reg [10:0] ddls_addr;
    reg ddls_overflow;
    reg ddls_64_cnt_overflow;

    assign riscv_errcnt = ddls_result_cnt;
    assign riscv_err_overflow = ddls_overflow;
    assign riscv_64_cnt_overflow = ddls_64_cnt_overflow;

    wire [BUFFERWIDTH - 1 :0] ddls_result;
    wire ddls_result_flag;
    reg [BUFFERWIDTH - 1 : 0] ddls_result_reg;
    reg ddls_result_flag_reg;

    // DDLS from RISCV
    wire [BUFFERWIDTH - 1 :0] ddls_in_primary; // signals from primary RARE
    wire [BUFFERWIDTH - 1 :0] ddls_in_secondary; // signals from secondary RARE
    wire [535:0] scan_out_primary;
    wire [535:0] scan_out_secondary;

    always @ (*) begin
        if (stream_select == 2'b01 || stream_select == 2'b11) begin
            scan_out <= scan_out_primary;
        end else if (stream_select == 2'b10) begin
            scan_out <= scan_out_secondary;
        end
    end


    wire [8:0][32-1:0] mem_scan_out_primary;
    //wire [288:0] ddls_mem_scan_out_primary;
    wire [8:0][32-1:0] mem_scan_out_secondary;
    //wire [288:0] ddls_mem_scan_out_secondary;
    
    

    assign scan_out_primary[0+:32] = mem_scan_out_primary[0]; // X1
    assign scan_out_secondary[0+:32] = mem_scan_out_secondary[0];
    assign scan_out_primary[32+:32] = mem_scan_out_primary[1]; // X2
    assign scan_out_secondary[32+:32] = mem_scan_out_secondary[1];
    assign scan_out_primary[64+:32] = mem_scan_out_primary[2]; // X3
    assign scan_out_secondary[64+:32] = mem_scan_out_secondary[2];
    assign scan_out_primary[96+:32] = mem_scan_out_primary[3]; // X10
    assign scan_out_secondary[96+:32] = mem_scan_out_secondary[3];
    assign scan_out_primary[128+:32] = mem_scan_out_primary[4]; // X11
    assign scan_out_secondary[128+:32] = mem_scan_out_secondary[4];
    assign scan_out_primary[160+:32] = mem_scan_out_primary[5]; // X12
    assign scan_out_secondary[160+:32] = mem_scan_out_secondary[5];
    assign scan_out_primary[192+:32] = mem_scan_out_primary[6]; // X13
    assign scan_out_secondary[192+:32] = mem_scan_out_secondary[6];
    assign scan_out_primary[224+:32] = mem_scan_out_primary[7]; // X14
    assign scan_out_secondary[224+:32] = mem_scan_out_secondary[7];
    assign scan_out_primary[256+:32] = mem_scan_out_primary[8]; // X15
    assign scan_out_secondary[256+:32] = mem_scan_out_secondary[8];

    // From IF
    wire instr_req_ddls_primary;
    assign scan_out_primary[288+:1] = instr_req_ddls_primary;
    assign ddls_in_primary[0] = instr_req_ddls_primary;
    wire instr_req_ddls_secondary;
    assign scan_out_secondary[288+:1] = instr_req_ddls_secondary;
    assign ddls_in_secondary[0] = instr_req_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [10:0] instr_addr_ddls_primary;
    assign scan_out_primary[289+:11] = instr_addr_ddls_primary;
    assign ddls_in_primary[1+:11] = instr_addr_ddls_primary;
    wire [10:0] instr_addr_ddls_secondary;
    assign scan_out_secondary[289+:11] = instr_addr_ddls_secondary;
    assign ddls_in_secondary[1+:11] = instr_addr_ddls_secondary;
    //////////////////////////////////////////////////////////////
    //wire instr_valid_id_ddls_primary;
    //assign scan_out_primary[300+:1] = instr_valid_id_ddls_primary;
    //assign ddls_in_primary[12+:1] = instr_valid_id_ddls_primary;
    //wire instr_valid_id_ddls_secondary;
    //assign scan_out_secondary[300+:1] = instr_valid_id_ddls_secondary;
    //assign ddls_in_secondary[12+:1] = instr_valid_id_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [31:0] instr_rdata_id_ddls_primary;
    assign scan_out_primary[300+:32] = instr_rdata_id_ddls_primary;
    assign ddls_in_primary[12+:32] = instr_rdata_id_ddls_primary;
    wire [31:0] instr_rdata_id_ddls_secondary;
    assign scan_out_secondary[300+:32] = instr_rdata_id_ddls_secondary;
    assign ddls_in_secondary[12+:32] = instr_rdata_id_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire is_fetch_failed_id_ddls_primary;
    assign scan_out_primary[332+:1] = is_fetch_failed_id_ddls_primary;
    assign ddls_in_primary[44+:1] = is_fetch_failed_id_ddls_primary;
    wire is_fetch_failed_id_ddls_secondary;
    assign scan_out_secondary[332+:1] = is_fetch_failed_id_ddls_secondary;
    assign ddls_in_secondary[44+:1] = is_fetch_failed_id_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [10:0] pc_if_ddls_primary;
    assign scan_out_primary[333+:11] = pc_if_ddls_primary;
    assign ddls_in_primary[45+:11] = pc_if_ddls_primary;
    wire [10:0] pc_if_ddls_secondary;
    assign scan_out_secondary[333+:11] = pc_if_ddls_secondary;
    assign ddls_in_secondary[45+:11] = pc_if_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [10:0] pc_id_ddls_primary;
    assign scan_out_primary[344+:11] = pc_id_ddls_primary;
    assign ddls_in_primary[56+:11] = pc_id_ddls_primary;
    wire [10:0] pc_id_ddls_secondary;
    assign scan_out_secondary[344+:11] = pc_id_ddls_secondary;
    assign ddls_in_secondary[56+:11] = pc_id_ddls_secondary;
    //////////////////////////////////////////////////////////////
    // From ID
    wire branch_in_ex_ddls_primary;
    assign scan_out_primary[355+:1] = branch_in_ex_ddls_primary;
    assign ddls_in_primary[67+:1] = branch_in_ex_ddls_primary;
    wire branch_in_ex_ddls_secondary;
    assign scan_out_secondary[355+:1] = branch_in_ex_ddls_secondary;
    assign ddls_in_secondary[67+:1] = branch_in_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [10:0] jump_target_id_ddls_primary;
    assign scan_out_primary[356+:11] = jump_target_id_ddls_primary;
    assign ddls_in_primary[68+:11] = jump_target_id_ddls_primary;
    wire [10:0] jump_target_id_ddls_secondary;
    assign scan_out_secondary[356+:11] = jump_target_id_ddls_secondary;
    assign ddls_in_secondary[68+:11] = jump_target_id_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire alu_en_ex_ddls_primary;
    assign scan_out_primary[367+:1] = alu_en_ex_ddls_primary;
    assign ddls_in_primary[79+:1] = alu_en_ex_ddls_primary;
    wire alu_en_ex_ddls_secondary;
    assign scan_out_secondary[367+:1] = alu_en_ex_ddls_secondary;
    assign ddls_in_secondary[79+:1] = alu_en_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [6:0] alu_operator_ex_ddls_primary;
    assign scan_out_primary[368+:7] = alu_operator_ex_ddls_primary;
    assign ddls_in_primary[80+:7] = alu_operator_ex_ddls_primary;
    wire [6:0] alu_operator_ex_ddls_secondary;
    assign scan_out_secondary[368+:7] = alu_operator_ex_ddls_secondary;
    assign ddls_in_secondary[80+:7] = alu_operator_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire mult_en_ex_ddls_primary;
    assign scan_out_primary[375+:1] = mult_en_ex_ddls_primary;
    assign ddls_in_primary[87+:1] = mult_en_ex_ddls_primary;
    wire mult_en_ex_ddls_secondary;
    assign scan_out_secondary[375+:1] = mult_en_ex_ddls_secondary;
    assign ddls_in_secondary[87+:1] = mult_en_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [2:0] mult_operator_ex_ddls_primary;
    assign scan_out_primary[376+:3] = mult_operator_ex_ddls_primary;
    assign ddls_in_primary[88+:3] = mult_operator_ex_ddls_primary;
    wire [2:0] mult_operator_ex_ddls_secondary;
    assign scan_out_secondary[376+:3] = mult_operator_ex_ddls_secondary;
    assign ddls_in_secondary[88+:3] = mult_operator_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    // From EX
    wire [10:0] jump_target_ex_ddls_primary;
    assign scan_out_primary[379+:11] = jump_target_ex_ddls_primary;
    assign ddls_in_primary[91+:11] = jump_target_ex_ddls_primary;
    wire [10:0] jump_target_ex_ddls_secondary;
    assign scan_out_secondary[379+:11] = jump_target_ex_ddls_secondary;
    assign ddls_in_secondary[91+:11] = jump_target_ex_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire branch_decision_ddls_primary;
    assign scan_out_primary[390+:1] = branch_decision_ddls_primary;
    assign ddls_in_primary[102+:1] = branch_decision_ddls_primary;
    wire branch_decision_ddls_secondary;
    assign scan_out_secondary[390+:1] = branch_decision_ddls_secondary;
    assign ddls_in_secondary[102+:1] = branch_decision_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [5:0] regfile_waddr_fw_wb_ddls_primary;
    assign scan_out_primary[391+:6] = regfile_waddr_fw_wb_ddls_primary;
    assign ddls_in_primary[103+:6] = regfile_waddr_fw_wb_ddls_primary;
    wire [5:0] regfile_waddr_fw_wb_ddls_secondary;
    assign scan_out_secondary[391+:6] = regfile_waddr_fw_wb_ddls_secondary;
    assign ddls_in_secondary[103+:6] = regfile_waddr_fw_wb_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire regfile_we_wb_ddls_primary;
    assign scan_out_primary[397+:1] = regfile_we_wb_ddls_primary;
    assign ddls_in_primary[109+:1] = regfile_we_wb_ddls_primary;
    wire regfile_we_wb_ddls_secondary;
    assign scan_out_secondary[397+:1] = regfile_we_wb_ddls_secondary;
    assign ddls_in_secondary[109+:1] = regfile_we_wb_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [31:0] regfile_wdata_ddls_primary;
    assign scan_out_primary[398+:32] = regfile_wdata_ddls_primary;
    assign ddls_in_primary[110+:32] = regfile_wdata_ddls_primary;
    wire [31:0] regfile_wdata_ddls_secondary;
    assign scan_out_secondary[398+:32] = regfile_wdata_ddls_secondary;
    assign ddls_in_secondary[110+:32] = regfile_wdata_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [5:0] regfile_alu_waddr_fw_ddls_primary;
    assign scan_out_primary[430+:6] = regfile_alu_waddr_fw_ddls_primary;
    assign ddls_in_primary[142+:6] = regfile_alu_waddr_fw_ddls_primary;
    wire [5:0] regfile_alu_waddr_fw_ddls_secondary;
    assign scan_out_secondary[430+:6] = regfile_alu_waddr_fw_ddls_secondary;
    assign ddls_in_secondary[142+:6] = regfile_alu_waddr_fw_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire regfile_alu_we_fw_ddls_primary;
    assign scan_out_primary[436+:1] = regfile_alu_we_fw_ddls_primary;
    assign ddls_in_primary[148+:1] = regfile_alu_we_fw_ddls_primary;
    wire regfile_alu_we_fw_ddls_secondary;
    assign scan_out_secondary[436+:1] = regfile_alu_we_fw_ddls_secondary;
    assign ddls_in_secondary[148+:1] = regfile_alu_we_fw_ddls_secondary;
    //////////////////////////////////////////////////////////////
    wire [31:0] regfile_alu_wdata_fw_ddls_primary;
    assign scan_out_primary[437+:32] = regfile_alu_wdata_fw_ddls_primary;
    assign ddls_in_primary[149+:32] = regfile_alu_wdata_fw_ddls_primary;
    wire [31:0] regfile_alu_wdata_fw_ddls_secondary;
    assign scan_out_secondary[437+:32] = regfile_alu_wdata_fw_ddls_secondary;
    assign ddls_in_secondary[149+:32] = regfile_alu_wdata_fw_ddls_secondary;
    //////////////////////////////////////////////////////////////
    assign ddls_in_primary[181+:11] = pc_if_ddls_primary;
    assign ddls_in_secondary[181+:11] = pc_if_ddls_primary;

    
    always @ (posedge clk_out_system or negedge riscv_resetb_reg[1]) begin
        if (!riscv_resetb_reg[1]) begin
            core_cycle_count_primary_reg <= 0;
            core_cycle_count_secondary_reg <= 0;
        end else begin
            core_cycle_count_primary_reg <= core_cycle_count_primary;
            core_cycle_count_secondary_reg <= core_cycle_count_secondary;
        end
    end

    assign ddls_in_primary[192+:64] = core_cycle_count_primary_reg;
    assign scan_out_primary[469+:64] = core_cycle_count_primary_reg;
    assign ddls_in_secondary[192+:64] = core_cycle_count_secondary_reg;
    assign scan_out_secondary[469+:64] = core_cycle_count_secondary_reg;
    assign scan_out_primary[533+:3] = 3'b000;
    assign scan_out_secondary[533+:3] = 3'b000;

    ddls_decoder # (
        .INPUTSIZE(INPUTSIZE),
        .BUFFERSIZE(BUFFERSIZE)
    ) ddls_decoder_inst (
        .delay_sel_in(delay_sel_in),
        .delay_sel_out(delay_sel_out)
    );

    ddls_delayed_resetb # (
        .BUFFERSIZE(BUFFERSIZE)
    ) ddls_delayed_resetb_inst (
        .clk(clk_out_system),
        .resetb(riscv_resetb_reg[1]),
        .delay_sel(delay_sel_out),
        // output
        .primary_resetb(primary_resetb),
        .secondary_resetb(secondary_resetb),
        .riscv_clk_en(riscv_clk_en_reg[1]),
        .riscv_clk_en_primary(clk_en_in_primary),
        .riscv_clk_en_secondary(clk_en_in_secondary)
    );

    // ddls signals

    ddls # (
        .BUFFERSIZE(BUFFERSIZE),
        .BUFFERWIDTH(BUFFERWIDTH)
    ) ddls_inst (
        .clk(clk_out_system),
        .resetb(ddls_resetb_reg[1]),
        .start(secondary_resetb),
        .primary_data(ddls_in_primary),
        .secondary_data(ddls_in_secondary),
        .delay_sel(delay_sel_out),
        .result(ddls_result),
        .result_flag(ddls_result_flag)
    );

    reg [4:0] clk_en_in_secondary_reg;
    always @ (posedge clk_out_system or negedge riscv_resetb_reg[1]) begin
        if (!riscv_resetb_reg[1]) begin
            clk_en_in_secondary_reg <= 0;
        end else begin
            clk_en_in_secondary_reg <= {clk_en_in_secondary_reg[3:0], (clk_en_in_secondary&!riscv_ready_out_secondary&!riscv_ready_out_primary)};
        end
    end

    always @ (posedge clk_out_system or negedge ddls_resetb_reg[1]) begin
        if (!ddls_resetb_reg[1]) begin
            ddls_result_reg <= {{BUFFERWIDTH}{1'b0}};
            ddls_result_flag_reg <= 1'b0;
            ddls_result_cnt <= 0;
            ddls_overflow <= 1'b0;
        ddls_64_cnt_overflow <= 1'b0;
            ddls_addr <= 0;
        end else if (clk_en_in_secondary_reg[4]) begin
            ddls_result_reg <= ddls_result;
            ddls_result_flag_reg <= ddls_result_flag;
            if (ddls_result_flag_reg == 1'b1) begin // previous one was flagged now increment cnt because it is uploaded to sram
                if (ddls_result_cnt == 64'hFFFFFFFFFFFFFFFF) begin
                    ddls_result_cnt <= ddls_result_cnt;
                    ddls_64_cnt_overflow <= 1'b1;
                end else if (ddls_result_cnt >= 10'b1111111111) begin
                    ddls_overflow <= 1'b1;
                    ddls_addr <= ddls_addr;
                    ddls_result_cnt <= ddls_result_cnt + 1'b1;
                end else  begin
                    ddls_result_cnt <= ddls_result_cnt + 1'b1;
                    ddls_addr <= ddls_addr + 1'b1;
                end
            end
        end else begin
            ddls_result_reg <= 0;
            ddls_result_flag_reg <= 0;
        end
    end

// DDLS SRAMs
    wire mem_clk_ddls;
    wire [BUFFERWIDTH - 1 : 0] mem_data_in_ddls;
    wire [BUFFERWIDTH - 1 : 0] mem_data_out_ddls;
    wire [31:0] mem_addr_ddls;
    wire mem_we_ddls;
    wire mem_en_ddls;
    wire [31:0] mem_beb_ddls;

    rare_sram_mux # (
        .data_width(BUFFERWIDTH)
    ) rare_sram_mux_ddls (
        // from AXI
        .stream_clk(stream_clk),
        .stream_enable(~fetch_enable_i),

        .stream_addr(stream_addr_ddls),
        .stream_data_in(stream_data_in_ddls),
        .stream_data_out(stream_data_out_ddls),
        .stream_we(stream_we_ddls),
        .stream_en(stream_en_ddls),
        // from DDLS
        .core_clk(clk_out_system),
        .core_addr(ddls_addr), // ddls counter, number of errors counted
        .core_data_in(ddls_result_reg), // result
        .core_data_out(), // open
        .core_we(ddls_result_flag_reg && (!ddls_overflow)), // enable when error occurs
        .core_en(ddls_result_flag_reg && (!ddls_overflow)), // enable when error occures
        .core_beb(32'h00000000),
        // to MEM
        .mem_clk(mem_clk_ddls),
        .mem_addr(mem_addr_ddls),
        .mem_data_in(mem_data_in_ddls),
        .mem_data_out(mem_data_out_ddls),
        .mem_we(mem_we_ddls),
        .mem_en(mem_en_ddls),
        .mem_beb(mem_beb_ddls)
        
    );

    // how many SRAMs do we need?
    // each sram holds 32 bits.
    // if we need to store 100 bits -> 100 / 32
    genvar ddls_sram_cnt;
    generate
        for (ddls_sram_cnt = 0; ddls_sram_cnt*32 < BUFFERWIDTH; ddls_sram_cnt = ddls_sram_cnt + 1) begin
            TS1N28HPCPSVTB1024X32M4SW DDLS_CACHE (
                .CLK(mem_clk_ddls),
                .A(mem_addr_ddls[9:0]),
                .D(mem_data_in_ddls[ddls_sram_cnt*32 +: 32]),
                .Q(mem_data_out_ddls[ddls_sram_cnt*32 +: 32]),
                .CEB(!(mem_en_ddls)),
                .WEB(!(mem_we_ddls)),
                .BWEB(mem_beb_ddls)
            );
        end
        /*
        for (ddls_sram_cnt = 0; ddls_sram_cnt < 8; ddls_sram_cnt = ddls_sram_cnt + 1) begin
            TS1N28HPCPSVTB2048X32M4SW DDLS_CACHE_MEM (
                .CLK(mem_clk_ddls),
                .A(mem_addr_ddls[10:0]),
                .D(mem_data_in_ddls[(256+ddls_sram_cnt*32)+:32]),
                .Q(mem_data_out_ddls[(256+ddls_sram_cnt*32)+:32]),
                .CEB(!(mem_en_ddls)),
                .WEB(!(mem_we_ddls)),
                .BWEB(mem_beb_ddls)
            );
        end
        */
    endgenerate

    // Saving register values when error
    // not all of them will be saved
    // [256+:32] X1 return address 
    // [32+:32] X2 stack pointer
    // [64+:32] X3 global pointer
    // [96+:32] X10 function argument 0
    // [128+:32] X11 function argument 1
    // [160+:32] X12 function argument 2
    // [192+:32] X13 function argument 3
    // [224+:32] X14 function argument 4
    // [256+:32] X15 fucction argument 5

    




// --------------------------------------------------------------------
// 0   0 00000   0   00000 00000 0000        00000 0   0
// 0   0 0      0 0    0   0     0   0       0     00  0
// 00000 00000 00000   0   00000 0000        00000 0 0 0
// 0   0 0     0   0   0   0     0   0       0     0  00
// 0   0 00000 0   0   0   00000 0   0       00000 0   0
// --------------------------------------------------------------------
    // actual enable signal going into the individual heater is 
    // AND of global enable, pwm and individual cell enable
    //reg [NUM_HEATER_TOP - 1 : 0] heater_enable_pwm_top;
    //wire [7:0] heater_enable_pwm_dut_primary = heater_enable_pwm_top[0+:8];
    //wire [7:0] heater_enable_pwm_dut_secondary = heater_enable_pwm_top[8+:8];

    reg [NUM_HEATER_P - 1 : 0] heater_enable_pwm_primary;
    reg [NUM_HEATER_S - 1 : 0] heater_enable_pwm_secondary;

    // half of each top/primary/secondary heater are controlled by heater_pwm_a
    // whiel the other other will be controlled using heater_pwm_b
    // even ones are controlled using heater_pwm_a
    // and odd ones are controlled using heater_pwm_b
    integer pwm_cnt;
    always @ (*) begin
        //for (pwm_cnt = 0; pwm_cnt < NUM_HEATER_TOP; pwm_cnt = pwm_cnt + 1) begin
        //    if (pwm_cnt % 2 == 0) begin
        //        heater_enable_pwm_top[pwm_cnt] <= (heater_global_enable && heater_pwm_a) ? heater_enable_top[pwm_cnt] : 1'b0;
        //    end else if (pwm_cnt % 2 == 1) begin
        //        heater_enable_pwm_top[pwm_cnt] <= (heater_global_enable && heater_pwm_b) ? heater_enable_top[pwm_cnt] : 1'b0;
        //    end
        //end

        for (pwm_cnt = 0; pwm_cnt < NUM_HEATER_P; pwm_cnt = pwm_cnt + 1) begin
            if (pwm_cnt % 2 == 0) begin
                heater_enable_pwm_primary[pwm_cnt] <= (heater_global_enable && heater_pwm_a) ? heater_enable_primary[pwm_cnt] : 1'b0;
            end else if (pwm_cnt % 2 == 1) begin
                heater_enable_pwm_primary[pwm_cnt] <= (heater_global_enable && heater_pwm_b) ? heater_enable_primary[pwm_cnt] : 1'b0;
            end
        end

        for (pwm_cnt = 0; pwm_cnt < NUM_HEATER_S; pwm_cnt = pwm_cnt + 1) begin
            if (pwm_cnt % 2 == 0) begin
                heater_enable_pwm_secondary[pwm_cnt] <= (heater_global_enable && heater_pwm_a) ? heater_enable_secondary[pwm_cnt] : 1'b0;
            end else if (pwm_cnt % 2 == 1) begin
                heater_enable_pwm_secondary[pwm_cnt] <= (heater_global_enable && heater_pwm_b) ? heater_enable_secondary[pwm_cnt] : 1'b0;
            end
        end
    end
    //assign heater_enable_pwm_top = (heater_global_enable && heater_pwm) ? heater_enable_top : {NUM_HEATER_TOP{1'b0}};
    //assign heater_enable_pwm_primary = (heater_global_enable && heater_pwm) ? heater_enable_primary : {NUM_HEATER_TOP{1'b0}};
    //assign heater_enable_pwm_secondary = (heater_global_enable && heater_pwm) ? heater_enable_secondary : {NUM_HEATER_TOP{1'b0}};

    // MAIN TOP heaters
    /*
    genvar heater_count;
    generate
        for (heater_count = 16; heater_count < NUM_HEATER_TOP; heater_count = heater_count + 1) begin
            heater_wrapper heater_unit_inst(
                .EN(heater_enable_pwm_top[heater_count])
            );
        end
    endgenerate
    */
    // Temperatur sensor
    //wire [NUM_TEMP_SENSOR_TOP - 1 : 0] temp_sensor_decode_top;
    //assign temp_sensor_decode_top = temp_sensor_decode[NUM_TEMP_SENSOR_PRIMARY+NUM_TEMP_SENSOR_SECONDARY+:NUM_TEMP_SENSOR_TOP];
    //wire [1:0] temp_sensor_decode_dut_primary = temp_sensor_decode_top[1:0];
    //wire [1:0] temp_sensor_decode_dut_secondary = temp_sensor_decode_top[3:2];
    
    
    wire [NUM_TEMP_SENSOR_PRIMARY - 1 : 0] temp_sensor_decode_primary;
    assign temp_sensor_decode_primary = temp_sensor_decode[0+:NUM_TEMP_SENSOR_PRIMARY];
    wire [NUM_TEMP_SENSOR_SECONDARY - 1 : 0] temp_sensor_decode_secondary;
    assign temp_sensor_decode_secondary = temp_sensor_decode[NUM_TEMP_SENSOR_PRIMARY+:NUM_TEMP_SENSOR_SECONDARY];

    /*
    genvar temp_count;
    generate
        for (temp_count = 4; temp_count < NUM_TEMP_SENSOR_TOP; temp_count = temp_count + 1) begin
            Sensor_Unit temp_sensor_inst(
                .Sense_Hi(sense_hi),
                .Sense_Lo(sense_lo),
                .Source_Hi(source_hi),
                .Source_Lo(source_lo),
                .SEL(temp_sensor_decode_top[temp_count])
            );
        end
    endgenerate
    */


// --------------------------------------------------------------------
//  000  00000 0000  00000   0   0   0
// 0       0   0   0 0      0 0  00 00
//  000    0   0000  00000 00000 0 0 0
//     0   0   0   0 0     0   0 0   0
//  000    0   0   0 00000 0   0 0   0
// --------------------------------------------------------------------
    ///////////////////////////////////////////////////////////////////////////////////////////
    // ODOMETER
    // odometer signal [NUM_ODOMETER_P+NUM_ODOMETER_S] -> odometer_DCO
    // odometer [NUM_ODOMETER_P-1:0] -> primary
    // odoemter [NUM_ODOMETER_P+NUM_ODOMETER_S-1:NUM_ODOMETER_P] -> secondary
    // odometer signals assign
    
    // Primary
    //wire [NUM_ODOMETER_P - 1 : 0] stress_primary;
    //assign stress_primary = stress[0+:NUM_ODOMETER_P];
    //wire [NUM_ODOMETER_P - 1 : 0] ac_dc_primary;
    //assign ac_dc_primary = ac_dc[0+:NUM_ODOMETER_P];
    //wire [NUM_ODOMETER_P - 1 : 0] sel_inv_primary;
    //assign sel_inv_primary = sel_inv[0+:NUM_ODOMETER_P];
    //wire [NUM_ODOMETER_P - 1 : 0] sel_nand_primary;
    //assign sel_nand_primary = sel_nand[0+:NUM_ODOMETER_P];
    //wire [NUM_ODOMETER_P - 1 : 0] sel_nor_primary;
    //assign sel_nor_primary = sel_nor[0+:NUM_ODOMETER_P];
    
    wire [11:0] bit_count_primary;
    //assign bit_count[0+:NUM_ODOMETER_P] = bit_count_primary;
    // Secondary
    //wire [NUM_ODOMETER_S - 1 : 0] stress_secondary;
    //assign stress_secondary = stress[NUM_ODOMETER_P+:NUM_ODOMETER_S];
    //wire [NUM_ODOMETER_S - 1 : 0] ac_dc_secondary;
    //assign ac_dc_secondary = ac_dc[NUM_ODOMETER_P+:NUM_ODOMETER_S];
    //wire [NUM_ODOMETER_S - 1 : 0] sel_inv_secondary;
    //assign sel_inv_secondary = sel_inv[NUM_ODOMETER_P+:NUM_ODOMETER_S];
    //wire [NUM_ODOMETER_S - 1 : 0] sel_nand_secondary;
    //assign sel_nand_secondary = sel_nand[NUM_ODOMETER_P+:NUM_ODOMETER_S];
    //wire [NUM_ODOMETER_S - 1 : 0] sel_nor_secondary;
    //assign sel_nor_secondary = sel_nor[NUM_ODOMETER_P+:NUM_ODOMETER_S];
    wire [11:0] bit_count_secondary;
    //assign bit_count[NUM_ODOMETER_P+:NUM_ODOMETER_S] = bit_count_secondary;
    //assign bit_count[NUM_ODOMETER_P+NUM_ODOMETER_S] = 16'h0000;
    localparam NUM_ODOMETER_P_LOG2 = $clog2(NUM_ODOMETER_P);
    localparam NUM_ODOMETER_S_LOG2 = $clog2(NUM_ODOMETER_S);

    wire [NUM_ODOMETER_LOG2 - 1 : 0] odometer_sel_primary_temp;
    assign odometer_sel_primary_temp = odometer_sel;

    wire [NUM_ODOMETER_LOG2 - 1 : 0] odometer_sel_secondary_temp;
    assign odometer_sel_secondary_temp = odometer_sel - NUM_ODOMETER_P;

    wire [NUM_ODOMETER_P_LOG2 - 1 : 0] odometer_sel_primary;
    wire [NUM_ODOMETER_S_LOG2 - 1 : 0] odometer_sel_secondary;
    assign odometer_sel_primary = odometer_sel_primary_temp[NUM_ODOMETER_P_LOG2 - 1 : 0];
    assign odometer_sel_secondary = odometer_sel_secondary_temp[NUM_ODOMETER_S_LOG2 - 1 : 0];
  


    always @ (*) begin
        if (odometer_enable_primary) begin
            bit_count <= {4'b0000, bit_count_primary};
        end else if (odometer_enable_secondary) begin
            bit_count <= {4'b0000, bit_count_secondary};
        end else begin
            bit_count <= 0;
        end
    end
    ///////////////////////////////////////////////////////////////////////////////////////////
    // Instruction memory check
    reg [31:0] stream_addr_instr_primary, stream_addr_instr_secondary;
    reg [31:0] stream_data_in_instr_primary, stream_data_in_instr_secondary;
    wire [31:0] stream_data_out_instr_primary, stream_data_out_instr_secondary;
    reg stream_we_instr_primary, stream_we_instr_secondary;
    reg stream_en_instr_primary, stream_en_instr_secondary;

    always @ (posedge stream_clk or negedge resetb) begin
        if (!resetb) begin
            stream_addr_instr_primary <= 32'b0;
            stream_addr_instr_secondary <= 32'b0;
            stream_data_in_instr_primary <= 32'b0;
            stream_data_in_instr_secondary <= 32'b0;
            stream_data_out_instr <= 32'b0;
            stream_we_instr_primary <= 1'b0;
            stream_we_instr_secondary <= 1'b0;
            stream_en_instr_primary <= 1'b0;
            stream_en_instr_secondary <= 1'b0;
        end else begin
            // default state /////////////////////
            stream_addr_instr_primary <= 32'b0;
            stream_addr_instr_secondary <= 32'b0;
            stream_data_in_instr_primary <= 32'b0;
            stream_data_in_instr_secondary <= 32'b0;
            stream_data_out_instr <= 32'b0;
            stream_we_instr_primary <= 1'b0;
            stream_we_instr_secondary <= 1'b0;
            stream_en_instr_primary <= 1'b0;
            stream_en_instr_secondary <= 1'b0;
            /////////////////////////////////////
            case(stream_select)
                2'b00: begin
                    stream_addr_instr_primary <= 32'b0;
                    stream_addr_instr_secondary <= 32'b0;
                    stream_data_in_instr_primary <= 32'b0;
                    stream_data_in_instr_secondary <= 32'b0;
                    stream_data_out_instr <= 32'b0;
                    stream_we_instr_primary <= 1'b0;
                    stream_we_instr_secondary <= 1'b0;
                    stream_en_instr_primary <= 1'b0;
                    stream_en_instr_secondary <= 1'b0;
                end

                2'b01: begin
                    stream_addr_instr_primary <= stream_addr_instr;;
                    stream_data_in_instr_primary <= stream_data_in_instr;
                    stream_data_out_instr <= stream_data_out_instr_primary;
                    stream_we_instr_primary <= stream_we_instr;
                    stream_en_instr_primary <= stream_en_instr;
                end

                2'b10: begin
                    stream_addr_instr_secondary <= stream_addr_instr;
                    stream_data_in_instr_secondary <= stream_data_in_instr;
                    stream_data_out_instr <= stream_data_out_instr_secondary;
                    stream_we_instr_secondary <= stream_we_instr;
                    stream_en_instr_secondary <= stream_en_instr;
                end

                2'b11: begin // write to both but no read gets performed
                    stream_addr_instr_primary <= stream_addr_instr;
                    stream_addr_instr_secondary <= stream_addr_instr;
                    stream_data_in_instr_primary <= stream_data_in_instr;
                    stream_data_in_instr_secondary <= stream_data_in_instr;
                    stream_we_instr_primary <= stream_we_instr;
                    stream_we_instr_secondary <= stream_we_instr;
                    stream_en_instr_primary <= stream_en_instr;
                    stream_en_instr_secondary <= stream_en_instr;
                end
            endcase            
        end
    end

    // Data memory check
    reg [31:0] stream_addr_data_primary, stream_addr_data_secondary;
    reg [31:0] stream_data_in_data_primary, stream_data_in_data_secondary;
    reg [31:0] stream_data_out_data_primary, stream_data_out_data_secondary;
    reg stream_we_data_primary, stream_we_data_secondary;
    reg stream_en_data_primary, stream_en_data_secondary;

    always @ (posedge stream_clk or negedge resetb) begin
        if (!resetb) begin
            stream_addr_data_primary <= 32'b0;
            stream_addr_data_secondary <= 32'b0;
            stream_data_in_data_primary <= 32'b0;
            stream_data_in_data_secondary <= 32'b0;
            stream_data_out_data <= 32'b0;
            stream_we_data_primary <= 1'b0;
            stream_we_data_secondary <= 1'b0;
            stream_en_data_primary <= 1'b0;
            stream_en_data_secondary <= 1'b0;
        end else begin
            // default state /////////////////////
            stream_addr_data_primary <= 32'b0;
            stream_addr_data_secondary <= 32'b0;
            stream_data_in_data_primary <= 32'b0;
            stream_data_in_data_secondary <= 32'b0;
            stream_data_out_data <= 32'b0;
            stream_we_data_primary <= 1'b0;
            stream_we_data_secondary <= 1'b0;
            stream_en_data_primary <= 1'b0;
            stream_en_data_secondary <= 1'b0;
            /////////////////////////////////////
            case(stream_select)
                2'b00: begin
                    stream_addr_data_primary <= 32'b0;
                    stream_addr_data_secondary <= 32'b0;
                    stream_data_in_data_primary <= 32'b0;
                    stream_data_in_data_secondary <= 32'b0;
                    stream_data_out_data <= 32'b0;
                    stream_we_data_primary <= 1'b0;
                    stream_we_data_secondary <= 1'b0;
                    stream_en_data_primary <= 1'b0;
                    stream_en_data_secondary <= 1'b0;
                end

                2'b01: begin
                    stream_addr_data_primary <= stream_addr_data;;
                    stream_data_in_data_primary <= stream_data_in_data;
                    stream_data_out_data <= stream_data_out_data_primary;
                    stream_we_data_primary <= stream_we_data;
                    stream_en_data_primary <= stream_en_data;
                end

                2'b10: begin
                    stream_addr_data_secondary <= stream_addr_data;
                    stream_data_in_data_secondary <= stream_data_in_data;
                    stream_data_out_data <= stream_data_out_data_secondary;
                    stream_we_data_secondary <= stream_we_data;
                    stream_en_data_secondary <= stream_en_data;
                end

                2'b11: begin // write to both but no read gets performed
                    stream_addr_data_primary <= stream_addr_data;
                    stream_addr_data_secondary <= stream_addr_data;
                    stream_data_in_data_primary <= stream_data_in_data;
                    stream_data_in_data_secondary <= stream_data_in_data;
                    stream_we_data_primary <= stream_we_data;
                    stream_we_data_secondary <= stream_we_data;
                    stream_en_data_primary <= stream_en_data;
                    stream_en_data_secondary <= stream_en_data;
                end
            endcase
        end
    end    

// --------------------------------------------------------------------
// 0000  0   0 00000  
// 0   0 0   0   0   
// 0   0 0   0   0   
// 0   0 0   0   0   
// 0000   000    0   
// --------------------------------------------------------------------

    rare_riscv_test_w_mem # (
        .COREV_PULP(COREV_PULP),
        .COREV_CLUSTER(COREV_CLUSTER),
        .FPU(FPU),
        .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
        .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
        .ZFINX(ZFINX),
        .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
        //.NUM_HEATER_MEM(8),
        //.NUM_TEMP_SENSOR_MEM(2),
        .NUM_HEATER_DUT(NUM_HEATER_P),
        .NUM_TEMP_SENSOR_DUT(NUM_TEMP_SENSOR_PRIMARY),
        .NUM_ODOMETER_LOG2(NUM_ODOMETER_P_LOG2),
        .NUM_ODOMETER(NUM_ODOMETER_P)
    ) rare_primary (
        .stream_clk(stream_clk),
        .stream_resetb(resetb),
        .core_clk(clk_out_system),
        .core_resetb(primary_resetb),
        .clk_out_riscv_en(clk_out_riscv_en_primary),
        // RISC-V
        .boot_addr_i(boot_addr_i),
        .mtvec_addr_i(mtvec_addr_i),
        //.dm_halt_addr_i(dm_halt_addr_i),
        //.hart_id_i(hart_id_i),
        //.dm_exception_addr_i(dm_exception_addr_i),
        .fetch_enable_i(fetch_enable_i),

        // AXI instruction memory
        .stream_addr_instr(stream_addr_instr_primary),
        .stream_data_in_instr(stream_data_in_instr_primary),
        .stream_data_out_instr(stream_data_out_instr_primary),
        .stream_we_instr(stream_we_instr_primary),
        .stream_en_instr(stream_en_instr_primary),
        
        // AXI data memory
        .stream_addr_data(stream_addr_data_primary),
        .stream_data_in_data(stream_data_in_data_primary),
        .stream_data_out_data(stream_data_out_data_primary),
        .stream_we_data(stream_we_data_primary),
        .stream_en_data(stream_en_data_primary),

        // DDLS outputs
        .instr_req_ddls(instr_req_ddls_primary),
        .instr_addr_ddls(instr_addr_ddls_primary),
        //.instr_valid_id_ddls(instr_valid_id_ddls_primary),
        .instr_rdata_id_ddls(instr_rdata_id_ddls_primary),
        .is_fetch_failed_id_ddls(is_fetch_failed_id_ddls_primary),
        .pc_if_ddls(pc_if_ddls_primary),
        .pc_id_ddls(pc_id_ddls_primary),
        .branch_in_ex_ddls(branch_in_ex_ddls_primary),
        .jump_target_id_ddls(jump_target_id_ddls_primary),
        .alu_en_ex_ddls(alu_en_ex_ddls_primary),
        .alu_operator_ex_ddls(alu_operator_ex_ddls_primary),
        .mult_en_ex_ddls(mult_en_ex_ddls_primary),
        .mult_operator_ex_ddls(mult_operator_ex_ddls_primary),
        .jump_target_ex_ddls(jump_target_ex_ddls_primary),
        .branch_decision_ddls(branch_decision_ddls_primary),
        .regfile_waddr_fw_wb_ddls(regfile_waddr_fw_wb_ddls_primary),
        .regfile_we_wb_ddls(regfile_we_wb_ddls_primary),
        .regfile_wdata_ddls(regfile_wdata_ddls_primary),
        .regfile_alu_waddr_fw_ddls(regfile_alu_waddr_fw_ddls_primary),
        .regfile_alu_we_fw_ddls(regfile_alu_we_fw_ddls_primary),
        .regfile_alu_wdata_fw_ddls(regfile_alu_wdata_fw_ddls_primary),
        .core_sleep_o(core_sleep_o_primary),
        .mem_scan_out(mem_scan_out_primary),

        // Heater ports
        //.heater_enable_mem(heater_enable_pwm_dut_primary),
        //.temp_sensor_decode_mem(temp_sensor_decode_dut_primary),
        .heater_enable_dut(heater_enable_pwm_primary),
        .temp_sensor_decode_dut(temp_sensor_decode_primary),
        .sense_hi(sense_hi),
        .sense_lo(sense_lo),
        .source_hi(source_hi),
        .source_lo(source_lo),
        // Odometer ports
        .odometer_sel(odometer_sel_primary),
        .odometer_enable(odometer_enable_primary),
        .ac_stress_clk(ac_stress_clk),
        .odometer_resetb(odometer_resetb),
        .stress(stress),
        .ac_dc(ac_dc),
        .sel_inv(sel_inv),
        .sel_nand(sel_nand),
        .sel_nor(sel_nor),
        .odometer_meas_trig(odometer_meas_trig),
        .odometer_load(odometer_load),
        .bit_count(bit_count_primary),
        
        .riscv_ready_out(riscv_ready_out_primary),
        .riscv_debug_out(riscv_debug_out_primary)
    );

    rare_riscv_test_w_mem # (
        .COREV_PULP(COREV_PULP),
        .COREV_CLUSTER(COREV_CLUSTER),
        .FPU(FPU),
        .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
        .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
        .ZFINX(ZFINX),
        .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
        //.NUM_HEATER_MEM(8),
        //.NUM_TEMP_SENSOR_MEM(2),
        .NUM_HEATER_DUT(NUM_HEATER_S),
        .NUM_TEMP_SENSOR_DUT(NUM_TEMP_SENSOR_SECONDARY),
        .NUM_ODOMETER_LOG2(NUM_ODOMETER_S_LOG2),
        .NUM_ODOMETER(NUM_ODOMETER_S)
    ) rare_secondary (
        .stream_clk(stream_clk),
        .stream_resetb(resetb),
        .core_clk(clk_out_system),
        .core_resetb(secondary_resetb),
        .clk_out_riscv_en(clk_out_riscv_en_secondary),
        // RISC-V
        .boot_addr_i(boot_addr_i),
        .mtvec_addr_i(mtvec_addr_i),
        //.dm_halt_addr_i(dm_halt_addr_i),
        //.hart_id_i(hart_id_i),
        //.dm_exception_addr_i(dm_exception_addr_i),
        .fetch_enable_i(fetch_enable_i),

        // AXI instruction memory
        .stream_addr_instr(stream_addr_instr_secondary),
        .stream_data_in_instr(stream_data_in_instr_secondary),
        .stream_data_out_instr(stream_data_out_instr_secondary),
        .stream_we_instr(stream_we_instr_secondary),
        .stream_en_instr(stream_en_instr_secondary),
        
        // AXI data memory
        .stream_addr_data(stream_addr_data_secondary),
        .stream_data_in_data(stream_data_in_data_secondary),
        .stream_data_out_data(stream_data_out_data_secondary),
        .stream_we_data(stream_we_data_secondary),
        .stream_en_data(stream_en_data_secondary),

        // DDLS outputs
        .instr_req_ddls(instr_req_ddls_secondary),
        .instr_addr_ddls(instr_addr_ddls_secondary),
        //.instr_valid_id_ddls(instr_valid_id_ddls_secondary),
        .instr_rdata_id_ddls(instr_rdata_id_ddls_secondary),
        .is_fetch_failed_id_ddls(is_fetch_failed_id_ddls_secondary),
        .pc_if_ddls(pc_if_ddls_secondary),
        .pc_id_ddls(pc_id_ddls_secondary),
        .branch_in_ex_ddls(branch_in_ex_ddls_secondary),
        .jump_target_id_ddls(jump_target_id_ddls_secondary),
        .alu_en_ex_ddls(alu_en_ex_ddls_secondary),
        .alu_operator_ex_ddls(alu_operator_ex_ddls_secondary),
        .mult_en_ex_ddls(mult_en_ex_ddls_secondary),
        .mult_operator_ex_ddls(mult_operator_ex_ddls_secondary),
        .jump_target_ex_ddls(jump_target_ex_ddls_secondary),
        .branch_decision_ddls(branch_decision_ddls_secondary),
        .regfile_waddr_fw_wb_ddls(regfile_waddr_fw_wb_ddls_secondary),
        .regfile_we_wb_ddls(regfile_we_wb_ddls_secondary),
        .regfile_wdata_ddls(regfile_wdata_ddls_secondary),
        .regfile_alu_waddr_fw_ddls(regfile_alu_waddr_fw_ddls_secondary),
        .regfile_alu_we_fw_ddls(regfile_alu_we_fw_ddls_secondary),
        .regfile_alu_wdata_fw_ddls(regfile_alu_wdata_fw_ddls_secondary),
        .core_sleep_o(core_sleep_o_secondary),
        .mem_scan_out(mem_scan_out_secondary),

        // Heater ports
        //.heater_enable_mem(heater_enable_pwm_dut_secondary),
        //.temp_sensor_decode_mem(temp_sensor_decode_dut_secondary),
        .heater_enable_dut(heater_enable_pwm_secondary),
        .temp_sensor_decode_dut(temp_sensor_decode_secondary),
        .sense_hi(sense_hi),
        .sense_lo(sense_lo),
        .source_hi(source_hi),
        .source_lo(source_lo),
        // Odometer ports
        .odometer_sel(odometer_sel_secondary),
        .odometer_enable(odometer_enable_secondary),
        .ac_stress_clk(ac_stress_clk),
        .odometer_resetb(odometer_resetb),
        .stress(stress),
        .ac_dc(ac_dc),
        .sel_inv(sel_inv),
        .sel_nand(sel_nand),
        .sel_nor(sel_nor),
        .odometer_meas_trig(odometer_meas_trig),
        .odometer_load(odometer_load),
        .bit_count(bit_count_secondary),

        .riscv_ready_out(riscv_ready_out_secondary),
        .riscv_debug_out(riscv_debug_out_secondary)
    );

endmodule

    



