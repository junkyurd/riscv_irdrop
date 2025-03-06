`timescale 1ns / 1ps

module rare_riscv_test # (
    // RISC-V
    parameter COREV_PULP = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1,

    // Heater parameters
    parameter NUM_HEATER = 80,
    parameter NUM_TEMP_SENSOR = 16,

    // Odometer parameters
    
    parameter NUM_ODOMETER = 21,
    parameter NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER)
)(
    input clk,
    input stream_clk,
    input odometer_bus_resetb,
    input resetb,

    // RISC-V
    input [31:0] boot_addr_i,
    input [31:0] mtvec_addr_i,
    //input [31:0] dm_halt_addr_i,
    //input [31:0] hart_id_i,
    //input [31:0] dm_exception_addr_i,

    input clk_out_riscv_en,
    
    // Instruction memory interface
    input fetch_enable_i,
    output instr_req_o,
    input instr_gnt_i,
    input instr_rvalid_i,
    output [31:0] instr_addr_o,
    input [31:0] instr_rdata_i,

    // Data memory interface
    output data_req_o,
    input data_gnt_i,
    input data_rvalid_i,
    output data_we_o,
    output [3:0] data_be_o,
    output [31:0] data_addr_o,
    input [31:0] data_rdata_i,
    output [31:0] data_wdata_o,

    // DDLS outputs
    // From IF
    output wire instr_req_ddls,
    output wire [10:0] instr_addr_ddls,
    //output wire instr_valid_id_ddls,
    output wire [31:0] instr_rdata_id_ddls,
    output wire is_fetch_failed_id_ddls,
    output wire [10:0] pc_if_ddls,
    output wire [10:0] pc_id_ddls,    

    // From ID
    output wire branch_in_ex_ddls,
    output wire [10:0] jump_target_id_ddls,
    output wire alu_en_ex_ddls,
    output wire [6:0] alu_operator_ex_ddls,
    output wire mult_en_ex_ddls,
    output wire [2:0] mult_operator_ex_ddls,

    // From EX
    output wire [10:0] jump_target_ex_ddls,
    output wire branch_decision_ddls,
    output wire [5:0] regfile_waddr_fw_wb_ddls,
    output wire regfile_we_wb_ddls,
    output wire [31:0] regfile_wdata_ddls,
    output wire [5:0] regfile_alu_waddr_fw_ddls,
    output wire regfile_alu_we_fw_ddls,
    output wire [31:0] regfile_alu_wdata_fw_ddls,

    output wire core_sleep_o,

    output wire [8:0][32-1:0] mem_scan_out,

    // Heater ports
    input wire [NUM_HEATER - 1 : 0] heater_enable,
    input wire [NUM_TEMP_SENSOR - 1 : 0] temp_sensor_decode,

    inout wire sense_hi,
    inout wire sense_lo,
    inout wire source_hi,
    inout wire source_lo,

    // Odometer ports
    input wire [NUM_ODOMETER_LOG2 - 1 : 0] odometer_sel,
    input odometer_enable,
    input wire ac_stress_clk,
    input wire odometer_resetb,
    input wire stress,
    input wire ac_dc,
    input wire sel_inv,
    input wire sel_nand,
    input wire sel_nor,
    input wire odometer_meas_trig,
    input wire odometer_load,
    output reg [11:0] bit_count
);

// --------------------------------------------------------------------
// 0000  00000 0000  0   0  000
// 0   0 0     0   0 0   0 0   
// 0   0 00000 0000  0   0 0  00
// 0   0 0     0   0 0   0 0   0
// 0000  00000 0000   000   000
// --------------------------------------------------------------------



wire [31:0] instr_addr_ddls_temp;
assign instr_addr_ddls = instr_addr_ddls_temp[12:2];
wire [31:0] pc_if_ddls_temp;
assign pc_if_ddls = pc_if_ddls_temp[12:2];
wire [31:0] pc_id_ddls_temp;
assign pc_id_ddls = pc_id_ddls_temp[12:2];
wire [31:0] jump_target_id_ddls_temp;
assign jump_target_id_ddls = jump_target_id_ddls_temp[12:2];
wire [31:0] jump_target_ex_ddls_temp;
assign jump_target_ex_ddls = jump_target_ex_ddls_temp[12:2];

// --------------------------------------------------------------------
// 0   0 00000   0   00000 00000 0000        
// 0   0 0      0 0    0   0     0   0     
// 00000 00000 00000   0   00000 0000  
// 0   0 0     0   0   0   0     0   0  
// 0   0 00000 0   0   0   00000 0   0  
// --------------------------------------------------------------------
// generating heaters

genvar heater_count;
generate
    for (heater_count = 0; heater_count < NUM_HEATER; heater_count = heater_count + 1) begin
        heater_wrapper heater_unit_dut_inst(
            .EN(heater_enable[heater_count])
        );
    end
endgenerate

// generating temperature sensors
genvar temp_count;
generate
    for (temp_count = 0; temp_count < NUM_TEMP_SENSOR; temp_count = temp_count + 1) begin
        temp_sensor_wrapper temp_sensor_dut_inst(
            .Sense_Hi(sense_hi),
            .Sense_Lo(sense_lo),
            .Source_Hi(source_hi),
            .Source_Lo(source_lo),
            .SEL(temp_sensor_decode[temp_count])
        );
    end
endgenerate

// --------------------------------------------------------------------
//  000  0000   000  0   0 00000 00000 00000 0000
// 0   0 0   0 0   0 00 00 0       0   0     0   0
// 0   0 0   0 0   0 0 0 0 00000   0   00000 0000
// 0   0 0   0 0   0 0   0 0       0   0     0   0
//  000  0000   000  0   0 00000   0   00000 0   0
// -------------------------------------------------------------------- 

reg [NUM_ODOMETER - 1 : 0] stress_bus;
reg [NUM_ODOMETER - 1 : 0] ac_dc_bus;
reg [NUM_ODOMETER - 1 : 0] sel_inv_bus;
reg [NUM_ODOMETER - 1 : 0] sel_nand_bus;
reg [NUM_ODOMETER - 1 : 0] sel_nor_bus;
reg [11:0] bit_count_bus [NUM_ODOMETER - 1 : 0];

integer odo_bus;
always @ (posedge stream_clk or negedge odometer_bus_resetb) begin
    if (!odometer_bus_resetb) begin
        stress_bus <= 0;
        ac_dc_bus <= 0;
        sel_inv_bus <= 0;
        sel_nand_bus <= 0;
        sel_nor_bus <= 0;
    end else if (odometer_enable) begin
        for (odo_bus = 0; odo_bus < NUM_ODOMETER; odo_bus = odo_bus + 1) begin
            if (odometer_sel == odo_bus) begin
                stress_bus[odo_bus] <= stress;
                ac_dc_bus[odo_bus] <= ac_dc;
                sel_inv_bus[odo_bus] <= sel_inv;
                sel_nand_bus[odo_bus] <= sel_nand;
                sel_nor_bus[odo_bus] <= sel_nor;                
            end
        end
    end
    
end

always @ (*) begin
    case (odometer_sel)
        0: bit_count <= bit_count_bus[0];
        1: bit_count <= bit_count_bus[1];
        2: bit_count <= bit_count_bus[2];
        3: bit_count <= bit_count_bus[3];
        4: bit_count <= bit_count_bus[4];
        5: bit_count <= bit_count_bus[5];
        6: bit_count <= bit_count_bus[6];
        7: bit_count <= bit_count_bus[7];
        8: bit_count <= bit_count_bus[8];
        9: bit_count <= bit_count_bus[9];
        10: bit_count <= bit_count_bus[10];
        11: bit_count <= bit_count_bus[11];
        12: bit_count <= bit_count_bus[12];
        13: bit_count <= bit_count_bus[13];
        14: bit_count <= bit_count_bus[14];
        15: bit_count <= bit_count_bus[15];
        16: bit_count <= bit_count_bus[16];
        17: bit_count <= bit_count_bus[17];
        18: bit_count <= bit_count_bus[18];
        19: bit_count <= bit_count_bus[19];
        20: bit_count <= bit_count_bus[20];
        default: bit_count <= 0; 
    endcase
end


genvar odo_count;
// some of odometers are used as voltage sensor (10% of NUM_ODOMETER)
//wire [11:0] bit_count_temp [NUM_ODOMETER - 1 : 0];
//assign bit_count = {4'b0000,bit_count_temp};


generate
    for (odo_count = 0; odo_count < NUM_ODOMETER; odo_count = odo_count + 1) begin
        if (odo_count < 3 ) begin
            
            odometer_full_stacked_rvt odometer_stacked_inst(
                .RESETB(odometer_resetb),
                .AC_STRESS_CLK(ac_stress_clk),
                .START(stress_bus[odo_count]),
                .AC_DC(ac_dc_bus[odo_count]),
                .SEL_INV99(sel_inv_bus[odo_count]),
                .SEL_INV101(sel_nand_bus[odo_count]),
                .SEL_INV97(sel_nor_bus[odo_count]),
                .MEAS_TRIG(odometer_meas_trig),
                .LOAD(odometer_load),
                .BF_COUNT(bit_count_bus[odo_count])
            );
            
            
        end else if (odo_count < 3+6) begin
        
            odometer_full_rvt odometer_inst(
                .RESETB(odometer_resetb),
                .AC_STRESS_CLK(ac_stress_clk),
                .START(stress_bus[odo_count]),
                .AC_DC(ac_dc_bus[odo_count]),
                .SEL_INV(sel_inv_bus[odo_count]),
                .SEL_NAND(sel_nand_bus[odo_count]),
                .SEL_NOR(sel_nor_bus[odo_count]),
                .MEAS_TRIG(odometer_meas_trig),
                .LOAD(odometer_load),
                .BF_COUNT(bit_count_bus[odo_count])
            );

        end else if (odo_count < 3+6+6) begin

            odometer_full_lvt odometer_inst(
                .RESETB(odometer_resetb),
                .AC_STRESS_CLK(ac_stress_clk),
                .START(stress_bus[odo_count]),
                .AC_DC(ac_dc_bus[odo_count]),
                .SEL_INV(sel_inv_bus[odo_count]),
                .SEL_NAND(sel_nand_bus[odo_count]),
                .SEL_NOR(sel_nor_bus[odo_count]),
                .MEAS_TRIG(odometer_meas_trig),
                .LOAD(odometer_load),
                .BF_COUNT(bit_count_bus[odo_count])
            );

        end else begin

            odometer_full_hvt odometer_inst(
                .RESETB(odometer_resetb),
                .AC_STRESS_CLK(ac_stress_clk),
                .START(stress_bus[odo_count]),
                .AC_DC(ac_dc_bus[odo_count]),
                .SEL_INV(sel_inv_bus[odo_count]),
                .SEL_NAND(sel_nand_bus[odo_count]),
                .SEL_NOR(sel_nor_bus[odo_count]),
                .MEAS_TRIG(odometer_meas_trig),
                .LOAD(odometer_load),
                .BF_COUNT(bit_count_bus[odo_count])
            );

        end

        //assign bit_count[odo_count] = {bit_count_temp[odo_count]};
    end
endgenerate

// --------------------------------------------------------------------
// 0000  00000  000   000        0   0
// 0   0   0   0     0   0       0   0
// 0000    0    000  0     00000 0   0
// 0   0   0       0 0   0        0 0
// 0   0 00000  000   000          0
// --------------------------------------------------------------------    

cv32e40p_top_RARE # (
    .COREV_PULP(COREV_PULP),
    .COREV_CLUSTER(COREV_CLUSTER),
    .FPU(FPU),
    .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
    .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
    .ZFINX(ZFINX),
    .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS)
) cv32e40p_top_inst (
    .clk_i(clk),
    .rst_ni(resetb),
    .pulp_clock_en_i(1'b0),
    .scan_cg_en_i(1'b0),
    // Core ID, Cluster ID, debug mode halt address and boot address are considered more or less static
    .boot_addr_i(boot_addr_i),                  // input [31:0]
    .mtvec_addr_i(mtvec_addr_i),                // input [31:0]
    .dm_halt_addr_i(32'b0),                     // input [31:0]
    .hart_id_i(32'b0),                          // input [31:0]
    .dm_exception_addr_i(32'b0),                // input [31:0]

    .clk_out_riscv_en(clk_out_riscv_en),
    .mem_scan_out(mem_scan_out),
    // Instruction memory interface
    .instr_req_o(instr_req_o),
    .instr_gnt_i(instr_gnt_i),
    .instr_rvalid_i(instr_rvalid_i),
    .instr_addr_o(instr_addr_o),
    .instr_rdata_i(instr_rdata_i),
    // Data memory interface
    .data_req_o(data_req_o),
    .data_gnt_i(data_gnt_i),
    .data_rvalid_i(data_rvalid_i),
    .data_we_o(data_we_o),
    .data_be_o(data_be_o), // output [3:0]
    .data_addr_o(data_addr_o), // output [31:0]
    .data_wdata_o(data_wdata_o), // output [31:0]
    .data_rdata_i(data_rdata_i), // input [31:0]
    // Interrupt
    .irq_i(32'b0),
    .irq_ack_o(),
    .irq_id_o(),
    // Debug interface
    .debug_req_i(1'b0),
    .debug_havereset_o(),                       // output
    .debug_running_o(),                         // output
    .debug_halted_o(),                          // output
    // CPU control signals
    .fetch_enable_i(fetch_enable_i),
    .core_sleep_o(core_sleep_o),

    // DDLS
    .instr_req_ddls(instr_req_ddls),
    .instr_addr_ddls(instr_addr_ddls_temp),
    //.instr_valid_id_ddls(instr_valid_id_ddls),
    .instr_rdata_id_ddls(instr_rdata_id_ddls),
    .is_fetch_failed_id_ddls(is_fetch_failed_id_ddls),
    .pc_if_ddls(pc_if_ddls_temp),
    .pc_id_ddls(pc_id_ddls_temp),
    .branch_in_ex_ddls(branch_in_ex_ddls),
    .jump_target_id_ddls(jump_target_id_ddls_temp),
    .alu_en_ex_ddls(alu_en_ex_ddls),
    .alu_operator_ex_ddls(alu_operator_ex_ddls),
    .mult_en_ex_ddls(mult_en_ex_ddls),
    .mult_operator_ex_ddls(mult_operator_ex_ddls),
    .jump_target_ex_ddls(jump_target_ex_ddls_temp),
    .branch_decision_ddls(branch_decision_ddls),
    .regfile_waddr_fw_wb_ddls(regfile_waddr_fw_wb_ddls),
    .regfile_we_wb_ddls(regfile_we_wb_ddls),
    .regfile_wdata_ddls(regfile_wdata_ddls),
    .regfile_alu_waddr_fw_ddls(regfile_alu_waddr_fw_ddls),
    .regfile_alu_we_fw_ddls(regfile_alu_we_fw_ddls),
    .regfile_alu_wdata_fw_ddls(regfile_alu_wdata_fw_ddls)
);

endmodule