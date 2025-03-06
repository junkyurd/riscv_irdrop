`timescale 1ns / 1ps

module rare_riscv_test_w_mem # (
    // RISC-V
    parameter COREV_PULP = 0, // PULP ISA Extension (incl. custom CSRs and hardware loop, excl. cv.elw)
    parameter COREV_CLUSTER = 0,  // PULP Cluster interface (incl. cv.elw)
    parameter FPU = 0,  // Floating Point Unit (interfaced via APU interface)
    parameter FPU_ADDMUL_LAT = 0,  // Floating-Point ADDition/MULtiplication computing lane pipeline registers number
    parameter FPU_OTHERS_LAT = 0,  // Floating-Point COMParison/CONVersion computing lanes pipeline registers number
    parameter ZFINX = 0,  // Float-in-General Purpose registers
    parameter NUM_MHPMCOUNTERS = 1,

    // Heater parameters
    //parameter NUM_HEATER_MEM = 16,
    //parameter NUM_TEMP_SENSOR_MEM = 4,
    parameter NUM_HEATER_DUT = 84,
    parameter NUM_TEMP_SENSOR_DUT = 18,
    // Odometer parameters
    parameter NUM_ODOMETER = 21,
    parameter NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER)
)(
    input stream_clk, // from axi fpga
    input stream_resetb,

    input core_clk, // gate clock from chip dco
    input core_resetb,
    input clk_out_riscv_en,

    //input riscv_stop_ctrl_resetb,
    
    // RISC-V
    input [31:0] boot_addr_i,
    input [31:0] mtvec_addr_i,
    //input [31:0] dm_halt_addr_i,
    //input [31:0] hart_id_i,
    //input [31:0] dm_exception_addr_i,
    input fetch_enable_i,

    // AXI instruction memory
    input [31:0] stream_addr_instr,
    input [31:0] stream_data_in_instr,
    output [31:0] stream_data_out_instr,
    input wire stream_we_instr,
    input wire stream_en_instr,
    // AXI data memory
    input [31:0] stream_addr_data,
    input [31:0] stream_data_in_data,
    output [31:0] stream_data_out_data,
    input wire stream_we_data,
    input wire stream_en_data,

    // DDLS outputs
    // From IF
    output reg instr_req_ddls,
    output reg [10:0] instr_addr_ddls,
    //output reg instr_valid_id_ddls,
    output reg [31:0] instr_rdata_id_ddls,
    output reg is_fetch_failed_id_ddls,
    output reg [10:0] pc_if_ddls,
    output reg [10:0] pc_id_ddls,    

    // From ID
    output reg branch_in_ex_ddls,
    output reg [10:0] jump_target_id_ddls,
    output reg alu_en_ex_ddls,
    output reg [6:0] alu_operator_ex_ddls,
    output reg mult_en_ex_ddls,
    output reg [2:0] mult_operator_ex_ddls,

    // From EX
    output reg [10:0] jump_target_ex_ddls,
    output reg branch_decision_ddls,
    output reg [5:0] regfile_waddr_fw_wb_ddls,
    output reg regfile_we_wb_ddls,
    output reg [31:0] regfile_wdata_ddls,
    output reg [5:0] regfile_alu_waddr_fw_ddls,
    output reg regfile_alu_we_fw_ddls,
    output reg [31:0] regfile_alu_wdata_fw_ddls,

    output wire core_sleep_o,

    output reg [31:0] mem_scan_out_0,
    output reg [31:0] mem_scan_out_1,
    output reg [31:0] mem_scan_out_2,
    output reg [31:0] mem_scan_out_3,
    output reg [31:0] mem_scan_out_4,
    output reg [31:0] mem_scan_out_5,
    output reg [31:0] mem_scan_out_6,
    output reg [31:0] mem_scan_out_7,
    output reg [31:0] mem_scan_out_8,

    

    // Heater ports
    //input wire [NUM_HEATER_MEM - 1 : 0] heater_enable_mem,
    //input wire [NUM_TEMP_SENSOR_MEM - 1 : 0] temp_sensor_decode_mem,
    input wire [NUM_HEATER_DUT - 1 : 0] heater_enable_dut,
    input wire [NUM_TEMP_SENSOR_DUT - 1 : 0] temp_sensor_decode_dut,

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
    output wire [11:0] bit_count,

    output wire riscv_ready_out,
    output wire riscv_debug_out

);
/*
reg [NUM_ODOMETER - 1 : 0] stress_bus;
reg [NUM_ODOMETER - 1 : 0] ac_dc_bus;
reg [NUM_ODOMETER - 1 : 0] sel_inv_bus;
reg [NUM_ODOMETER - 1 : 0] sel_nand_bus;
reg [NUM_ODOMETER - 1 : 0] sel_nor_bus;
reg [11:0] bit_count_bus [NUM_ODOMETER - 1 : 0];

integer odo_bus;
always @ (posedge stream_clk or negedge stream_resetb) begin
    if (!stream_resetb) begin
        stress_bus <= 0;
        ac_dc_bus <= 0;
        sel_inv_bus <= 0;
        sel_nand_bus <= 0;
        sel_nor_bus <= 0;
    end else if (odometer_enable) begin
        for (odo_bus = 0; odo_bus < NUM_ODOMETER; odo_bus = odo_bus+1) begin
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


always @ (posedge stream_clk) begin
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
        21: bit_count <= bit_count_bus[21];
        default: bit_count <= 0;
    endcase          
end
*/
// --------------------------------------------------------------------
// 0   0 00000   0   00000 00000 0000        
// 0   0 0      0 0    0   0     0   0       
// 00000 00000 00000   0   00000 0000        
// 0   0 0     0   0   0   0     0   0       
// 0   0 00000 0   0   0   00000 0   0       
// --------------------------------------------------------------------
/*
genvar heater_count;
generate
    for (heater_count = 0; heater_count < NUM_HEATER_MEM; heater_count = heater_count + 1) begin
        heater_wrapper heater_unit_mem_inst(
            .EN(heater_enable_mem[heater_count])
        );
    end
endgenerate
genvar temp_count;
generate
    for (temp_count = 0; temp_count < NUM_TEMP_SENSOR_MEM; temp_count = temp_count + 1) begin
        temp_sensor_wrapper temp_sensor_mem_inst(
            .Sense_Hi(sense_hi),
            .Sense_Lo(sense_lo),
            .Source_Hi(source_hi),
            .Source_Lo(source_lo),
            .SEL(temp_sensor_decode_mem[temp_count])
        );
    end
endgenerate
*/


// --------------------------------------------------------------------
// 0000  0000  0      000
// 0   0 0   0 0     0
// 0   0 0   0 0      000
// 0   0 0   0 0         0
// 0000  0000  00000  000
// --------------------------------------------------------------------

// From IF
wire instr_req_ddls_temp;
wire [10:0] instr_addr_ddls_temp;
//wire instr_valid_id_ddls_temp;
wire [31:0] instr_rdata_id_ddls_temp;
wire is_fetch_failed_id_ddls_temp;
wire [10:0] pc_if_ddls_temp;
wire [10:0] pc_id_ddls_temp;

// From ID
wire branch_in_ex_ddls_temp;
wire [10:0] jump_target_id_ddls_temp;
wire alu_en_ex_ddls_temp;
wire [6:0] alu_operator_ex_ddls_temp;
wire mult_en_ex_ddls_temp;
wire [2:0] mult_operator_ex_ddls_temp;

// From EX
wire [10:0] jump_target_ex_ddls_temp;
wire branch_decision_ddls_temp;
wire [5:0] regfile_waddr_fw_wb_ddls_temp;
wire regfile_we_wb_ddls_temp;
wire [31:0] regfile_wdata_ddls_temp;
wire [5:0] regfile_alu_waddr_fw_ddls_temp;
wire regfile_alu_we_fw_ddls_temp;
wire [31:0] regfile_alu_wdata_fw_ddls_temp;

// From MEM
reg [8:0][32-1:0] mem_scan_out;
wire [8:0][32-1:0] mem_scan_out_temp;
assign mem_scan_out_0 = mem_scan_out[0];
assign mem_scan_out_1 = mem_scan_out[1];
assign mem_scan_out_2 = mem_scan_out[2];
assign mem_scan_out_3 = mem_scan_out[3];
assign mem_scan_out_4 = mem_scan_out[4];
assign mem_scan_out_5 = mem_scan_out[5];
assign mem_scan_out_6 = mem_scan_out[6];
assign mem_scan_out_7 = mem_scan_out[7];
assign mem_scan_out_8 = mem_scan_out[8];


always @ (posedge core_clk or negedge core_resetb) begin
    if (!core_resetb) begin
        instr_req_ddls <= 0;
        instr_addr_ddls <= 0;
        //instr_valid_id_ddls <= 0;
        instr_rdata_id_ddls <= 0;
        is_fetch_failed_id_ddls <= 0;
        pc_if_ddls <= 0;
        pc_id_ddls <= 0;
        branch_in_ex_ddls <= 0;
        jump_target_id_ddls <= 0;
        alu_en_ex_ddls <= 0;
        alu_operator_ex_ddls <= 0;
        mult_en_ex_ddls <= 0;
        mult_operator_ex_ddls <= 0;
        jump_target_ex_ddls <= 0;
        branch_decision_ddls <= 0;
        regfile_waddr_fw_wb_ddls <= 0;
        regfile_we_wb_ddls <= 0;
        regfile_wdata_ddls <= 0;
        regfile_alu_waddr_fw_ddls <= 0;
        regfile_alu_we_fw_ddls <= 0;
        regfile_alu_wdata_fw_ddls <= 0;
        mem_scan_out <= 0;
    end else begin
        instr_req_ddls <= instr_req_ddls_temp;
        instr_addr_ddls <= instr_addr_ddls_temp;
        //instr_valid_id_ddls <= instr_valid_id_ddls_temp;
        instr_rdata_id_ddls <= instr_rdata_id_ddls_temp;
        is_fetch_failed_id_ddls <= is_fetch_failed_id_ddls_temp;
        pc_if_ddls <= pc_if_ddls_temp;
        pc_id_ddls <= pc_id_ddls_temp;
        branch_in_ex_ddls <= branch_in_ex_ddls_temp;
        jump_target_id_ddls <= jump_target_id_ddls_temp;
        alu_en_ex_ddls <= alu_en_ex_ddls_temp;
        alu_operator_ex_ddls <= alu_operator_ex_ddls_temp;
        mult_en_ex_ddls <= mult_en_ex_ddls_temp;
        mult_operator_ex_ddls <= mult_operator_ex_ddls_temp;
        jump_target_ex_ddls <= jump_target_ex_ddls_temp;
        branch_decision_ddls <= branch_decision_ddls_temp;
        regfile_waddr_fw_wb_ddls <= regfile_waddr_fw_wb_ddls_temp;
        regfile_we_wb_ddls <= regfile_we_wb_ddls_temp;
        regfile_wdata_ddls <= regfile_wdata_ddls_temp;
        regfile_alu_waddr_fw_ddls <= regfile_alu_waddr_fw_ddls_temp;
        regfile_alu_we_fw_ddls <= regfile_alu_we_fw_ddls_temp;
        regfile_alu_wdata_fw_ddls <= regfile_alu_wdata_fw_ddls_temp;
        mem_scan_out <= mem_scan_out_temp;
    end

end

// --------------------------------------------------------------------
// 00000 0   0  000  00000 0000  0   0  000  00000 00000  000  0   0
//   0   00  0 0       0   0   0 0   0 0   0   0     0   0   0 00  0
//   0   0 0 0  000    0   0000  0   0 0       0     0   0   0 0 0 0
//   0   0  00     0   0   0   0 0   0 0   0   0     0   0   0 0  00
// 00000 0   0  000    0   0   0  000   000    0   00000  000  0   0
// --------------------------------------------------------------------
// Muxing between AXI data inputs/RISC-V access
// based on fetch_enable_i
// if fetch_enable_i = 0 then from AXI
wire mem_clk_instr;
wire [31:0] mem_addr_instr;
wire [31:0] mem_data_in_instr;
reg [31:0] mem_data_out_instr;
//wire [31:0] mem_data_out_instr_00, mem_data_out_instr_01, mem_data_out_instr_10, mem_data_out_instr_11;
wire mem_we_instr;
wire mem_en_instr;
wire [31:0] mem_beb_instr;

// delayed stream signal into sram to guarantee timing closure
reg [31:0] stream_addr_instr_reg;
reg [31:0] stream_data_in_instr_reg;
//reg [31:0] stream_data_out_instr_reg;
reg stream_we_instr_reg;
reg stream_en_instr_reg;

// Core instruction memory
wire [31:0] core_addr_instr;
wire [31:0] core_data_in_instr;
wire [31:0] core_data_out_instr;
wire core_we_instr;
wire core_en_instr;


always @ (posedge stream_clk or negedge stream_resetb) begin
    if (!stream_resetb) begin
        stream_addr_instr_reg <= 32'b0;
        stream_data_in_instr_reg <= 32'b0;
        //stream_data_out_instr_reg <= 32'b0;
        stream_we_instr_reg <= 1'b0;
        stream_en_instr_reg <= 1'b0;
    end else begin
        stream_addr_instr_reg <= stream_addr_instr;
        stream_data_in_instr_reg <= stream_data_in_instr;
        //stream_data_out_instr_reg <= stream_data_out_instr;
        stream_we_instr_reg <= stream_we_instr;
        stream_en_instr_reg <= stream_en_instr;
    end
end

rare_sram_mux # (
    .data_width(32)
)rare_sram_mux_instr (
    // from AXI
    .stream_clk(stream_clk),
    .stream_enable(~fetch_enable_i),
    .stream_addr(stream_addr_instr_reg),
    .stream_data_in(stream_data_in_instr_reg),
    .stream_data_out(stream_data_out_instr),
    .stream_we(stream_we_instr_reg),
    .stream_en(stream_en_instr_reg),
    // from RISC-V
    .core_clk(core_clk),
    .core_addr({2'b00,core_addr_instr[31:2]}),
    .core_data_in(32'h00000000),
    .core_data_out(core_data_out_instr),
    .core_we(1'b0),
    .core_en(core_en_instr),
    .core_beb(32'h00000000),
    // to MEM
    .mem_clk(mem_clk_instr),
    .mem_addr(mem_addr_instr),
    .mem_data_in(mem_data_in_instr),
    .mem_data_out(mem_data_out_instr),
    .mem_we(mem_we_instr),
    .mem_en(mem_en_instr),
    .mem_beb(mem_beb_instr)
);

// four srams each sram is 2048 x 32 (8 KB)
/*
TS1N28HPCPSVTB2048X32M4SW INSTR_CACHE_11 (
    .CLK(mem_clk_instr),
    .A(mem_addr_instr[10:0]),
    .D(mem_data_in_instr),
    .Q(mem_data_out_instr_11),
    .CEB(!(mem_en_instr && (mem_addr_instr[12:11] == 2'b11))),
    .WEB(!(mem_we_instr && (mem_addr_instr[12:11] == 2'b11))),
    .BWEB(32'h00000000)
);

TS1N28HPCPSVTB2048X32M4SW INSTR_CACHE_10 (
    .CLK(mem_clk_instr),
    .A(mem_addr_instr[10:0]),
    .D(mem_data_in_instr),
    .Q(mem_data_out_instr_10),
    .CEB(!(mem_en_instr && (mem_addr_instr[12:11] == 2'b10))),
    .WEB(!(mem_we_instr && (mem_addr_instr[12:11] == 2'b10))),
    .BWEB(32'h00000000)
);
*/

//wire [31:0] mem_data_out_instr_0;
//wire [31:0] mem_data_out_instr_1;
//assign mem_data_out_instr = (mem_addr_instr[10] == 1'b1) ? mem_data_out_instr_1:mem_data_out_instr_0;
/*
TS1N28HPCPSVTB1024X32M4SW INSTR_CACHE_1 (
    .CLK(mem_clk_instr),
    .A(mem_addr_instr[9:0]),
    .D(mem_data_in_instr),
    .Q(mem_data_out_instr_1),
    .CEB(!(mem_en_instr && (mem_addr_instr[10] == 1'b1))),
    .WEB(!(mem_we_instr && (mem_addr_instr[10] == 1'b1))),
    .BWEB(mem_beb_instr)
);
*/
TS1N28HPCPSVTB2048X32M4SW INSTR_CACHE_0 (
    .CLK(mem_clk_instr),
    .A(mem_addr_instr[10:0]),
    .D(mem_data_in_instr),
    .Q(mem_data_out_instr), //_00),
    .CEB(!(mem_en_instr)), // && (mem_addr_instr[10] == 1'b0))), // && (mem_addr_instr[12:11] == 2'b00))),
    .WEB(!(mem_we_instr)), // && (mem_addr_instr[10] == 1'b0))), // && (mem_addr_instr[12:11] == 2'b00))),
    .BWEB(mem_beb_instr)
);

// --------------------------------------------------------------------
// 0000    0   00000   0
// 0   0  0 0    0    0 0
// 0   0 00000   0   00000
// 0   0 0   0   0   0   0
// 0000  0   0   0   0   0
// --------------------------------------------------------------------
// Muxing between AXI data inputs/RISC-V access
// based on fetch_enable_i
// if fetch_enable_i = 0 then from AXI
wire mem_clk_data;
wire [31:0] mem_addr_data;
wire [31:0] mem_data_in_data;
wire [31:0] mem_data_out_data;
wire mem_we_data;
wire mem_en_data;
wire [31:0] mem_beb_data;

// delayed stream signal into sram to guarantee timing closure
reg [31:0] stream_addr_data_reg;
reg [31:0] stream_data_in_data_reg;
//reg [31:0] stream_data_out_data_reg;
reg stream_we_data_reg;
reg stream_en_data_reg;

// Core data memory
wire [31:0] core_addr_data;
wire [31:0] core_data_in_data;
wire [31:0] core_data_out_data;
wire core_we_data;
wire core_en_data;
wire [31:0] core_beb_data;
wire [3:0] data_be_o;

assign core_beb_data = {{8{~data_be_o[3]}}, {8{~data_be_o[2]}}, {8{~data_be_o[1]}}, {8{~data_be_o[0]}}};

reg instr_rvalid_i;
always @ (posedge core_clk or negedge core_resetb) begin
    if (!core_resetb) begin
        instr_rvalid_i <= 1'b0;
    end else begin
        instr_rvalid_i <= core_en_instr;
    end
end

always @ (posedge stream_clk or negedge stream_resetb) begin
    if (!stream_resetb) begin
        stream_addr_data_reg <= 32'b0;
        stream_data_in_data_reg <= 32'b0;
        //stream_data_out_data_reg <= 32'b0;
        stream_we_data_reg <= 1'b0;
        stream_en_data_reg <= 1'b0;
    end else begin
        stream_addr_data_reg <= stream_addr_data;
        stream_data_in_data_reg <= stream_data_in_data;
        //stream_data_out_data_reg <= stream_data_out_data;
        stream_we_data_reg <= stream_we_data;
        stream_en_data_reg <= stream_en_data;
    end
end

rare_sram_mux # (
    .data_width(32)
) rare_sram_mux_data (
    // from AXI
    .stream_clk(stream_clk),
    .stream_enable(~fetch_enable_i),
    .stream_addr(stream_addr_data_reg),
    .stream_data_in(stream_data_in_data_reg),
    .stream_data_out(stream_data_out_data),
    .stream_we(stream_we_data_reg),
    .stream_en(stream_en_data_reg),
    // from RISC-V
    .core_clk(core_clk),
    .core_addr({2'b00, core_addr_data[31:2]}),
    .core_data_in(core_data_in_data),
    .core_data_out(core_data_out_data),
    .core_we(core_we_data),
    .core_en(core_en_data),
    .core_beb(core_beb_data),
    // to MEM
    .mem_clk(mem_clk_data),
    .mem_addr(mem_addr_data),
    .mem_data_in(mem_data_in_data),
    .mem_data_out(mem_data_out_data),
    .mem_we(mem_we_data),
    .mem_en(mem_en_data),
    .mem_beb(mem_beb_data)
);
// two srams each sram is 2048 x 32 (8 KB)

//wire [31:0] mem_data_out_data_0;
//wire [31:0] mem_data_out_data_1;
//assign mem_data_out_data = (mem_addr_data[10] == 1'b1) ? (mem_data_out_data_1):(mem_data_out_data_0);
/*
TS1N28HPCPSVTB1024X32M4SW DATA_CACHE_1 (
    .CLK(mem_clk_data),
    .A(mem_addr_data[9:0]),
    .D(mem_data_in_data),
    .Q(mem_data_out_data_1),
    .CEB(!(mem_en_data && (mem_addr_data[10] == 1'b1))),
    .WEB(!(mem_we_data && (mem_addr_data[10] == 1'b1))),
    .BWEB(mem_beb_data)
);
*/
TS1N28HPCPSVTB2048X32M4SW DATA_CACHE_0 (
    .CLK(mem_clk_data),
    .A(mem_addr_data[10:0]),
    .D(mem_data_in_data),
    .Q(mem_data_out_data),
    .CEB(!(mem_en_data)), // && (mem_addr_data[10] == 1'b0))),
    .WEB(!(mem_we_data)), // && (mem_addr_data[10] == 1'b0))),
    .BWEB(mem_beb_data)
);

reg data_rvalid_i;
always @ (posedge core_clk or negedge core_resetb) begin
    if (!core_resetb) begin
        data_rvalid_i <= 1'b0;
    end else begin
        data_rvalid_i <= core_en_data;
    end
end

// --------------------------------------------------------------------
//  000  00000  000  0000   000   000  0   0
// 0       0   0   0 0   0 0   0 0   0 00  0
//  000    0   0   0 0000  0     0   0 0 0 0
//     0   0   0   0 0     0   0 0   0 0  00
//  000    0    000  0      000   000  0   0
// --------------------------------------------------------------------
riscv_stop_ctrl riscv_stop_ctrl_inst(
    .clk(mem_clk_data),
    .resetb(clk_out_riscv_en),
    .ce (mem_en_data),
    .we (mem_we_data),
    .addr (mem_addr_data[31:0]),
    .data(mem_data_in_data[3:0]),
    .riscv_ready_out(riscv_ready_out),
    .riscv_debug_out(riscv_debug_out)
);

// --------------------------------------------------------------------
// 0000  0   0 00000  
// 0   0 0   0   0   
// 0   0 0   0   0   
// 0   0 0   0   0   
// 0000   000    0   
// --------------------------------------------------------------------
//part that gets influenced by noise/fault injection/heat
rare_riscv_test # (
    .COREV_PULP(COREV_PULP),
    .COREV_CLUSTER(COREV_CLUSTER),
    .FPU(FPU),
    .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
    .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
    .ZFINX(ZFINX),
    .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
    .NUM_HEATER(NUM_HEATER_DUT),
    .NUM_TEMP_SENSOR(NUM_TEMP_SENSOR_DUT),
    .NUM_ODOMETER(NUM_ODOMETER)    
) rare_riscv_test_inst (
    .clk(core_clk),
    .stream_clk(stream_clk),
    .odometer_bus_resetb(stream_resetb),
    .resetb(core_resetb),
    .fetch_enable_i(fetch_enable_i),
    // RISCV
    .boot_addr_i(boot_addr_i),
    .mtvec_addr_i(mtvec_addr_i),
    //.dm_halt_addr_i(dm_halt_addr_i),
    //.hart_id_i(hart_id_i),
    //.dm_exception_addr_i(dm_exception_addr_i),

    .clk_out_riscv_en(clk_out_riscv_en),
    .mem_scan_out(mem_scan_out_temp),

    // DDLS outputs
    .instr_req_ddls(instr_req_ddls_temp),
    .instr_addr_ddls(instr_addr_ddls_temp),
    //.instr_valid_id_ddls(instr_valid_id_ddls_temp),
    .instr_rdata_id_ddls(instr_rdata_id_ddls_temp),
    .is_fetch_failed_id_ddls(is_fetch_failed_id_ddls_temp),
    .pc_if_ddls(pc_if_ddls_temp),
    .pc_id_ddls(pc_id_ddls_temp),
    .branch_in_ex_ddls(branch_in_ex_ddls_temp),
    .jump_target_id_ddls(jump_target_id_ddls_temp),
    .alu_en_ex_ddls(alu_en_ex_ddls_temp),
    .alu_operator_ex_ddls(alu_operator_ex_ddls_temp),
    .mult_en_ex_ddls(mult_en_ex_ddls_temp),
    .mult_operator_ex_ddls(mult_operator_ex_ddls_temp),
    .jump_target_ex_ddls(jump_target_ex_ddls_temp),
    .branch_decision_ddls(branch_decision_ddls_temp),
    .regfile_waddr_fw_wb_ddls(regfile_waddr_fw_wb_ddls_temp),
    .regfile_we_wb_ddls(regfile_we_wb_ddls_temp),
    .regfile_wdata_ddls(regfile_wdata_ddls_temp),
    .regfile_alu_waddr_fw_ddls(regfile_alu_waddr_fw_ddls_temp),
    .regfile_alu_we_fw_ddls(regfile_alu_we_fw_ddls_temp),
    .regfile_alu_wdata_fw_ddls(regfile_alu_wdata_fw_ddls_temp),
    .core_sleep_o(core_sleep_o),

    // Instruction memory
    .instr_req_o(core_en_instr),
    .instr_gnt_i(1'b1),
    .instr_rvalid_i(instr_rvalid_i),
    .instr_addr_o(core_addr_instr),
    .instr_rdata_i(core_data_out_instr),
    // Data memory
    .data_req_o(core_en_data),
    .data_gnt_i(1'b1),
    .data_rvalid_i(data_rvalid_i),
    .data_we_o(core_we_data),
    .data_be_o(data_be_o),
    .data_addr_o(core_addr_data),
    .data_rdata_i(core_data_out_data),
    .data_wdata_o(core_data_in_data),

    // Heater
    .heater_enable(heater_enable_dut),
    .temp_sensor_decode(temp_sensor_decode_dut),
    .sense_hi(sense_hi),
    .sense_lo(sense_lo),
    .source_hi(source_hi),
    .source_lo(source_lo),
    // Odometer
    .odometer_sel(odometer_sel),
    .odometer_enable(odometer_enable),
    .ac_stress_clk(ac_stress_clk),
    .odometer_resetb(odometer_resetb),
    .stress(stress),
    .ac_dc(ac_dc),
    .sel_inv(sel_inv),
    .sel_nand(sel_nand),
    .sel_nor(sel_nor),
    .odometer_meas_trig(odometer_meas_trig),
    .odometer_load(odometer_load),
    .bit_count(bit_count)
);

endmodule