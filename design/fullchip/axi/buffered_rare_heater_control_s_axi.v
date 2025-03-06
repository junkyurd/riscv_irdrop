`timescale 1ns / 1ps

module buffered_rare_heater_control_s_axi # (
    // parameters for rare_heater_control_s_axi
    parameter C_S_AXI_ADDR_WIDTH = 6,
    parameter C_S_AXI_DATA_WIDTH = 8,
    
    parameter NUM_ODOMETER = 43, // NUM_ODOMETER is a combined value of primary + secondary + dco control
    parameter NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER),
    parameter NUM_ODOMETER_P = 21,
    parameter NUM_ODOMETER_S = 21,
    //parameter NUM_HEATER_TOP = 64,
    parameter NUM_HEATER_P = 64,
    parameter NUM_HEATER_S = 64,
    parameter DDLS_WIDTH = 100,
    // parameters for sync_fifo
    parameter FIFO_IN_REG = 1,
    parameter FIFO_OUT_REG = 1,
    parameter FIFO_DEPTH = 2,
    parameter FIFO_LOG2_DEPTH = 1
)(
    input wire                                  ACLK,
    input wire                                  ARESETB, // assync neg reset
    input wire                                  ACLK_EN,
    // write address
    input wire [C_S_AXI_ADDR_WIDTH - 1 : 0]     AWADDR,
    input wire                                  AWVALID,
    output wire                                 AWREADY,
    // write data
    input wire [C_S_AXI_DATA_WIDTH - 1 : 0]     WDATA,
    //input wire [C_S_AXI_DATA_WIDTH/8 - 1 : 0]   WSTRB,
    input wire                                  WVALID,
    output wire                                 WREADY,
    // read address
    input wire [C_S_AXI_ADDR_WIDTH - 1 : 0]     ARADDR,
    input wire                                  ARVALID,
    output wire                                 ARREADY,
    // read data
    output wire [C_S_AXI_DATA_WIDTH - 1 : 0]    RDATA,
    output wire [1:0]                           RRESP,
    output wire                                 RVALID,
    input wire                                  RREADY,

    // Clock control
    output wire                                 CLK_SELECT,
    output wire                                 FREQ_OUT_EN,
    
    output wire                                 DCO_EN,
    output wire [3:0]                           DCO_COARSE,
    output wire [3:0]                           DCO_FINE,

    output wire                                 FREQ_OUT_CNT_CLK_SELECT,
    output wire                                 FREQ_OUT_CNT_RESETB,
    output wire                                 FREQ_OUT_CNT_START,
    input wire                                  FREQ_OUT_CNT_DONE,

    // I/F with Intel rare ddls caches
    output wire [31:0]                          STREAM_ADDR_DDLS,
    output wire [DDLS_WIDTH - 1 : 0]            STREAM_DATA_IN_DDLS,
    input wire [DDLS_WIDTH - 1 : 0]             STREAM_DATA_OUT_DDLS,
    output wire                                 STREAM_WE_DDLS,
    output wire                                 STREAM_EN_DDLS,

    // I/F with Intel rare instr caches
    output wire [31:0]                          STREAM_ADDR_INSTR,
    output wire [31:0]                          STREAM_DATA_IN_INSTR,
    input wire [31:0]                           STREAM_DATA_OUT_INSTR,
    output wire                                 STREAM_WE_INSTR,
    output wire                                 STREAM_EN_INSTR,

    // I/F with Intel rare data caches
    output wire [31:0]                          STREAM_ADDR_DATA,
    output wire [31:0]                          STREAM_DATA_IN_DATA,
    input wire [31:0]                           STREAM_DATA_OUT_DATA,
    output wire                                 STREAM_WE_DATA,
    output wire                                 STREAM_EN_DATA,

    // Control signals
    output wire [7:0]                           DDLS_DELAY_SEL,
    output wire                                 DDLS_RESETB,
    output wire                                 CLK_CNT_RESETB,
    output wire                                 RISCV_RESETB,
    output wire                                 RISCV_CLK_EN,
    output wire                                 RISCV_RUN_CYCLE_ENABLE,
    output wire [63:0]                          RISCV_RUN_CYCLE,
    input wire                                  RISCV_RUN_DONE,
    input wire                                  RISCV_RUN_DONE_PRIMARY,
    input wire                                  RISCV_RUN_DONE_SECONDARY,

    input wire [63:0]                           CORE_CYCLE_COUNT_PRIMARY,
    input wire [63:0]                           CORE_CYCLE_COUNT_SECONDARY,

    input wire                                  CORE_SLEEP_O_PRIMARY,
    input wire                                  CORE_SLEEP_O_SECONDARY,

    input wire                                  RISCV_ERR_OVERFLOW,
    input wire                                  RISCV_64_CNT_OVERFLOW,
    input wire [63:0]                           RISCV_ERRCNT,
    
    output wire [1:0]                           STREAM_SEL,
    output wire                                 FETCH_ENABLE,
    output wire [31:0]                          BOOT_ADDR,
    output wire [31:0]                          MTVEC_ADDR, 

    // Heater
    output wire                                 HEATER_GLOBAL_ENABLE,
    //output wire [NUM_HEATER_TOP - 1 : 0]        HEATER_ENABLE_TOP,
    //output wire                                 HEATER_LOAD_TOP,
    output wire [NUM_HEATER_P - 1 : 0]          HEATER_ENABLE_PRIMARY,
    output wire                                 HEATER_LOAD_PRIMARY,
    output wire [NUM_HEATER_S - 1 : 0]          HEATER_ENABLE_SECONDARY,
    output wire                                 HEATER_LOAD_SECONDARY,

    // Sensors
    output wire [7:0]                           TEMP_SENSOR_SEL,
    output wire                                 TEMP_SENSOR_EN,
    //output wire [7:0]                           VOLTAGE_SENSOR_SEL,
    //output wire                                 VOLTAGE_SENSOR_EN,

    // Odometer
    output wire [NUM_ODOMETER_LOG2 - 1 : 0]     ODOMETER_SEL,
    output wire                                 ODOMETER_ENABLE_PRIMARY,
    output wire                                 ODOMETER_ENABLE_SECONDARY,
    output wire                                 ODOMETER_ENABLE_DCO,

    output reg                                  ODOMETER_RESETB,
    output reg                                  STRESS,
    output reg                                  AC_DC,
    output reg                                  SEL_INV,
    output reg                                  SEL_NAND,
    output reg                                  SEL_NOR,
    output reg                                  ODOMETER_MEAS_TRIG,
    output wire                                 ODOMETER_LOAD,
    input wire [15:0]                           BIT_COUNT,

    output wire                                 FREQ_OUT_SEL,
    output wire                                 DEBUG_OUT_SEL,

    input wire [535:0]                          SCAN_OUT
);

    // Wires between sync_fifo & rare_heater_control_s_axi
    // AW Channel
    wire AWVALID_sync;
    wire AWREADY_sync;
    wire [C_S_AXI_ADDR_WIDTH - 1 : 0] AWADDR_sync;
    // W Channel
    wire WVALID_sync;
    wire WREADY_sync;
    wire [C_S_AXI_DATA_WIDTH - 1 : 0] WDATA_sync;
    // AR Channel
    wire ARVALID_sync;
    wire ARREADY_sync;
    wire [C_S_AXI_ADDR_WIDTH - 1 : 0] ARADDR_sync;
    // R Channel
    wire RVALID_sync;
    wire RREADY_sync;
    wire [C_S_AXI_DATA_WIDTH - 1 : 0] RDATA_sync;

    // --------------------------------------------------------------------
    // 00000 00000 00000  000
    // 0       0   0     0   0
    // 00000   0   00000 0   0
    // 0       0   0     0   0
    // 0     00000 0      000
    // --------------------------------------------------------------------

    // AW FIFO
    sync_fifo # (
        .FIFO_IN_REG(FIFO_IN_REG),
        .FIFO_OUT_REG(FIFO_OUT_REG),
        .FIFO_CMD_LENGTH(C_S_AXI_ADDR_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH)
    ) sync_fifo_AW (
        .clk(ACLK),
        .resetn(ARESETB),
        .s_valid(AWVALID),
        .s_ready(AWREADY),
        .s_data(AWADDR),
        .m_valid(AWVALID_sync),
        .m_ready(AWREADY_sync),
        .m_data(AWADDR_sync)
    );

    // W FIFO
    sync_fifo # (
        .FIFO_IN_REG(FIFO_IN_REG),
        .FIFO_OUT_REG(FIFO_OUT_REG),
        .FIFO_CMD_LENGTH(C_S_AXI_DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH)
    ) sync_fifo_W (
        .clk(ACLK),
        .resetn(ARESETB),
        .s_valid(WVALID),
        .s_ready(WREADY),
        .s_data(WDATA),
        .m_valid(WVALID_sync),
        .m_ready(WREADY_sync),
        .m_data(WDATA_sync)
    );

    // AR FIFO
    sync_fifo # (
        .FIFO_IN_REG(FIFO_IN_REG),
        .FIFO_OUT_REG(FIFO_OUT_REG),
        .FIFO_CMD_LENGTH(C_S_AXI_ADDR_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH)
    ) sync_fifo_AR (
        .clk(ACLK),
        .resetn(ARESETB),
        .s_valid(ARVALID),
        .s_ready(ARREADY),
        .s_data(ARADDR),
        .m_valid(ARVALID_sync),
        .m_ready(ARREADY_sync),
        .m_data(ARADDR_sync)
    );

    // R FIFO
    sync_fifo # (
        .FIFO_IN_REG(FIFO_IN_REG),
        .FIFO_OUT_REG(FIFO_OUT_REG),
        .FIFO_CMD_LENGTH(C_S_AXI_DATA_WIDTH),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH)
    ) sync_fifo_R (
        .clk(ACLK),
        .resetn(ARESETB),
        .s_valid(RVALID_sync),
        .s_ready(RREADY_sync),
        .s_data(RDATA_sync),
        .m_valid(RVALID),
        .m_ready(RREADY),
        .m_data(RDATA)
    );

    // --------------------------------------------------------------------
    //   0   0   0 00000
    //  0 0   0 0    0
    // 00000   0     0
    // 0   0  0 0    0
    // 0   0 0   0 00000
    // --------------------------------------------------------------------

    rare_heater_control_s_axi # (
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .NUM_ODOMETER(NUM_ODOMETER),
        .NUM_ODOMETER_LOG2(NUM_ODOMETER_LOG2),
        .NUM_ODOMETER_P(NUM_ODOMETER_P),
        .NUM_ODOMETER_S(NUM_ODOMETER_S),
        //.NUM_HEATER_TOP(NUM_HEATER_TOP),
        .NUM_HEATER_P(NUM_HEATER_P),
        .NUM_HEATER_S(NUM_HEATER_S),
        .DDLS_WIDTH(DDLS_WIDTH)
    ) rare_heater_control_s_axi_inst (
        // Chip IO Ports
        .ACLK(ACLK),
        .ARESETB(ARESETB),
        .ACLK_EN(1'b1),
        // AW Channel
        .AWADDR(AWADDR_sync),
        .AWVALID(AWVALID_sync),
        .AWREADY(AWREADY_sync),
        // W Channel
        .WDATA(WDATA_sync),
        .WVALID(WVALID_sync),
        .WREADY(WREADY_sync),
        // B Channel
        .BREADY(1'b1),
        // AR Channel
        .ARADDR(ARADDR_sync),
        .ARVALID(ARVALID_sync),
        .ARREADY(ARREADY_sync),       
        // R Channel
        .RDATA(RDATA_sync),
        .RVALID(RVALID_sync),
        .RREADY(RREADY_sync),

        .CLK_SELECT(CLK_SELECT),
        .FREQ_OUT_EN(FREQ_OUT_EN),

        .DCO_EN(DCO_EN),
        .DCO_COARSE(DCO_COARSE),
        .DCO_FINE(DCO_FINE),
        
        .FREQ_OUT_CNT_CLK_SELECT(FREQ_OUT_CNT_CLK_SELECT),
        .FREQ_OUT_CNT_RESETB(FREQ_OUT_CNT_RESETB),
        .FREQ_OUT_CNT_START(FREQ_OUT_CNT_START),
        .FREQ_OUT_CNT_DONE(FREQ_OUT_CNT_DONE),

        .STREAM_ADDR_DDLS(STREAM_ADDR_DDLS),
        .STREAM_DATA_IN_DDLS(STREAM_DATA_IN_DDLS),
        .STREAM_DATA_OUT_DDLS(STREAM_DATA_OUT_DDLS),
        .STREAM_WE_DDLS(STREAM_WE_DDLS),
        .STREAM_EN_DDLS(STREAM_EN_DDLS),

        .STREAM_ADDR_INSTR(STREAM_ADDR_INSTR),
        .STREAM_DATA_IN_INSTR(STREAM_DATA_IN_INSTR),
        .STREAM_DATA_OUT_INSTR(STREAM_DATA_OUT_INSTR),
        .STREAM_WE_INSTR(STREAM_WE_INSTR),
        .STREAM_EN_INSTR(STREAM_EN_INSTR),

        .STREAM_ADDR_DATA(STREAM_ADDR_DATA),
        .STREAM_DATA_IN_DATA(STREAM_DATA_IN_DATA),
        .STREAM_DATA_OUT_DATA(STREAM_DATA_OUT_DATA),
        .STREAM_WE_DATA(STREAM_WE_DATA),
        .STREAM_EN_DATA(STREAM_EN_DATA),

        .DDLS_DELAY_SEL(DDLS_DELAY_SEL),
        .DDLS_RESETB(DDLS_RESETB),
        .CLK_CNT_RESETB(CLK_CNT_RESETB),
        // RISCV Control
        .RISCV_RESETB(RISCV_RESETB),
        .RISCV_CLK_EN(RISCV_CLK_EN),
        .RISCV_RUN_CYCLE_ENABLE(RISCV_RUN_CYCLE_ENABLE),
        .RISCV_RUN_CYCLE(RISCV_RUN_CYCLE),
        .RISCV_RUN_DONE(RISCV_RUN_DONE),
        .RISCV_RUN_DONE_PRIMARY(RISCV_RUN_DONE_PRIMARY),
        .RISCV_RUN_DONE_SECONDARY(RISCV_RUN_DONE_SECONDARY),

        .CORE_CYCLE_COUNT_PRIMARY(CORE_CYCLE_COUNT_PRIMARY),
        .CORE_CYCLE_COUNT_SECONDARY(CORE_CYCLE_COUNT_SECONDARY),

        .CORE_SLEEP_O_PRIMARY(CORE_SLEEP_O_PRIMARY),
        .CORE_SLEEP_O_SECONDARY(CORE_SLEEP_O_SECONDARY),

        .RISCV_ERR_OVERFLOW(RISCV_ERR_OVERFLOW),
        .RISCV_64_CNT_OVERFLOW(RISCV_64_CNT_OVERFLOW),
        .RISCV_ERRCNT(RISCV_ERRCNT),

        .STREAM_SEL(STREAM_SEL),
        .FETCH_ENABLE(FETCH_ENABLE),
        .BOOT_ADDR(BOOT_ADDR),
        .MTVEC_ADDR(MTVEC_ADDR),

        // To heater
        .HEATER_GLOBAL_ENABLE(HEATER_GLOBAL_ENABLE),
        //.HEATER_ENABLE_TOP(HEATER_ENABLE_TOP),
        //.HEATER_LOAD_TOP(HEATER_LOAD_TOP),
        .HEATER_ENABLE_PRIMARY(HEATER_ENABLE_PRIMARY),
        .HEATER_LOAD_PRIMARY(HEATER_LOAD_PRIMARY),
        .HEATER_ENABLE_SECONDARY(HEATER_ENABLE_SECONDARY),
        .HEATER_LOAD_SECONDARY(HEATER_LOAD_SECONDARY),

        .TEMP_SENSOR_SEL(TEMP_SENSOR_SEL),
        .TEMP_SENSOR_EN(TEMP_SENSOR_EN),
        //.VOLTAGE_SENSOR_SEL(VOLTAGE_SENSOR_SEL),       
        //.VOLTAGE_SENSOR_EN(VOLTAGE_SENSOR_EN),

        // To odometer
        .ODOMETER_SEL(ODOMETER_SEL),
        .ODOMETER_ENABLE_PRIMARY(ODOMETER_ENABLE_PRIMARY),
        .ODOMETER_ENABLE_SECONDARY(ODOMETER_ENABLE_SECONDARY),
        .ODOMETER_ENABLE_DCO(ODOMETER_ENABLE_DCO),

        .ODOMETER_RESETB(ODOMETER_RESETB),


        .STRESS(STRESS),
        .AC_DC(AC_DC),
        .SEL_INV(SEL_INV),
        .SEL_NAND(SEL_NAND),
        .SEL_NOR(SEL_NOR),
        .ODOMETER_MEAS_TRIG(ODOMETER_MEAS_TRIG),
        .ODOMETER_LOAD(ODOMETER_LOAD),
        .BIT_COUNT(BIT_COUNT),

        .FREQ_OUT_SEL(FREQ_OUT_SEL),
        .DEBUG_OUT_SEL(DEBUG_OUT_SEL),

        .SCAN_OUT(SCAN_OUT)
    );

endmodule
