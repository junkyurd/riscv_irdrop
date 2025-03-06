`timescale 1 ns / 1 ps

module RARECHIP # (
    // AXIL
    parameter C_S_AXI_ADDR_WIDTH = 6,
    parameter C_S_AXI_DATA_WIDTH = 8,
    
    // SYNC_FIFO
    parameter FIFO_IN_REG = 1,
    parameter FIFO_OUT_REG = 1,
    parameter FIFO_DEPTH = 2,
    parameter FIFO_LOG2_DEPTH = 1,

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
    parameter BUFFERWIDTH = 256,

    // Heater paramters
    //parameter NUM_HEATER_TOP = 64,
    parameter NUM_HEATER_P = 84,
    parameter NUM_HEATER_S = 84,

    parameter NUM_TEMP_SENSOR_PRIMARY = 18,
    parameter NUM_TEMP_SENSOR_SECONDARY = 18,

    // Odometer parameters
    parameter NUM_ODOMETER_P = 21,
    parameter NUM_ODOMETER_S = 21
)(
    // ACLK_EN and BREADY at AXI is tied high
    //AXI CONTROL
    input wire                                  ACLK,
    input wire                                  ARESETB, // assync neg reset
    // input wire                                  ACLK_EN, always 1
    // write address
    input wire                                  A_WRITE_READ_SEL, // if 0 AW selected, 0 AR selected
    input wire [C_S_AXI_ADDR_WIDTH - 1 : 0]     AADDR,
    input wire                                  AVALID,
    output reg                                  AREADY,
    //input wire [C_S_AXI_ADDR_WIDTH - 1 : 0]     AWADDR,
    //input wire                                  AWVALID,
    //output wire                                 AWREADY,
    // write data
    input wire [C_S_AXI_DATA_WIDTH - 1 : 0]     WDATA,
    //input wire [C_S_AXI_DATA_WIDTH/8 - 1 : 0]   WSTRB,
    input wire                                  WVALID,
    output wire                                 WREADY,
    // write resp
    //output wire [1:0]                           BRESP,
    //output wire                                 BVALID,    
    //input wire                                  BREADY,
    // read address
    //input wire [C_S_AXI_ADDR_WIDTH - 1 : 0]     ARADDR,
    //input wire                                  ARVALID,
    //output wire                                 ARREADY,
    // read data
    output wire [C_S_AXI_DATA_WIDTH - 1 : 0]    RDATA,
    //output wire [1:0]                           RRESP,
    output wire                                 RVALID,
    input wire                                  RREADY,

    // HEATER
    input wire                                  HEATER_PWM_A,
    input wire                                  HEATER_PWM_B,
    inout wire                                  SENSE_HI,
    inout wire                                  SENSE_LO,
    inout wire                                  SOURCE_HI,
    inout wire                                  SOURCE_LO,

    // CLK PAD out
    output wire                                 FREQ_OUT,

    // Debug pins
    output wire                                 RISCV_DEBUG_OUT,

    input                                       VDD,
    input                                       VDD0P9_CORE1,
    input                                       VDD0P9_CORE2,
    input                                       VSS
);

localparam NUM_ODOMETER = NUM_ODOMETER_P + NUM_ODOMETER_S + 1; // 1 for odometer dco control
localparam NUM_TEMP_SENSOR = NUM_TEMP_SENSOR_PRIMARY + NUM_TEMP_SENSOR_SECONDARY;

// --------------------------------------------------------------------
// 0000  00000   0   0000  0   0 0000  00000 00000 00000
// 0   0 0      0 0  0   0 0   0 0   0   0     0   0
// 0000  00000 00000 0   0 0 0 0 0000    0     0   00000 
// 0   0 0     0   0 0   0 0 0 0 0   0   0     0   0
// 0   0 00000 0   0 0000   0 0  0   0 00000   0   00000
// --------------------------------------------------------------------
reg [C_S_AXI_ADDR_WIDTH - 1 : 0]     AWADDR;
reg                                  AWVALID;
wire                                 AWREADY;
reg [C_S_AXI_ADDR_WIDTH - 1 : 0]     ARADDR;
reg                                  ARVALID;
wire                                 ARREADY;
wire                                 CLK_OUT_FREQ_OUT;
wire                                 ODOMETER_VCO_PAD_OUT;
wire                                 FREQ_OUT_SEL;
wire                                 RISCV_DEBUG_OUT_PRIMARY;
wire                                 RISCV_DEBUG_OUT_SECONDARY;
wire                                 DEBUG_OUT_SEL;

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        AWADDR <= 0;
        AWVALID <= 1'b0;
        ARADDR <= 0;
        ARVALID <= 1'b0;
    end else begin
        if (A_WRITE_READ_SEL == 1'b0) begin
            AWADDR <= AADDR;
            AWVALID <= AVALID;
            ARADDR <= 0;
            ARVALID <= 1'b0;
        end else begin
            AWADDR <= 0;
            AWVALID <= 1'b0;
            ARADDR <= AADDR;
            ARVALID <= AVALID;
        end
    end
end

// AREADY and FREQ_OUT MUX
fullchip_out_mux fullchip_out_mux_inst(
    .a_write_read_sel(A_WRITE_READ_SEL),
    .awready(AWREADY),
    .arready(ARREADY),
    .aready(AREADY),

    .freq_out_sel(FREQ_OUT_SEL),
    .clk_out_freq_out(CLK_OUT_FREQ_OUT),
    .odometer_vco_pad_out(ODOMETER_VCO_PAD_OUT),
    .freq_out(FREQ_OUT),

    .debug_out_sel(DEBUG_OUT_SEL),
    .riscv_debug_out_primary(RISCV_DEBUG_OUT_PRIMARY),
    .riscv_debug_out_secondary(RISCV_DEBUG_OUT_SECONDARY),
    .riscv_debug_out(RISCV_DEBUG_OUT)
);

/*
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        AREADY <= 1'b0;
    end else begin
        if (!A_WRITE_READ_SEL) begin
            AREADY <= AWREADY;
        end else begin
            AREADY <= ARREADY;
        end
    end
end
*/



// Internal signals
// --------------------------------------------------------------------
// 00000 0   0 00000 00000 0000  0   0   0   0
//   0   00  0   0   0     0   0 00  0  0 0  0
//   0   0 0 0   0   00000 0000  0 0 0 00000 0
//   0   0  00   0   0     0   0 0  00 0   0 0
// 00000 0   0   0   00000 0   0 0   0 0   0 00000
// --------------------------------------------------------------------
// Clock control
wire CLK_SELECT;
wire FREQ_OUT_EN;

wire DCO_EN;
wire [3:0] DCO_COARSE;
wire [3:0] DCO_FINE;

wire FREQ_OUT_CNT_CLK_SELECT;
wire FREQ_OUT_CNT_RESETB;
wire FREQ_OUT_CNT_START;
wire FREQ_OUT_CNT_DONE;

// --------------------------------------------------------------------
// RISC-V Memory

// DDLS CACHE
wire [31:0] STREAM_ADDR_DDLS;
wire [BUFFERWIDTH - 1 : 0] STREAM_DATA_IN_DDLS;
wire [BUFFERWIDTH - 1 : 0] STREAM_DATA_OUT_DDLS;
wire STREAM_WE_DDLS;
wire STREAM_EN_DDLS;

// INSTR CACHE
wire [31:0] STREAM_ADDR_INSTR;
wire [31:0] STREAM_DATA_IN_INSTR;
wire [31:0] STREAM_DATA_OUT_INSTR;
wire STREAM_WE_INSTR;
wire STREAM_EN_INSTR;

wire [10:0] INSTR_ADDR;
wire INSTR_WRITE_ENABLE;
wire INSTR_ENABLE;
wire [31:0] INSTR_DATA_write;
wire [31:0] INSTR_DATA_read;
// DATA CACHE
wire [31:0] STREAM_ADDR_DATA;
wire [31:0] STREAM_DATA_IN_DATA;
wire [31:0] STREAM_DATA_OUT_DATA;
wire STREAM_WE_DATA;
wire STREAM_EN_DATA;

wire [10:0] DATA_ADDR_P, DATA_ADDR_S;
wire DATA_WRITE_ENABLE_P, DATA_WRITE_ENABLE_S;
wire DATA_ENABLE_P, DATA_ENABLE_S;
wire [31:0] DATA_DATA_write_P, DATA_DATA_write_S;
wire [31:0] DATA_DATA_read_P, DATA_DATA_read_S;

// RISC-V Block Control
wire RISCV_RESETB;
wire RISCV_CLK_EN;
wire RISCV_RUN_CYCLE_ENABLE;
wire [63:0] RISCV_RUN_CYCLE;
wire RISCV_RUN_DONE;
wire RISCV_RUN_DONE_PRIMARY;
wire RISCV_RUN_DONE_SECONDARY;
wire RISCV_ERR_OVERFLOW;
wire RISCV_64_CNT_OVERFLOW;
wire [63:0] RISCV_ERRCNT;

wire [63:0] CORE_CYCLE_COUNT_PRIMARY;
wire [63:0] CORE_CYCLE_COUNT_SECONDARY;

wire [1:0] STREAM_SEL;
wire FETCH_ENABLE;

wire [31:0] BOOT_ADDR;
wire [31:0] MTVEC_ADDR;
// DDLS
wire [7:0] DDLS_DELAY_SEL;
wire DDLS_RESETB;
wire CLK_CNT_RESETB;

wire CORE_SLEEP_O_PRIMARY;
wire CORE_SLEEP_O_SECONDARY;

// Heater
wire HEATER_GLOBAL_ENABLE;
//wire [NUM_HEATER_TOP - 1 :0] HEATER_ENABLE_TOP;
//reg [NUM_HEATER_TOP - 1 : 0] HEATER_ENABLE_TOP_reg;
wire HEATER_LOAD_TOP;
wire [NUM_HEATER_P - 1 : 0] HEATER_ENABLE_PRIMARY;
reg [NUM_HEATER_P - 1 : 0] HEATER_ENABLE_PRIMARY_reg;
wire HEATER_LOAD_PRIMARY;
wire [NUM_HEATER_S - 1 : 0] HEATER_ENABLE_SECONDARY;
reg [NUM_HEATER_S - 1 : 0] HEATER_ENABLE_SECONDARY_reg;
wire HEATER_LOAD_SECONDARY;

wire [7:0] TEMP_SENSOR_SEL;
wire TEMP_SENSOR_EN;
reg [NUM_TEMP_SENSOR - 1 : 0] TEMP_SENSOR_DECODE;
//wire [7:0] VOLTAGE_SENSOR_SEL;
//reg [NUM_VOLTAGE_SENSOR - 1 : 0] VOLTAGE_SENSOR_DECODE;
//wire VOLTAGE_SENSOR_EN;
// --------------------------------------------------------------------
// ODOMETER
localparam NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER);
wire [NUM_ODOMETER_LOG2 - 1:0] ODOMETER_SEL;
wire ODOMETER_ENABLE_PRIMARY;
wire ODOMETER_ENABLE_SECONDARY;
wire ODOMETER_ENABLE_DCO;


wire ODOMETER_RESETB;
wire STRESS_internal;
wire AC_DC_internal;
wire SEL_INV_internal;
wire SEL_NAND_internal;
wire SEL_NOR_internal;
wire [15:0] BIT_COUNT_internal;

//wire [NUM_ODOMETER - 1 : 0] STRESS;
//wire [NUM_ODOMETER - 1 : 0] AC_DC;
//wire [NUM_ODOMETER - 1 : 0] SEL_INV;
//wire [NUM_ODOMETER - 1 : 0] SEL_NAND;
//wire [NUM_ODOMETER - 1 : 0] SEL_NOR;
//wire ODOMETER_MEAS_TRIG;
//wire ODOMETER_LOAD;
//wire [15:0] BIT_COUNT [NUM_ODOMETER - 1 : 0];
// --------------------------------------------------------------------
wire [535:0] SCAN_OUT;


// Loading of heater signals
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        //HEATER_ENABLE_TOP_reg <= 0;
        HEATER_ENABLE_PRIMARY_reg <= 0;
        HEATER_ENABLE_SECONDARY_reg <= 0;
    end else begin
        //if (HEATER_LOAD_TOP) begin
        //    HEATER_ENABLE_TOP_reg <= HEATER_ENABLE_TOP;
        //end else begin
        //    HEATER_ENABLE_TOP_reg <= HEATER_ENABLE_TOP_reg;
        //end

        if (HEATER_LOAD_PRIMARY) begin
            HEATER_ENABLE_PRIMARY_reg <= HEATER_ENABLE_PRIMARY;
        end else begin
            HEATER_ENABLE_PRIMARY_reg <= HEATER_ENABLE_PRIMARY_reg;
        end

        if (HEATER_LOAD_SECONDARY) begin
            HEATER_ENABLE_SECONDARY_reg <= HEATER_ENABLE_SECONDARY;
        end else begin
            HEATER_ENABLE_SECONDARY_reg <= HEATER_ENABLE_SECONDARY_reg;
        end

    end
end

// Temp sensor decoder
integer t_sensor;
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        TEMP_SENSOR_DECODE <= 0;
    end else if (TEMP_SENSOR_EN) begin
        for (t_sensor = 0; t_sensor < NUM_TEMP_SENSOR; t_sensor = t_sensor + 1) begin
            if (TEMP_SENSOR_SEL == t_sensor) begin
                TEMP_SENSOR_DECODE[t_sensor] <= 1'b1;
            end else begin
                TEMP_SENSOR_DECODE[t_sensor] <= 1'b0;
            end
        end 
    end else begin
        TEMP_SENSOR_DECODE <= 0;
    end
end
/*
// Voltage sensor decoder
integer v_sensor;
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        VOLTAGE_SENSOR_DECODE <= 0;
    end else begin
        for (v_sensor = 0; v_sensor < NUM_VOLTAGE_SENSOR; v_sensor = v_sensor + 1) begin
            if (VOLTAGE_SENSOR_SEL == v_sensor) begin
                VOLTAGE_SENSOR_DECODE[v_sensor] <= 1'b1;
            end else begin
                VOLTAGE_SENSOR_DECODE[v_sensor] <= 1'b0;
            end
        end
    end
end
*/

// --------------------------------------------------------------------
//   0   0   0 00000       0
//  0 0   0 0    0         0
// 00000   0     0   00000 0
// 0   0  0 0    0         0
// 0   0 0   0 00000       00000
// --------------------------------------------------------------------
buffered_rare_heater_control_s_axi # (
    .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
    .NUM_ODOMETER(NUM_ODOMETER),
    .NUM_ODOMETER_LOG2(NUM_ODOMETER_LOG2),
    .NUM_ODOMETER_P(NUM_ODOMETER_P),
    .NUM_ODOMETER_S(NUM_ODOMETER_S),
    //.NUM_HEATER_TOP(NUM_HEATER_TOP),
    .NUM_HEATER_P(NUM_HEATER_P),
    .NUM_HEATER_S(NUM_HEATER_S),
    .DDLS_WIDTH(BUFFERWIDTH),
    .FIFO_IN_REG(FIFO_IN_REG),
    .FIFO_OUT_REG(FIFO_OUT_REG),
    .FIFO_DEPTH(FIFO_DEPTH),
    .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH)
) buffered_rare_heater_control_s_axi_inst (
    // Chip IO ports
    .ACLK(ACLK),
    .ARESETB(ARESETB),
    .ACLK_EN(1'b1),
    .AWADDR(AWADDR),
    .AWVALID(AWVALID),
    .AWREADY(AWREADY),
    .WDATA(WDATA),
    .WVALID(WVALID),
    .WREADY(WREADY),
    .ARADDR(ARADDR),
    .ARVALID(ARVALID),
    .ARREADY(ARREADY),
    .RDATA(RDATA),
    .RVALID(RVALID),
    .RREADY(RREADY),

    // Internal signals
    // Clock control
    .CLK_SELECT(CLK_SELECT),
    .FREQ_OUT_EN(FREQ_OUT_EN),

    .DCO_EN(DCO_EN),
    .DCO_COARSE(DCO_COARSE),
    .DCO_FINE(DCO_FINE),

    .FREQ_OUT_CNT_CLK_SELECT(FREQ_OUT_CNT_CLK_SELECT),
    .FREQ_OUT_CNT_RESETB(FREQ_OUT_CNT_RESETB),
    .FREQ_OUT_CNT_START(FREQ_OUT_CNT_START),
    .FREQ_OUT_CNT_DONE(FREQ_OUT_CNT_DONE),

    // DDLS memory
    .STREAM_ADDR_DDLS(STREAM_ADDR_DDLS),
    .STREAM_DATA_IN_DDLS(STREAM_DATA_IN_DDLS),
    .STREAM_DATA_OUT_DDLS(STREAM_DATA_OUT_DDLS),
    .STREAM_WE_DDLS(STREAM_WE_DDLS),
    .STREAM_EN_DDLS(STREAM_EN_DDLS),

    // Instruction memory
    .STREAM_ADDR_INSTR(STREAM_ADDR_INSTR),
    .STREAM_DATA_IN_INSTR(STREAM_DATA_IN_INSTR),
    .STREAM_DATA_OUT_INSTR(STREAM_DATA_OUT_INSTR),
    .STREAM_WE_INSTR(STREAM_WE_INSTR),
    .STREAM_EN_INSTR(STREAM_EN_INSTR),

    // Data memory
    .STREAM_ADDR_DATA(STREAM_ADDR_DATA),
    .STREAM_DATA_IN_DATA(STREAM_DATA_IN_DATA),
    .STREAM_DATA_OUT_DATA(STREAM_DATA_OUT_DATA),
    .STREAM_WE_DATA(STREAM_WE_DATA),
    .STREAM_EN_DATA(STREAM_EN_DATA),

    // Delayed dual lockstep
    .DDLS_DELAY_SEL(DDLS_DELAY_SEL),
    .DDLS_RESETB(DDLS_RESETB),
    .CLK_CNT_RESETB(CLK_CNT_RESETB),

    // RISC-V Control
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

    // Heater
    .HEATER_GLOBAL_ENABLE(HEATER_GLOBAL_ENABLE),
    //.HEATER_ENABLE_TOP(HEATER_ENABLE_TOP),
    //.HEATER_LOAD_TOP(HEATER_LOAD_TOP),
    .HEATER_ENABLE_PRIMARY(HEATER_ENABLE_PRIMARY),
    .HEATER_LOAD_PRIMARY(HEATER_LOAD_PRIMARY),
    .HEATER_ENABLE_SECONDARY(HEATER_ENABLE_SECONDARY),
    .HEATER_LOAD_SECONDARY(HEATER_LOAD_SECONDARY),

    .TEMP_SENSOR_SEL(TEMP_SENSOR_SEL),
    .TEMP_SENSOR_EN(TEMP_SENSOR_EN),

    // Odometer
    .ODOMETER_SEL(ODOMETER_SEL),
    .ODOMETER_ENABLE_PRIMARY(ODOMETER_ENABLE_PRIMARY),
    .ODOMETER_ENABLE_SECONDARY(ODOMETER_ENABLE_SECONDARY),
    .ODOMETER_ENABLE_DCO(ODOMETER_ENABLE_DCO),

    .ODOMETER_RESETB(ODOMETER_RESETB),
    .STRESS(STRESS_internal),
    .AC_DC(AC_DC_internal),
    .SEL_INV(SEL_INV_internal),
    .SEL_NAND(SEL_NAND_internal),
    .SEL_NOR(SEL_NOR_internal),
    .ODOMETER_MEAS_TRIG(ODOMETER_MEAS_TRIG),
    .ODOMETER_LOAD(ODOMETER_LOAD),
    .BIT_COUNT(BIT_COUNT_internal),

    .FREQ_OUT_SEL(FREQ_OUT_SEL),
    .DEBUG_OUT_SEL(DEBUG_OUT_SEL),
    
    .SCAN_OUT(SCAN_OUT)
);

// --------------------------------------------------------------------
// 0000    0   0000  00000       00000  000  0000
// 0   0  0 0  0   0 0             0   0   0 0   0
// 0000  00000 0000  00000         0   0   0 0000
// 0   0 0   0 0   0 0             0   0   0 0
// 0   0 0   0 0   0 00000         0    000  0
// --------------------------------------------------------------------

top_RARE # (
    .COREV_PULP(COREV_PULP),
    .COREV_CLUSTER(COREV_CLUSTER),
    .FPU(FPU),
    .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
    .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
    .ZFINX(ZFINX),
    .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),

    //DDLS
    .INPUTSIZE(INPUTSIZE),
    .BUFFERSIZE(BUFFERSIZE),
    .BUFFERWIDTH(BUFFERWIDTH),

    // Heater
    .NUM_HEATER_P(NUM_HEATER_P),
    .NUM_HEATER_S(NUM_HEATER_S),

    .NUM_TEMP_SENSOR_PRIMARY(NUM_TEMP_SENSOR_PRIMARY),
    .NUM_TEMP_SENSOR_SECONDARY(NUM_TEMP_SENSOR_SECONDARY),

    // Odometer
    .NUM_ODOMETER_LOG2(NUM_ODOMETER_LOG2),
    .NUM_ODOMETER_P(NUM_ODOMETER_P),
    .NUM_ODOMETER_S(NUM_ODOMETER_S)    
) top_RARE_inst (
    .stream_clk(ACLK),
    .resetb(ARESETB),
    .riscv_resetb(RISCV_RESETB),

    // Clock control
    .clk_select(CLK_SELECT),
    .freq_out_en(FREQ_OUT_EN),

    .dco_en(DCO_EN),
    .dco_coarse(DCO_COARSE),
    .dco_fine(DCO_FINE),

    .freq_out_cnt_clk_select(FREQ_OUT_CNT_CLK_SELECT),
    .freq_out_cnt_resetb(FREQ_OUT_CNT_RESETB),
    .freq_out_cnt_start(FREQ_OUT_CNT_START),
    .freq_out_cnt_done(FREQ_OUT_CNT_DONE),
    .clk_out_freq_out(CLK_OUT_FREQ_OUT),

    // DDLS stream
    .stream_addr_ddls(STREAM_ADDR_DDLS),
    .stream_data_in_ddls(STREAM_DATA_IN_DDLS),
    .stream_data_out_ddls(STREAM_DATA_OUT_DDLS),
    .stream_we_ddls(STREAM_WE_DDLS),
    .stream_en_ddls(STREAM_EN_DDLS),

    .stream_select(STREAM_SEL),
    .fetch_enable(FETCH_ENABLE),

    .riscv_clk_en(RISCV_CLK_EN),
    .riscv_run_cycle(RISCV_RUN_CYCLE),
    .riscv_run_cycle_enable(RISCV_RUN_CYCLE_ENABLE),
    .riscv_run_done(RISCV_RUN_DONE),
    .riscv_run_done_primary(RISCV_RUN_DONE_PRIMARY),
    .riscv_run_done_secondary(RISCV_RUN_DONE_SECONDARY),

    .core_cycle_count_primary_reg(CORE_CYCLE_COUNT_PRIMARY),
    .core_cycle_count_secondary_reg(CORE_CYCLE_COUNT_SECONDARY),

    .core_sleep_o_primary(CORE_SLEEP_O_PRIMARY),
    .core_sleep_o_secondary(CORE_SLEEP_O_SECONDARY),

    .riscv_err_overflow(RISCV_ERR_OVERFLOW),
    .riscv_64_cnt_overflow(RISCV_64_CNT_OVERFLOW),
    .riscv_errcnt(RISCV_ERRCNT),

    // Instruction stream
    .stream_addr_instr(STREAM_ADDR_INSTR),
    .stream_data_in_instr(STREAM_DATA_IN_INSTR),
    .stream_data_out_instr(STREAM_DATA_OUT_INSTR),
    .stream_we_instr(STREAM_WE_INSTR),
    .stream_en_instr(STREAM_EN_INSTR),

    // Data stream
    .stream_addr_data(STREAM_ADDR_DATA),
    .stream_data_in_data(STREAM_DATA_IN_DATA),
    .stream_data_out_data(STREAM_DATA_OUT_DATA),
    .stream_we_data(STREAM_WE_DATA),
    .stream_en_data(STREAM_EN_DATA),

    // RISC-V boot
    .boot_addr_i(BOOT_ADDR),
    .mtvec_addr_i(MTVEC_ADDR),
    //.dm_halt_addr_i(32'b0),
    //.hart_id_i(32'b0),
    //.dm_exception_addr_i(32'b0),

    // DDLS
    .delay_sel_in(DDLS_DELAY_SEL[INPUTSIZE-1:0]),
    .ddls_resetb(DDLS_RESETB),
    .clk_cnt_resetb(CLK_CNT_RESETB),
    // HEATER ports
    .heater_pwm_a(HEATER_PWM_A),
    .heater_pwm_b(HEATER_PWM_B),
    .heater_global_enable(HEATER_GLOBAL_ENABLE),
    //.heater_enable_top(HEATER_ENABLE_TOP_reg),
    .heater_enable_primary(HEATER_ENABLE_PRIMARY_reg),
    .heater_enable_secondary(HEATER_ENABLE_SECONDARY_reg),
    .temp_sensor_decode(TEMP_SENSOR_DECODE),
    .temp_sensor_en(TEMP_SENSOR_EN),
    .sense_hi(SENSE_HI),
    .sense_lo(SENSE_LO),
    .source_hi(SOURCE_HI),
    .source_lo(SOURCE_LO),

    // Odometer ports
    .odometer_sel(ODOMETER_SEL),
    .odometer_enable_primary(ODOMETER_ENABLE_PRIMARY),
    .odometer_enable_secondary(ODOMETER_ENABLE_SECONDARY),
    .odometer_enable_dco(ODOMETER_ENABLE_DCO),

    .odometer_resetb(ODOMETER_RESETB),
    .stress(STRESS_internal),
    .ac_dc(AC_DC_internal),
    .sel_inv(SEL_INV_internal),
    .sel_nand(SEL_NAND_internal),
    .sel_nor(SEL_NOR_internal),
    .odometer_meas_trig(ODOMETER_MEAS_TRIG),
    .odometer_load(ODOMETER_LOAD),
    .bit_count(BIT_COUNT_internal),
    .odometer_vco_pad_out(ODOMETER_VCO_PAD_OUT),

    // Debug ports
    .riscv_debug_out_primary(RISCV_DEBUG_OUT_PRIMARY),
    .riscv_debug_out_secondary(RISCV_DEBUG_OUT_SECONDARY),

    .scan_out(SCAN_OUT),
    
    .VDD(VDD),
    .VDD0P9_CORE1(VDD0P9_CORE1),
    .VDD0P9_CORE2(VDD0P9_CORE2),
    .VSS(VSS)
);
/*
// dco clock
reg clk;
always begin
    clk = ~clk;
    #5;
end

top_circuit # (
    .COREV_PULP(COREV_PULP),
    .COREV_CLUSTER(COREV_CLUSTER),
    .FPU(FPU),
    .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
    .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
    .ZFINX(ZFINX),
    .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
    .INPUTSIZE(INPUTSIZE),
    .BUFFERSIZE(BUFFERSIZE),
    .BUFFERWIDTH(BUFFERWIDTH)
) top_circuit_inst (
    .clk(clk),
    .resetb(1'b0),

    .stream_clk(ACLK),
    
    .stream_instr_addr(INSTR_ADDR),
    .stream_instr_data_write(INSTR_DATA_write),
    .stream_instr_data_read(INSTR_DATA_read),
    .stream_instr_write_enable(INSTR_WRITE_ENABLE),
    .stream_instr_enable(INSTR_ENABLE),

    .boot_addr_i(32'b0),
    .mtvec_addr_i(32'b0),
    .dm_halt_addr_i(32'b0),
    .hart_id_i(32'b0),
    .dm_exception_addr_i(32'b0),
    .delay_sel_in(2'b01)
);
*/


endmodule
