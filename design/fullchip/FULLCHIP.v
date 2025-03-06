`timescale 1 ns / 1 ps

module FULLCHIP # (
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
    // input from Pad Port
    input ACLK_IO,
    input ARESETB_IO,
    input AWRITEB_IO,
    input [5:0] AADDR_IO,
    input AVALID_IO,

    input [7:0] WDATA_IO,
    input WVALID_IO,

    input RREADY_IO,
    
    input HEATER_PWM_A_IO,
    input HEATER_PWM_B_IO,
    
    // output to Pad Port
    output wire AREADY_IO,
    output wire WREADY_IO,
    output wire [7:0] RDATA_IO,
    output wire RVALID_IO,

    output wire DEBUG_OUT_IO,
    output wire DCO_FREQ_OUT_IO,

    // inout bare pad
    inout TEMP_SOURCE_HI_IO,
    inout TEMP_SOURCE_LO_IO,
    inout TEMP_SENSE_HI_IO,
    inout TEMP_SENSE_LO_IO,
    

    // power
    input wire VDD0P9_TOP,
    input wire VDD0P9_CORE1,
    input wire VDD0P9_CORE2,
    input wire VSS0P9,
    input wire VDD1P8,
    input wire VSS1P8   
);


    wire ACLK;
    wire ARESETB;
    wire AWRITEB;
    wire [C_S_AXI_ADDR_WIDTH - 1 : 0] AADDR;
    wire AVALID;
    wire AREADY;
    wire [C_S_AXI_DATA_WIDTH - 1 : 0] WDATA;
    wire WVALID;
    wire WREADY;
    wire [C_S_AXI_DATA_WIDTH - 1 : 0] RDATA;
    wire RVALID;
    wire RREADY;
    wire HEATER_PWM_A;
    wire HEATER_PWM_B;
    wire SENSE_HI;
    wire SENSE_LO;
    wire SOURCE_HI;
    wire SOURCE_LO;
    wire DCO_FREQ_OUT;
    wire DEBUG_OUT;
    wire TEMP_SOURCE_HI;
    wire TEMP_SOURCE_LO;
    wire TEMP_SENSE_HI;
    wire TEMP_SENSE_LO;

    io_conn io_conn_inst (
        .ACLK_IO(ACLK_IO),
        .ARESETB_IO(ARESETB_IO),
        .AWRITEB_IO(AWRITEB_IO),
        .AADDR_IO(AADDR_IO),
        .AVALID_IO(AVALID_IO),
        .WDATA_IO(WDATA_IO),
        .WVALID_IO(WVALID_IO),
        .RREADY_IO(RREADY_IO),
        .HEATER_PWM_A_IO(HEATER_PWM_A_IO),
        .HEATER_PWM_B_IO(HEATER_PWM_B_IO),
        .AREADY_IO(AREADY_IO),
        .WREADY_IO(WREADY_IO),
        .RDATA_IO(RDATA_IO),
        .RVALID_IO(RVALID_IO),
        .DEBUG_OUT_IO(DEBUG_OUT_IO),
        .DCO_FREQ_OUT_IO(DCO_FREQ_OUT_IO),
        .TEMP_SOURCE_HI_IO(TEMP_SOURCE_HI_IO),
        .TEMP_SOURCE_LO_IO(TEMP_SOURCE_LO_IO),
        .TEMP_SENSE_HI_IO(TEMP_SENSE_HI_IO),
        .TEMP_SENSE_LO_IO(TEMP_SENSE_LO_IO),

        .AREADY(AREADY),
        .WREADY(WREADY),
        .RDATA(RDATA),
        .RVALID(RVALID),
        .DEBUG_OUT(DEBUG_OUT),
        .DCO_FREQ_OUT(DCO_FREQ_OUT),
        .ACLK(ACLK),
        .ARESETB(ARESETB),
        .AWRITEB(AWRITEB),
        .AADDR(AADDR),
        .AVALID(AVALID),
        .WDATA(WDATA),
        .WVALID(WVALID),
        .RREADY(RREADY),
        .HEATER_PWM_A(HEATER_PWM_A),
        .HEATER_PWM_B(HEATER_PWM_B),
        .TEMP_SOURCE_HI(TEMP_SOURCE_HI),
        .TEMP_SOURCE_LO(TEMP_SOURCE_LO),
        .TEMP_SENSE_HI(TEMP_SENSE_HI),
        .TEMP_SENSE_LO(TEMP_SENSE_LO),

        .VDD0P9_TOP(VDD0P9_TOP),
        .VDD0P9_CORE1(VDD0P9_CORE1),
        .VDD0P9_CORE2(VDD0P9_CORE2),
        .VSS0P9(VSS0P9),
        .VDD1P8(VDD1P8),
        .VSS1P8(VSS1P8)
    );


    RARECHIP # (
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH),
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .FIFO_IN_REG(FIFO_IN_REG),
        .FIFO_OUT_REG(FIFO_OUT_REG),
        .FIFO_DEPTH(FIFO_DEPTH),
        .FIFO_LOG2_DEPTH(FIFO_LOG2_DEPTH),
        .COREV_PULP(COREV_PULP),
        .COREV_CLUSTER(COREV_CLUSTER),
        .FPU(FPU),
        .FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
        .FPU_OTHERS_LAT(FPU_OTHERS_LAT),
        .ZFINX(ZFINX),
        .NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
        .INPUTSIZE(INPUTSIZE),
        .BUFFERSIZE(BUFFERSIZE),
        .BUFFERWIDTH(BUFFERWIDTH),
        .NUM_HEATER_P(NUM_HEATER_P),
        .NUM_HEATER_S(NUM_HEATER_S),
        .NUM_TEMP_SENSOR_PRIMARY(NUM_TEMP_SENSOR_PRIMARY),
        .NUM_ODOMETER_P(NUM_ODOMETER_P),
        .NUM_ODOMETER_S(NUM_ODOMETER_S)
    ) rarechip_inst (
        .ACLK(ACLK),
        .ARESETB(ARESETB),
        .A_WRITE_READ_SEL(AWRITEB),
        .AADDR(AADDR),
        .AVALID(AVALID),
        .AREADY(AREADY),
        .WDATA(WDATA),
        .WVALID(WVALID),
        .WREADY(WREADY),
        .RDATA(RDATA),
        .RVALID(RVALID),
        .RREADY(RREADY),
        .HEATER_PWM_A(HEATER_PWM_A),
        .HEATER_PWM_B(HEATER_PWM_B),
        .SENSE_HI(TEMP_SENSE_HI),
        .SENSE_LO(TEMP_SENSE_LO),
        .SOURCE_HI(TEMP_SOURCE_HI),
        .SOURCE_LO(TEMP_SOURCE_LO),
        .FREQ_OUT(DCO_FREQ_OUT),
        .RISCV_DEBUG_OUT(DEBUG_OUT),
        .VDD(VDD0P9_TOP),
        .VDD0P9_CORE1(VDD0P9_CORE1),
        .VDD0P9_CORE2(VDD0P9_CORE2),
        .VSS(VSS0P9)
    );


endmodule
