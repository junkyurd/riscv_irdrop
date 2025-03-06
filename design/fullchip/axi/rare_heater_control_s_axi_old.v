// University of Minnesota
// Chris Kim VLSI Lab
// Authour: Junkyu Kim
// Data: 9/12/2023
// Description: axi4_lite control block for rare project

// reference and license
// reference 1: Xilinx AXIL_S IP
// reference 2: https://github.com/matbi86/matbi_fpga_season_1/blob/master/LICENSE
    // Design Name: matbi_dma_ip_control_s_axi
    // Module Name: matbi_dma_ip_control_s_axi
    // Project Name:
    // Target Devices: Zybo Z7-20
    // Tool Versions:
    // Description: matbi dma ip control.
    // Dependencies:

`timescale 1ns / 1ps

module rare_heater_control_s_axi # (
    parameter C_S_AXI_ADDR_WIDTH = 6,
    parameter C_S_AXI_DATA_WIDTH = 8,
    parameter NUM_ODOMETER = 100, // NUM_ODOMETER is a combined value of primary + secondary + dco control
    //parameter NUM_HEATER = 192, // Number of heaters at TOP, Primary, and Secondary combined
    parameter NUM_HEATER_TOP = 64, //64, each SEL is connected to 8 heaters which means total of 8*8 heaters are connected with this value
    parameter NUM_HEATER_P = 64, //64,
    parameter NUM_HEATER_S = 64, //64
    parameter DDLS_WIDTH = 100
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
    // write resp
    output wire [1:0]                           BRESP,
    output wire                                 BVALID,    
    input wire                                  BREADY,
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
    output reg                                  CLK_SELECT,
    output reg                                  FREQ_OUT_EN,

    output reg [3:0]                            DCO_COARSE,
    output reg [3:0]                            DCO_FINE,



    output reg                                  FREQ_OUT_CNT_CLK_SELECT,
    output reg                                  FREQ_OUT_CNT_RESETB,
    output reg                                  FREQ_OUT_CNT_START,
    input wire                                  FREQ_OUT_CNT_DONE,

    // I/F with Intel rare ddls caches
    output reg [31:0]                           STREAM_ADDR_DDLS,
    output reg [DDLS_WIDTH - 1 : 0]             STREAM_DATA_IN_DDLS,
    input wire [DDLS_WIDTH - 1 : 0]             STREAM_DATA_OUT_DDLS,
    output reg                                  STREAM_WE_DDLS,
    output reg                                  STREAM_EN_DDLS,

    // I/F with Intel rare instr caches
    output reg [31:0]                           STREAM_ADDR_INSTR,
    output reg [31:0]                           STREAM_DATA_IN_INSTR,
    input wire [31:0]                           STREAM_DATA_OUT_INSTR,
    output reg                                  STREAM_WE_INSTR,
    output reg                                  STREAM_EN_INSTR,

    // I/F with Intel rare data caches
    output reg [31:0]                           STREAM_ADDR_DATA,
    output reg [31:0]                           STREAM_DATA_IN_DATA,
    input wire [31:0]                           STREAM_DATA_OUT_DATA,
    output reg                                  STREAM_WE_DATA,
    output reg                                  STREAM_EN_DATA,

    output reg [1:0]                            STREAM_SEL,
    output reg                                  FETCH_ENABLE,
    output reg [31:0]                           BOOT_ADDR,
    output reg [31:0]                           MTVEC_ADDR,
    //output reg [31:0]                           DM_HALT_ADDR,
    //output reg [31:0]                           HART_ID,

    output reg                                  RISCV_RESETB,
    output reg                                  RISCV_CLK_EN,
    output reg                                  RISCV_RUN_CYCLE_ENABLE,
    output reg [63:0]                           RISCV_RUN_CYCLE,
    input wire                                  RISCV_RUN_DONE,
    input wire                                  CORE_SLEEP_O_PRIMARY,
    input wire                                  CORE_SLEEP_O_SECONDARY,

    input wire                                  RISCV_ERR_OVERFLOW,
    input wire [31:0]                           RISCV_ERRCNT,

    // Control signals
    output reg [7:0]                            DDLS_DELAY_SEL,
    output reg                                  DDLS_RESETB,

    output reg                                  HEATER_GLOBAL_ENABLE,
    //output reg [NUM_HEATER - 1 : 0]             HEATER_CELL_ENABLE,
    output reg [NUM_HEATER_TOP - 1 : 0]         HEATER_ENABLE_TOP,
    output wire                                 HEATER_LOAD_TOP,
    output reg [NUM_HEATER_P - 1 : 0]           HEATER_ENABLE_PRIMARY,
    output wire                                 HEATER_LOAD_PRIMARY,
    output reg [NUM_HEATER_S - 1 : 0]           HEATER_ENABLE_SECONDARY,
    output wire                                 HEATER_LOAD_SECONDARY,

    output reg [7:0]                            TEMP_SENSOR_SEL,
    output reg                                  TEMP_SENSOR_EN,
    output reg [7:0]                            VOLTAGE_SENSOR_SEL,
    output reg                                  VOLTAGE_SENSOR_EN,

    // Odometer
    output reg                                  ODOMETER_RESETB,
    output reg [NUM_ODOMETER - 1 : 0]           STRESS,
    output reg [NUM_ODOMETER - 1 : 0]           AC_DC,
    output reg [NUM_ODOMETER - 1 : 0]           SEL_INV,
    output reg [NUM_ODOMETER - 1 : 0]           SEL_NAND,
    output reg [NUM_ODOMETER - 1 : 0]           SEL_NOR,
    output reg                                  ODOMETER_MEAS_TRIG,
    output wire                                 ODOMETER_LOAD,    
    input wire [15:0]                           BIT_COUNT [NUM_ODOMETER -1 : 0],

    input wire [532:0]                          SCAN_OUT

);

// -------------- address info --------------
// Since data width is only a byte address goes up by 1 compared to 4
// in a true AXI4_Lite I/F
// 0x00 : Instr address
// 0x01 : Instr data_in
// 0x02 : Instr data_out
// 0x03 : Data address
// 0x04 : Data data_in
// 0x05 : Data data_out
// 0x06 : ADDR_CACHE_CONTROL {Stream_select[7:6], DDLS read done, DDLS read start, Data read done, Instr read done, Data read start, Instr read start}
// 0x07 : BOOT_ADDR, (offset using RISC-V input offset)
// 0x08 : MTVEC_ADDR, (offset using RISC-V input offset)
// 0x09 : ADDR_RISCV_START [0] RISCV RESETB [1] FETCH_ENABLE [2] RISCV_CLK_EN [3] DDLS_RESETB
// 0x0A : ADDR_RISCV_OFFSET
// 0x0B : ADDR_RISCV_RUN_CYCLE
// 0x0C : ADDR_RISCV_STATUS [0] RISCV_RUN_DONE [1] RISCV_ERR_OVERFLOW [2] CORESLEEP_P [3] CORESLEEP_S
// 0x0D : ADDR_RISCV_ERRCNT
// 0x0E : Delay SEL input for DDLS
// 0x0F : ADDR_DDLS_OFFSET
// 0x10 : ADDR_DDLS_ADDR
// 0x11 : ADDR_DDLS_DATA_IN
// 0x12 : ADDR_DDLS_DATA_OUT
// 0x13 : ADDR_CLK_CONTROL [0] CLK_SELECT [1] FREQ_OUT_EN
//                         [2] FREQ_OUT_CNT_CLK_SELECT
//                         [3] FREQ_OUT_CNT_RESETB [4] FREQ_OUT_CNT_START [5] FREQ_OUT_CNT_DONE
// 0x14 : ADDR_MAIN_DCO_CONTROL
// 0x15 : ADDR_RISCV_RUN_CYCLE_OFFSET
// 0x16 : ADDR_SCAN_OUT (Read only)
// 0x17 : ADDR_SCAN_OUT_OFFSET
// 0x18 : 
// 0x19 : 
// 0x1A : 
// 0x1B : 
// 0x1C :
// 0x1D :
// 0x1E :
// 0x1F :
// 0x20 : HEATER Top Select
// 0x21 : HEATER Top In
// 0x22 : HEATER P Select
// 0x23 : HEATER P In
// 0x24 : HEATER S Select
// 0x25 : HEATER S In
// 0x26 : HEATER Control {GLOBAL_ENABLE, TEMP_SENSOR_EN, VOLT_SENSOR_EN, 2'b00, LOAD_HEATER_TOP, LOAD_HEATER_P, LOAD_HEATER_S}
// 0x27 : Temp Sensor Select
// 0x28 : Voltage Sensor Select
// 0x29 : Odometer_Select [7:0]
// 0x2A : Odometer_Select [15:8] , Reserved if Number of odometer > 2^8
// 0x2B : Odometer Input {STRESS, AC_DC, SEL_INV, SEL_NAND, SEL_NOR}
// 0x2C : Odometer Meas trig and ODOMETER_LOAD {ODOMETER_RESETB, 5'b00000, ODOMETER_MEAS_TRIG, ODOMETER_LOAD}
// 0x2D : Odometer Bit Count [7:0] Read Only
// 0x2E : Odometer Bit Count [11:8] Read Only
// 0x2F : 
// 0x30 : 
// 0x31 : 
// 0x32 : 
// 0x33 : 
// 0x34 : 
// 0x35 : 
// 0x36 : 
// 0x37 : 
// 0x38 : 
// 0x39 : 
// 0x3A : 
// ...
// 0x3F :


// --------------------------------------------------------------------
//   0   0000  0000  0000  0000    0   0000    0   0   0
//  0 0  0   0 0   0 0   0 0   0  0 0  0   0  0 0  00 00
// 00000 0   0 0   0 0000  0000  00000 0000  00000 0 0 0
// 0   0 0   0 0   0 0   0 0     0   0 0   0 0   0 0   0
// 0   0 0000  0000  0   0 0     0   0 0   0 0   0 0   0
// --------------------------------------------------------------------
// -------------- local param --------------
localparam
    ADDR_INSTR_ADDR                 = 6'h00,
    ADDR_INSTR_DATA_IN              = 6'h01,
    ADDR_INSTR_DATA_OUT             = 6'h02, // READ ONLY
    ADDR_DATA_ADDR                  = 6'h03,
    ADDR_DATA_DATA_IN               = 6'h04,
    ADDR_DATA_DATA_OUT              = 6'h05, // READ ONLY
    ADDR_CACHE_CONTROL              = 6'h06; //[0] Instr read start [1] Data read start
                                             //[2] Instr read done [3] Data read done
                                             //[4] DDLS read start
                                             //[5] DDLS read done
                                             //[7:6] STREAM_SEL


// RISC-V control
localparam
    ADDR_BOOT_ADDR                  = 6'h07,
    ADDR_MTVEC_ADDR                 = 6'h08,
    ADDR_RISCV_START                = 6'h09,
    ADDR_RISCV_OFFSET               = 6'h0A,
    ADDR_RISCV_RUN_CYCLE            = 6'h0B,
    ADDR_RISCV_STATUS               = 6'h0C,
    ADDR_RISCV_ERRCNT               = 6'h0D; // Read only

// Miscellaneous
localparam
    ADDR_DELAY_SEL                  = 6'h0E;

localparam
    ADDR_DDLS_OFFSET                = 6'h0F,
    ADDR_DDLS_ADDR                  = 6'h10,
    ADDR_DDLS_DATA_IN               = 6'h11,
    ADDR_DDLS_DATA_OUT              = 6'h12;

localparam
    ADDR_CLK_CONTROL                = 6'h13,
    ADDR_MAIN_DCO_CONTROL           = 6'h14, // [7:4] COARSE [3:0] FINE
    ADDR_RISCV_RUN_CYCLE_OFFSET     = 6'h15;

localparam
    ADDR_SCAN_OUT                   = 6'h16,
    ADDR_SCAN_OUT_OFFSET            = 6'h17;

// Heater control
localparam
    ADDR_HEATER_TOP_SEL             = 6'h20,
    ADDR_HEATER_TOP_IN              = 6'h21;
localparam
    ADDR_HEATER_P_SEL               = 6'h22,
    ADDR_HEATER_P_IN                = 6'h23;
localparam
    ADDR_HEATER_S_SEL               = 6'h24,
    ADDR_HEATER_S_IN                = 6'h25;
localparam
    ADDR_HEATER_CONTROL             = 6'h26; // {GLOBAL_ENABLE, TEMP_SENSOR_EN, VOLT_SENSOR_EN, 2'b00, LOAD_HEATER_TOP, LOAD_HEATER_P, LOAD_HEATER_S}
localparam
    ADDR_TEMP_SENSOR_SEL            = 6'h27,
    ADDR_VOLTAGE_SENSOR_SEL         = 6'h28;

// Odometer
localparam
    ADDR_ODOMETER_SEL_7_0           = 6'h29,
    ADDR_ODOMETER_SEL_15_8          = 6'h2A,
    ADDR_ODOMETER_IN                = 6'h2B,
    ADDR_ODOMETER_CONTROL           = 6'h2C,
    ADDR_ODOMETER_BIT_CNT_7_0       = 6'h2D,
    ADDR_ODOMETER_BIT_CNT_15_8      = 6'h2E;

localparam
    WRIDLE                          = 2'b00,
    WRDATA                          = 2'b01,
    WRRESP                          = 2'b10,
    WRRESET                         = 2'b11,
    RDIDLE                          = 2'b00,
    RDDATA                          = 2'b01,
    RDRESET                         = 2'b10;

// --------------------------------------------------------------------
// 0      000   000    0   0      000  00000  000  0   0   0   0
// 0     0   0 0   0  0 0  0     0       0   0     00  0  0 0  0
// 0     0   0 0     00000 0      000    0   0  00 0 0 0 00000 0
// 0     0   0 0   0 0   0 0         0   0   0   0 0  00 0   0 0
// 00000  000   000  0   0 00000  000  00000  000  0   0 0   0 00000
// --------------------------------------------------------------------

// -------------- local signals --------------
reg [1:0] wstate = WRRESET;
reg [1:0] wnext;
reg [C_S_AXI_ADDR_WIDTH - 1 : 0] waddr;
wire [C_S_AXI_DATA_WIDTH - 1 : 0] wmask;
wire aw_hs;
wire w_hs;
reg [1:0] rstate = RDRESET;
reg [1:0] rnext;
reg [C_S_AXI_DATA_WIDTH - 1 : 0] rdata;
wire ar_hs;
wire [C_S_AXI_ADDR_WIDTH - 1 : 0] raddr;

// DDLS status reg
//localparam DDLS_WIDTH_LOG2 = $clog2(DDLS_WIDTH);
reg [8:0] DDLS_OFFSET;
wire DDLS_OFFSET_write_hit;
wire DDLS_OFFSET_read_hit;
wire DDLS_DATA_read_hit;
reg [5:0] DDLS_DATA_read_hit_reg;
reg DDLS_DATA_read_done;
reg [DDLS_WIDTH - 1 : 0] DDLS_DATA_read_r;

wire DDLS_ADDR_increment_hit; // read/write all 32 bits from STREAM_DATA_IN, increment address to read/write to next address

// RISCV_RUN_CYCLE_OFFSET
reg [2:0] RISCV_RUN_CYCLE_OFFSET;
wire RISCV_RUN_CYCLE_OFFSET_write_hit;
wire RISCV_RUN_CYCLE_OFFSET_read_hit;

reg [7:0] SCAN_OUT_OFFSET;
wire SCAN_OUT_OFFSET_read_hit;

// -------------- RISC-V status reg -------
reg [1:0] RISCV_OFFSET;
wire RISCV_OFFSET_write_hit;
wire RISCV_OFFSET_read_hit;
//reg [31:0] RISCV_RUN_CYCLE;



// -------------- heater reg --------------
//localparam NUM_HEATER_TOP_BY_8 = $ceil(NUM_HETER_TOP/8);
//localparam NUM_HEATER_P_BY_8 = $ceil(NUM_HEATER_P/8);
//localparam NUM_HEATER_S_BY_8 = $ceil(NUM_HEATER_S/8);

localparam NUM_HEATER_TOP_LOG2 = $clog2(NUM_HEATER_TOP);
wire [NUM_HEATER_TOP_LOG2 - 1 :0] HEATER_TOP_SEL;
reg [7:0] HEATER_TOP_SEL_temp;
assign HEATER_TOP_SEL = HEATER_TOP_SEL_temp[NUM_HEATER_TOP_LOG2-1:0];
wire HEATER_TOP_IN_hit;

localparam NUM_HEATER_P_LOG2 = $clog2(NUM_HEATER_P);
wire [NUM_HEATER_P_LOG2 - 1 : 0] HEATER_P_SEL;
reg [7:0] HEATER_P_SEL_temp;
assign HEATER_P_SEL = HEATER_P_SEL_temp[NUM_HEATER_P_LOG2-1:0];
wire HEATER_P_IN_hit;

localparam NUM_HEATER_S_LOG2 = $clog2(NUM_HEATER_S);
wire [NUM_HEATER_S_LOG2 - 1 : 0] HEATER_S_SEL;
reg [7:0] HEATER_S_SEL_temp;
assign HEATER_S_SEL = HEATER_S_SEL_temp[NUM_HEATER_S_LOG2-1:0];
wire HEATER_S_IN_hit;

// -------------- odometer regs --------------
localparam NUM_ODOMETER_LOG2 = $clog2(NUM_ODOMETER);
wire [NUM_ODOMETER_LOG2 - 1:0] ODOMETER_SEL;     // Should be enough width to represent NUM_ODOMETER
                                                // $clog2(NUM_ODOMETER)
reg [15:0] ODOMETER_SEL_temp;
assign ODOMETER_SEL = ODOMETER_SEL_temp[NUM_ODOMETER_LOG2-1:0];
wire ODOMETER_IN_hit;

// -------------- internal registers --------------
wire INSTR_DATA_read_hit;
reg [5:0] INSTR_DATA_read_hit_reg;
reg INSTR_DATA_read_done;
wire DATA_DATA_read_hit;
reg [5:0]DATA_DATA_read_hit_reg;
reg DATA_DATA_read_done;
reg [31:0] INSTR_DATA_read_r;
reg [31:0] DATA_DATA_read_r;
wire INSTR_ADDR_increment_hit; // read/write all 32 bits from STREAM_DATA_IN, increment address to read/write to next address
wire DATA_ADDR_increment_hit;// read/write all 32 bits from STREAM_DATA_IN, increment address to read/write to next address
wire INSTR_DATA_write_enable_hit;
wire DATA_DATA_write_enable_hit;

//assign INSTR_WRITE_ENABLE = INSTR_DATA_write_hit_reg;
//assign INSTR_ENABLE = INSTR_DATA_write_hit_reg || INSTR_DATA_read_hit_reg[0];

// -------------- AXI Write FSM --------------
assign AWREADY = (wstate == WRIDLE);
assign WREADY = (wstate == WRDATA);
assign BRESP = 2'b00; // OKAY
assign BVALID = (wstate == WRRESP);
//assign wmask = {8{WSTRB[0]}};
assign aw_hs = AWVALID & AWREADY;
assign w_hs = WVALID & WREADY;

// wstate
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        wstate <= WRRESET;
    end else if (ACLK_EN) begin
        wstate <= wnext;
    end
end

// wnext
always @ (*) begin
    case (wstate)
        WRIDLE: begin
            if (AWVALID)
                wnext = WRDATA;
            else
                wnext = WRIDLE;
        end

        WRDATA: begin
            if (WVALID)
                wnext = WRRESP;
            else
                wnext = WRDATA;
        end

        WRRESP: begin
            if (BREADY)
                wnext = WRIDLE;
            else
                wnext = WRRESP;
        end

        default:
            wnext = WRIDLE;
    endcase
end

// waddr
always @ (posedge ACLK) begin
    if (ACLK_EN) begin
        if (aw_hs) begin
            waddr <= AWADDR;
        end
    end
end

// -------------- AXI Read FSM --------------
assign ARREADY = (rstate == RDIDLE);
assign RDATA = rdata;
assign RRESP = 2'b00; // OKAY
assign RVALID = (rstate == RDDATA);
assign ar_hs = ARVALID & ARREADY;
assign raddr = ARADDR;

// rstate
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        rstate <= RDRESET;
    end else if (ACLK_EN) begin
        rstate <= rnext;
    end
end

// rnext
always @ (*) begin
    case (rstate)
        RDIDLE: begin
            if (ARVALID)
                rnext = RDDATA;
            else
                rnext = RDIDLE;
        end

        RDDATA: begin
            if (RREADY & RVALID)
                rnext = RDIDLE;
            else
                rnext = RDDATA;
        end

        default:
            rnext = RDIDLE;
    endcase
end

// Memory Start Hit
assign DDLS_DATA_read_hit = ACLK_EN && w_hs && (waddr == ADDR_CACHE_CONTROL) && (WDATA[4] == 1'b1);
assign INSTR_DATA_read_hit = ACLK_EN && w_hs && (waddr == ADDR_CACHE_CONTROL) && (WDATA[0] == 1'b1);
assign DATA_DATA_read_hit = ACLK_EN && w_hs && (waddr == ADDR_CACHE_CONTROL) && (WDATA[1] == 1'b1);

assign DDLS_ADDR_increment_hit = ACLK_EN && ((DDLS_OFFSET+1)*8 == DDLS_WIDTH) &&
                    ((w_hs && (waddr == ADDR_DDLS_DATA_IN)) ||
                    (ar_hs && (raddr == ADDR_DDLS_DATA_IN))
                    );
assign DDLS_DATA_write_enable_hit = ACLK_EN && ((DDLS_OFFSET+1)*8 == DDLS_WIDTH) && w_hs && (waddr == ADDR_DDLS_DATA_IN);

assign INSTR_ADDR_increment_hit = ACLK_EN && (RISCV_OFFSET == 2'b11) && 
                    ((w_hs && (waddr == ADDR_INSTR_DATA_IN)) ||
                    (ar_hs && (raddr == ADDR_INSTR_DATA_IN))
                    );
assign INSTR_DATA_write_enable_hit = ACLK_EN && (RISCV_OFFSET == 2'b11) && w_hs && (waddr == ADDR_INSTR_DATA_IN);

assign DATA_ADDR_increment_hit = ACLK_EN && (RISCV_OFFSET == 2'b11) &&
                    ((w_hs && (waddr == ADDR_DATA_DATA_IN)) ||
                    (ar_hs && (raddr == ADDR_DATA_DATA_IN))
                    );
assign DATA_DATA_write_enable_hit = ACLK_EN && (RISCV_OFFSET == 2'b11) && w_hs && (waddr == ADDR_DATA_DATA_IN);

//assign DATA_DATA_read_P_hit = ACLK_EN && w_hs && (waddr == ADDR_DATA_ADDR_P_10_8) && (WDATA[4] == 1'b1);
//assign DATA_DATA_read_S_hit = ACLK_EN && w_hs && (waddr == ADDR_DATA_ADDR_S_10_8) && (WDATA[4] == 1'b1);
//assign INSTR_DATA_write_hit = ACLK_EN && w_hs && (waddr == ADDR_INSTR_ADDR_10_8) && (WDATA[3] == 1'b1);
//assign DATA_DATA_write_P_hit = ACLK_EN && w_hs && (waddr == ADDR_DATA_ADDR_P_10_8) && (WDATA[3] == 1'b1);
//assign DATA_DATA_write_S_hit = ACLK_EN && w_hs && (waddr == ADDR_DATA_ADDR_S_10_8) && (WDATA[3] == 1'b1);

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        INSTR_DATA_read_hit_reg <= 0;
        DATA_DATA_read_hit_reg <= 0;
        DDLS_DATA_read_hit_reg <= 0;
    end else begin
        INSTR_DATA_read_hit_reg <= {INSTR_DATA_read_hit_reg[4:0], INSTR_DATA_read_hit};
        DATA_DATA_read_hit_reg <= {DATA_DATA_read_hit_reg[4:0], DATA_DATA_read_hit};
        DDLS_DATA_read_hit_reg <= {DDLS_DATA_read_hit_reg[4:0], DDLS_DATA_read_hit};
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    // 1 cycle delay until data is read from SRAM
    if (!ARESETB) begin
        DDLS_DATA_read_r <= 0;
        DDLS_DATA_read_done <= 1'b0;
    end else if (DDLS_DATA_read_hit_reg[5]) begin
        DDLS_DATA_read_r <= STREAM_DATA_OUT_DDLS;
        DDLS_DATA_read_done <= 1'b1;
    end else if (DDLS_DATA_read_hit) begin
        DDLS_DATA_read_done <= 1'b0;
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    // 1 cycle delay until data is read from SRAM
    if (!ARESETB) begin
        INSTR_DATA_read_r <= 32'b0;
        INSTR_DATA_read_done <= 1'b0;
    end else if (INSTR_DATA_read_hit_reg[5]) begin
        INSTR_DATA_read_r <= STREAM_DATA_OUT_INSTR;
        INSTR_DATA_read_done <= 1'b1;
    end else if (INSTR_DATA_read_hit) begin
        INSTR_DATA_read_done <= 1'b0;
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    // 1 cycle delay until data is read from SRAM
    if (!ARESETB) begin
        DATA_DATA_read_r <= 32'b0;
        DATA_DATA_read_done <= 1'b0;
    end else if (DATA_DATA_read_hit_reg[5]) begin
        DATA_DATA_read_r <= STREAM_DATA_OUT_DATA;
        DATA_DATA_read_done <= 1'b1;
    end else if (DATA_DATA_read_hit) begin
        DATA_DATA_read_done <= 1'b0;
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_EN_DDLS <= 1'b0;
        STREAM_WE_DDLS <= 1'b0;
    end else if (DDLS_DATA_write_enable_hit || DDLS_DATA_read_hit) begin
        STREAM_EN_DDLS <= 1'b1;
        if (DDLS_DATA_write_enable_hit) begin
            STREAM_WE_DDLS <= 1'b1;
        end
    end else begin
        STREAM_EN_DDLS <= 1'b0;
        STREAM_WE_DDLS <= 1'b0;
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_EN_INSTR <= 1'b0;
        STREAM_WE_INSTR <= 1'b0;
    end else if (INSTR_DATA_write_enable_hit || INSTR_DATA_read_hit) begin
        STREAM_EN_INSTR <= 1'b1;
        if (INSTR_DATA_write_enable_hit) begin
            STREAM_WE_INSTR <= 1'b1;
        end
    end else begin
        STREAM_EN_INSTR <= 1'b0;
        STREAM_WE_INSTR <= 1'b0;
    end
end

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_EN_DATA <= 1'b0;
        STREAM_WE_DATA <= 1'b0;
    end else if (DATA_DATA_write_enable_hit || DATA_DATA_read_hit) begin
        STREAM_EN_DATA <= 1'b1;
        if (DATA_DATA_write_enable_hit) begin
            STREAM_WE_DATA <= 1'b1;
        end
    end else begin
        STREAM_EN_DATA <= 1'b0;
        STREAM_WE_DATA <= 1'b0;
    end
end


// rdata
// --------------------------------------------------------------------
// 0000  0000    0   00000   0
// 0   0 0   0  0 0    0    0 0
// 0000  0   0 00000   0   00000
// 0   0 0   0 0   0   0   0   0
// 0   0 0000  0   0   0   0   0
// --------------------------------------------------------------------
always @ (posedge ACLK) begin
    if (ACLK_EN) begin
        if (ar_hs) begin
            rdata <= 'b0;
            case (raddr)
                ///////////////////////////////////////////////////////////////////////////////
                // Instruction cache
                ADDR_INSTR_ADDR: begin
                    rdata <= STREAM_ADDR_INSTR[RISCV_OFFSET*8 +: 8];
                end

                ADDR_INSTR_DATA_IN: begin
                    rdata <= STREAM_DATA_IN_INSTR[RISCV_OFFSET*8 +: 8];
                end

                ADDR_INSTR_DATA_OUT: begin
                    rdata <= INSTR_DATA_read_r[RISCV_OFFSET*8 +: 8];
                end
                ///////////////////////////////////////////////////////////////////////////////
                // Data cache
                ADDR_DATA_ADDR: begin
                    rdata <= STREAM_ADDR_DATA[RISCV_OFFSET*8 +: 8];
                end
                
                ADDR_DATA_DATA_IN: begin
                    rdata <= STREAM_DATA_IN_DATA[RISCV_OFFSET*8 +: 8];
                end

                ADDR_DATA_DATA_OUT: begin
                    rdata <= DATA_DATA_read_r[RISCV_OFFSET*8 +: 8];
                end
                ///////////////////////////////////////////////////////////////////////////////
                // RISCV and Cache Control
                ADDR_RISCV_START: begin
                    rdata <= {4'b0000, DDLS_RESETB, RISCV_CLK_EN,FETCH_ENABLE,RISCV_RESETB};
                end

                ADDR_RISCV_OFFSET: begin
                    rdata <= {RISCV_OFFSET};
                end

                ADDR_RISCV_RUN_CYCLE: begin
                    rdata <= RISCV_RUN_CYCLE[RISCV_RUN_CYCLE_OFFSET*8 +: 8];
                end

                ADDR_RISCV_RUN_CYCLE_OFFSET: begin
                    rdata <= {RISCV_RUN_CYCLE_ENABLE, 4'b0000, RISCV_RUN_CYCLE_OFFSET};
                end

                ADDR_RISCV_ERRCNT: begin
                    rdata <= RISCV_ERRCNT[RISCV_OFFSET*8 +: 8];
                end
                
                ADDR_BOOT_ADDR: begin
                    rdata <= BOOT_ADDR[RISCV_OFFSET*8 +: 8];
                end

                ADDR_MTVEC_ADDR: begin
                    rdata <= MTVEC_ADDR[RISCV_OFFSET*8 +: 8];
                end

                ADDR_CACHE_CONTROL: begin
                    rdata <= {STREAM_SEL[1:0], DDLS_DATA_read_done, 1'b0, DATA_DATA_read_done, INSTR_DATA_read_done,2'b00};
                end
                ///////////////////////////////////////////////////////////////////////////////
                // DDLS Control
                ADDR_DDLS_OFFSET: begin
                    rdata <= DDLS_OFFSET;
                end

                ADDR_DELAY_SEL: begin
                    rdata <= DDLS_DELAY_SEL;
                end
                ///////////////////////////////////////////////////////////////////////////////
                // DDLS cache
                ADDR_DDLS_ADDR: begin
                    rdata <= STREAM_ADDR_DDLS[DDLS_OFFSET[1:0]*8 +: 8];
                end

                ADDR_DDLS_DATA_IN: begin
                    rdata <= STREAM_DATA_IN_DDLS[DDLS_OFFSET*8 +: 8];
                end

                ADDR_DDLS_DATA_OUT: begin
                    rdata <= DDLS_DATA_read_r[DDLS_OFFSET*8 +: 8];
                end
                
                ADDR_SCAN_OUT: begin
                    rdata <= 8'b00000000 | SCAN_OUT[SCAN_OUT_OFFSET*8 +: 8];
                end
                ///////////////////////////////////////////////////////////////////////////////
                // Heater
                ADDR_HEATER_TOP_SEL: begin
                    rdata <= HEATER_TOP_SEL_temp;
                end

                ADDR_HEATER_TOP_IN: begin
                    rdata <= HEATER_ENABLE_TOP[HEATER_TOP_SEL];
                end

                ADDR_HEATER_P_SEL: begin
                    rdata <= HEATER_P_SEL_temp;
                end

                ADDR_HEATER_P_IN: begin
                    rdata <= HEATER_ENABLE_PRIMARY[HEATER_P_SEL];
                end

                ADDR_HEATER_S_SEL: begin
                    rdata <= HEATER_S_SEL_temp;
                end

                ADDR_HEATER_S_IN: begin
                    rdata <= HEATER_ENABLE_SECONDARY[HEATER_S_SEL];
                end

                ADDR_HEATER_CONTROL: begin
                    rdata <= {HEATER_GLOBAL_ENABLE, 4'b0000, HEATER_LOAD_TOP, HEATER_LOAD_PRIMARY, HEATER_LOAD_SECONDARY};
                end
                ///////////////////////////////////////////////////////////////////////////////
                // Sensors
                ADDR_TEMP_SENSOR_SEL: begin
                    rdata <= TEMP_SENSOR_SEL;
                end

                ADDR_VOLTAGE_SENSOR_SEL: begin
                    rdata <= VOLTAGE_SENSOR_SEL;
                end
                ///////////////////////////////////////////////////////////////////////////////
                // Odometer
                ADDR_ODOMETER_SEL_7_0: begin
                    rdata <= ODOMETER_SEL_temp[7:0];
                end

                ADDR_ODOMETER_SEL_15_8: begin
                    rdata <= ODOMETER_SEL_temp[15:8];
                end

                ADDR_ODOMETER_IN: begin
                    rdata <= {STRESS[ODOMETER_SEL], AC_DC[ODOMETER_SEL], SEL_INV[ODOMETER_SEL], SEL_NAND[ODOMETER_SEL], SEL_NOR[ODOMETER_SEL]};
                end

                ADDR_ODOMETER_CONTROL: begin
                    rdata <= {ODOMETER_RESETB, 5'b00000, ODOMETER_MEAS_TRIG, ODOMETER_LOAD};
                end

                ADDR_ODOMETER_BIT_CNT_7_0: begin
                    rdata <= BIT_COUNT[ODOMETER_SEL][7:0];
                end

                ADDR_ODOMETER_BIT_CNT_15_8: begin
                    rdata <= BIT_COUNT[ODOMETER_SEL][15:8];
                end
                ///////////////////////////////////////////////////////////////////////////////
                
                ADDR_CLK_CONTROL: begin
                    rdata <= {2'b00, FREQ_OUT_CNT_DONE, FREQ_OUT_CNT_START, FREQ_OUT_CNT_RESETB, FREQ_OUT_CNT_CLK_SELECT, FREQ_OUT_EN, CLK_SELECT};
                end

                ADDR_RISCV_STATUS: begin
                    rdata <= {4'b0000, CORE_SLEEP_O_SECONDARY, CORE_SLEEP_O_PRIMARY, RISCV_ERR_OVERFLOW, RISCV_RUN_DONE};
                end

                ADDR_MAIN_DCO_CONTROL: begin
                    rdata <= {DCO_COARSE, DCO_FINE};
                end

            endcase
        end
    end
end

// wdata
// --------------------------------------------------------------------
// 0   0  0000    0   00000   0
// 0   0  0   0  0 0    0    0 0
// 0 0 0  0   0 00000   0   00000
// 0 0 0  0   0 0   0   0   0   0
//  0 0   0000  0   0   0   0   0
// --------------------------------------------------------------------
// ADDR_INSTR_ADDR
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_ADDR_INSTR <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_INSTR_ADDR) begin
            STREAM_ADDR_INSTR[RISCV_OFFSET*8 +: 8] <= WDATA;
        end

        if (STREAM_EN_INSTR) begin
            STREAM_ADDR_INSTR <= STREAM_ADDR_INSTR + 1'b1;
        end
    end
end

// ADDR_INSTR_DATA_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_DATA_IN_INSTR <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_INSTR_DATA_IN) begin
            STREAM_DATA_IN_INSTR[RISCV_OFFSET*8 +: 8] <= WDATA;
        end
    end
end

// ADDR_DATA_ADDR
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_ADDR_DATA <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DATA_ADDR) begin
            STREAM_ADDR_DATA[RISCV_OFFSET*8 +: 8] <= WDATA;
        end

        if (STREAM_EN_DATA) begin
            STREAM_ADDR_DATA <= STREAM_ADDR_DATA + 1'b1;
        end
    end
end

// ADDR_DATA_DATA_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_DATA_IN_DATA <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DATA_DATA_IN) begin
            STREAM_DATA_IN_DATA[RISCV_OFFSET*8 +: 8] <= WDATA;
        end
    end
end

// --------------------------------------------------------------------
// 0000  0000  0      000  
// 0   0 0   0 0     0
// 0   0 0   0 0      000
// 0   0 0   0 0         0
// 0000  0000  00000  000
// --------------------------------------------------------------------

// ADDR_DDLS_ADDR
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_ADDR_DDLS <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DDLS_ADDR) begin
            STREAM_ADDR_DDLS[DDLS_OFFSET[1:0]*8 +: 8] <= WDATA;
        end
        
        if (STREAM_EN_DDLS) begin
            STREAM_ADDR_DDLS <= STREAM_ADDR_DDLS + 1'b1;
        end
    end
end

// ADDR_DDLS_DATA_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_DATA_IN_DDLS <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DDLS_DATA_IN) begin
            STREAM_DATA_IN_DDLS[DDLS_OFFSET*8 +: 8] <= WDATA;
        end
    end
end


// --------------------------------------------------------------------
// 0   0 00000   0   00000 00000 0000        
// 0   0 0      0 0    0   0     0   0     
// 00000 00000 00000   0   00000 0000  
// 0   0 0     0   0   0   0     0   0  
// 0   0 00000 0   0   0   00000 0   0  
// --------------------------------------------------------------------

assign HEATER_TOP_IN_hit = ACLK_EN && w_hs && (waddr == ADDR_HEATER_TOP_IN);
assign HEATER_P_IN_hit = ACLK_EN && w_hs && (waddr == ADDR_HEATER_P_IN);
assign HEATER_S_IN_hit = ACLK_EN && w_hs && (waddr == ADDR_HEATER_S_IN);
assign HEATER_LOAD_TOP = ACLK_EN && w_hs && (waddr == ADDR_HEATER_CONTROL) && (WDATA[2]);
assign HEATER_LOAD_PRIMARY = ACLK_EN && w_hs && (waddr == ADDR_HEATER_CONTROL) && (WDATA[1]);
assign HEATER_LOAD_SECONDARY = ACLK_EN && w_hs && (waddr == ADDR_HEATER_CONTROL) && (WDATA[0]);

// ADDR_HEATER_TOP_SEL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_TOP_SEL_temp <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_TOP_SEL) begin
            HEATER_TOP_SEL_temp[7:0] <= WDATA[7:0] * 8;
        end
        // If new heater in signal comes in, increment heater sel
        if (HEATER_TOP_IN_hit) begin
            if (HEATER_TOP_SEL >= NUM_HEATER_TOP - 8) begin
                HEATER_TOP_SEL_temp <= 0;
            end else begin
                HEATER_TOP_SEL_temp <= (HEATER_TOP_SEL_temp + 8);
            end            
        end
    end
end

// ADDR_HEATER_TOP_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_ENABLE_TOP <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_TOP_IN) begin
            HEATER_ENABLE_TOP[HEATER_TOP_SEL +: 8] <= WDATA[7:0];
        end
    end
end

// ADDR_HEATER_P_SEL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_P_SEL_temp <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_P_SEL) begin
            HEATER_P_SEL_temp[7:0] <= WDATA[7:0] * 8;
        end
        // If new heater in signal comes in, increment heater sel
        if (HEATER_P_IN_hit) begin
            if (HEATER_P_SEL >= NUM_HEATER_P - 8) begin
                HEATER_P_SEL_temp <= 0;
            end else begin
                HEATER_P_SEL_temp <= (HEATER_P_SEL_temp + 8);
            end
        end
    end
end

// ADDR_HEATER_P_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_ENABLE_PRIMARY <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_P_IN) begin
            HEATER_ENABLE_PRIMARY[HEATER_P_SEL +: 8] <= WDATA[7:0];
        end
    end
end

// ADDR_HEATER_S_SEL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_S_SEL_temp <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_S_SEL) begin
            HEATER_S_SEL_temp[7:0] <= WDATA[7:0] * 8;
        end
        // If new heater in signal comes in, increment heater sel
        if (HEATER_S_IN_hit) begin
            if (HEATER_S_SEL >= NUM_HEATER_S - 8) begin
                HEATER_S_SEL_temp <= 0;
            end else begin
                HEATER_S_SEL_temp <= (HEATER_S_SEL_temp + 8);
            end
        end
    end
end

// ADDR_HEATER_S_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_ENABLE_SECONDARY <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_S_IN) begin
            HEATER_ENABLE_SECONDARY[HEATER_S_SEL +: 8] <= WDATA[7:0];
        end
    end
end

// ADDR_HEATER_CONTROL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        HEATER_GLOBAL_ENABLE <= 1'b0;
        TEMP_SENSOR_EN <= 1'b0;
        VOLTAGE_SENSOR_EN <= 1'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_HEATER_CONTROL) begin
            HEATER_GLOBAL_ENABLE <= WDATA[7];
            TEMP_SENSOR_EN <= WDATA[6];
            VOLTAGE_SENSOR_EN <= WDATA[5];
        end
    end
end

// ADDR_TEMP_SENSOR_SEL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        TEMP_SENSOR_SEL <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_TEMP_SENSOR_SEL) begin
            TEMP_SENSOR_SEL <= WDATA[7:0];
        end
    end
end

// --------------------------------------------------------------------
//  0000  0000   000  0   0 00000 00000 00000 0000
// 0    0 0   0 0   0 00 00 0       0   0     0   0
// 0    0 0   0 0   0 0 0 0 00000   0   00000 0000
// 0    0 0   0 0   0 0   0 0       0   0     0   0
//  0000  0000   000  0   0 00000   0   00000 0   0
// --------------------------------------------------------------------

// Odometer In Hit
assign ODOMETER_IN_hit = ACLK_EN && w_hs && (waddr == ADDR_ODOMETER_IN);
//assign ODOMETER_MEAS_TRIG = ACLK_EN && w_hs && (waddr == ADDR_ODOMETER_CONTROL) && (WDATA[1]);
assign ODOMETER_LOAD = ACLK_EN && w_hs && (waddr == ADDR_ODOMETER_CONTROL) && (WDATA[0]);



// ADDR_ODOMETER_SEL_7_0
// ADDR_ODOMETER_SEL_15_8
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        ODOMETER_SEL_temp <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ODOMETER_SEL_7_0) begin
            ODOMETER_SEL_temp[7:0] <= WDATA[7:0];
        end else if (w_hs && waddr == ADDR_ODOMETER_SEL_15_8) begin
            ODOMETER_SEL_temp[15:8] <= WDATA[7:0];
        end
    
        // If new odometer in signal comes in, increment odometer sel
        if (ODOMETER_IN_hit) begin
            if (ODOMETER_SEL == NUM_ODOMETER - 1) begin
                ODOMETER_SEL_temp <= 0;
            end else begin
                ODOMETER_SEL_temp <= (ODOMETER_SEL_temp + 1'b1);
            end            
        end
    end
end

// ADDR_ODOMETER_IN
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STRESS <= {NUM_ODOMETER{1'b0}};
        AC_DC <= {NUM_ODOMETER{1'b0}};
        SEL_INV <= {NUM_ODOMETER{1'b0}};
        SEL_NAND <= {NUM_ODOMETER{1'b0}};
        SEL_NOR <= {NUM_ODOMETER{1'b0}};
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ODOMETER_IN) begin
            STRESS[ODOMETER_SEL] <= WDATA[4];
            AC_DC[ODOMETER_SEL] <= WDATA[3];
            SEL_INV[ODOMETER_SEL] <= WDATA[2];
            SEL_NAND[ODOMETER_SEL] <= WDATA[1];
            SEL_NOR[ODOMETER_SEL] <= WDATA[0];
        end
    end
end

// ADDR_ODOMETER_CONTROL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        ODOMETER_RESETB <= 1'b0;
        ODOMETER_MEAS_TRIG <= 1'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_ODOMETER_CONTROL) begin
            ODOMETER_RESETB <= WDATA[7];
            ODOMETER_MEAS_TRIG <= WDATA[1];
        end
    end
end

// --------------------------------------------------------------------
// 0000  00000  000   000        0   0
// 0   0   0   0     0   0       0   0
// 0000    0    000  0     00000 0   0
// 0   0   0       0 0   0        0 0
// 0   0 00000  000   000          0
// --------------------------------------------------------------------

// ADDR_DDLS_OFFSET
assign DDLS_OFFSET_read_hit = ACLK_EN && (ar_hs && (
                    (raddr == ADDR_DDLS_DATA_IN) ||
                    (raddr == ADDR_DDLS_DATA_OUT) ||
                    (raddr == ADDR_DDLS_ADDR)
                    ));
assign DDLS_OFFSET_write_hit = ACLK_EN && (w_hs && (
                    (waddr == ADDR_DDLS_DATA_IN) ||
                    (waddr == ADDR_DDLS_ADDR)
                    ));

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        DDLS_OFFSET <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DDLS_OFFSET) begin
            DDLS_OFFSET <= WDATA;
        end

        if (DDLS_OFFSET_read_hit || DDLS_OFFSET_write_hit) begin
            if (DDLS_OFFSET*8 == DDLS_WIDTH-8) begin
                DDLS_OFFSET <= 0;
            end else begin
                DDLS_OFFSET <= DDLS_OFFSET + 1'b1;
            end
        end
    end
end

// ADDR_RISCV_RUN_CYCLE_OFFSET
assign RISCV_RUN_CYCLE_OFFSET_read_hit = ACLK_EN && (ar_hs && (
                    (raddr == ADDR_RISCV_RUN_CYCLE)
                    ));
assign RISCV_RUN_CYCLE_OFFSET_write_hit = ACLK_EN && (w_hs && (
                    (waddr == ADDR_RISCV_RUN_CYCLE)
                    ));


always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        RISCV_RUN_CYCLE_OFFSET <= 0;
        RISCV_RUN_CYCLE_ENABLE <= 1'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RISCV_RUN_CYCLE_OFFSET) begin
            RISCV_RUN_CYCLE_OFFSET <= WDATA[2:0];
            RISCV_RUN_CYCLE_ENABLE <= WDATA[7];
        end

        if (RISCV_RUN_CYCLE_OFFSET_read_hit || RISCV_RUN_CYCLE_OFFSET_write_hit) begin
            RISCV_RUN_CYCLE_OFFSET <= RISCV_RUN_CYCLE_OFFSET + 1'b1;
        end
    end
end

// ADDR_SCAN_OUT_OFFSET
assign SCAN_OUT_OFFSET_read_hit = ACLK_EN && (ar_hs && (
                    (raddr == ADDR_SCAN_OUT)
                    ));

always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        SCAN_OUT_OFFSET <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_SCAN_OUT_OFFSET) begin
            SCAN_OUT_OFFSET <= WDATA[7:0];
        end

        if (SCAN_OUT_OFFSET_read_hit) begin
            if (SCAN_OUT_OFFSET == 16) begin
                SCAN_OUT_OFFSET <= 0;
            end else begin
                SCAN_OUT_OFFSET <= SCAN_OUT_OFFSET + 1'b1;
            end
        end
    end
end

// If ADDR_RISCV_RUN_CYCLE or ADDR_RISCV_ERRCNT are hit
// then ADDR_RISCV_OFFSET increments
assign RISCV_OFFSET_read_hit = ACLK_EN && (ar_hs && (
                    (raddr == ADDR_INSTR_ADDR) ||
                    (raddr == ADDR_INSTR_DATA_IN) ||
                    (raddr == ADDR_INSTR_DATA_OUT) ||
                    (raddr == ADDR_DATA_ADDR) ||
                    (raddr == ADDR_DATA_DATA_IN) ||
                    (raddr == ADDR_DATA_DATA_OUT) ||
                    (raddr == ADDR_BOOT_ADDR) ||
                    (raddr == ADDR_MTVEC_ADDR) ||
                    (raddr == ADDR_RISCV_ERRCNT)              
                    ));
assign RISCV_OFFSET_write_hit = ACLK_EN && (w_hs && (
                    (waddr == ADDR_INSTR_ADDR) ||
                    (waddr == ADDR_INSTR_DATA_IN) ||
                    (waddr == ADDR_DATA_ADDR) ||
                    (waddr == ADDR_DATA_DATA_IN) ||
                    (waddr == ADDR_BOOT_ADDR) ||
                    (waddr == ADDR_MTVEC_ADDR)  
                    ));

// ADDR_RISCV_OFFSET
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        RISCV_OFFSET <= 2'b0;
        //STREAM_SEL <= 2'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RISCV_OFFSET) begin
            RISCV_OFFSET <= WDATA[1:0];
            //STREAM_SEL <= WDATA[3:2];
        end

        if (RISCV_OFFSET_read_hit || RISCV_OFFSET_write_hit) begin
            RISCV_OFFSET <= RISCV_OFFSET + 1'b1;
        end
    end
end

// ADDR_RISCV_RUN_CYCLE
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        RISCV_RUN_CYCLE <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RISCV_RUN_CYCLE) begin
            RISCV_RUN_CYCLE[RISCV_RUN_CYCLE_OFFSET*8 +: 8] <= WDATA;
        end
    end
end

// ADDR_CACHE_CONTROL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        STREAM_SEL <= 2'b0;
    end else if (ACLK_EN) begin
        if (w_hs & waddr == ADDR_CACHE_CONTROL) begin
            STREAM_SEL <= WDATA[7:6];
        end
    end
end

// ADDR_BOOT_ADDR
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        BOOT_ADDR <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_BOOT_ADDR) begin
            BOOT_ADDR[RISCV_OFFSET*8 +: 8] <= WDATA;
        end
    end
end

// ADDR_MTVEC_ADDR
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        MTVEC_ADDR <= 32'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_MTVEC_ADDR) begin
            MTVEC_ADDR[RISCV_OFFSET*8 +: 8] <= WDATA;
        end
    end
end

//assign RISCV_CLK_EN = ACLK_EN && w_hs && (waddr == ) && (WDATA[]);
// ADDR_RISCV_START
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        RISCV_RESETB <= 1'b0;
        FETCH_ENABLE <= 1'b0;
        RISCV_CLK_EN <= 1'b0;
        DDLS_RESETB <= 1'b0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_RISCV_START) begin
            RISCV_RESETB <= WDATA[0];
            FETCH_ENABLE <= WDATA[1];
            RISCV_CLK_EN <= WDATA[2];
            DDLS_RESETB <= WDATA[3];
        end
    end
end

// ADDR_DELAY_SEL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        DDLS_DELAY_SEL <= 0;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_DELAY_SEL) begin
            DDLS_DELAY_SEL <= WDATA[7:0];
        end
    end
end

// ADDR_CLK_CONTROL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        CLK_SELECT <= 1'b0;
        FREQ_OUT_EN <= 1'b0;
        FREQ_OUT_CNT_CLK_SELECT <= 1'b0;
        FREQ_OUT_CNT_RESETB <= 1'b0;
        FREQ_OUT_CNT_START <= 1'b0;     
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_CLK_CONTROL) begin
            CLK_SELECT <= WDATA[0];
            FREQ_OUT_EN <= WDATA[1];
            FREQ_OUT_CNT_CLK_SELECT <= WDATA[2];
            FREQ_OUT_CNT_RESETB <= WDATA[3];
            FREQ_OUT_CNT_START <= WDATA[4];
        end
    end
end

// ADDR_MAIN_DCO_CONTROL
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        DCO_COARSE <= 4'b0000;
        DCO_FINE <= 4'b0000;
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ADDR_MAIN_DCO_CONTROL) begin
            DCO_COARSE <= WDATA[7:4];
            DCO_FINE <= WDATA[3:0];
        end
    end
end
/*
always @ (posedge ACLK or negedge ARESETB) begin
    if (!ARESETB) begin
        
    end else if (ACLK_EN) begin
        if (w_hs && waddr == ) begin

        end
    end
end
*/

endmodule