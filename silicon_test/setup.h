//###############################################################################################################################
//  000  00000 00000 0   0 000
// 0     0       0   0   0 0  0
//  000  00000   0   0   0 000
//     0 0       0   0   0 0
//  000  00000   0    000  0
//###############################################################################################################################

// Number of noop between clock high -> low
#define CLK_HALF_PERIOD                 5000
//#define CLK_HALF_PERIOD                 50000
// GPIO Pin definitions
// Output pins
#define ACLK_PIN                        14
#define ARESETB_PIN                     4
#define AWRITEB_PIN                     22
#define AADDR_0_PIN                     10
#define AADDR_1_PIN                     9
#define AADDR_2_PIN                     11
#define AADDR_3_PIN                     18
#define AADDR_4_PIN                     23
#define AADDR_5_PIN                     13
#define AVALID_PIN                      15
#define WVALID_PIN                      16
#define RREADY_PIN                      3
#define RSEL_PIN                        2
// Input pins
#define AREADY_PIN                      25
#define WREADY_PIN                      8
#define RVALID_PIN                      24
// Data pins, used as both inputs and outputs based on read mode or write mode
// Initialize as write mode (output)
#define DATA_0_PIN                      7
#define DATA_1_PIN                      5
#define DATA_2_PIN                      12
#define DATA_3_PIN                      6
#define DATA_4_PIN                      19
#define DATA_5_PIN                      20
#define DATA_6_PIN                      26
#define DATA_7_PIN                      21
// PWM signal pins
// For this test these don't have to PWM as I will be enabling global enable only for a very short period of time
// Keep both pin at HIGH all time
#define PWM_A_PIN                       27
#define PWM_B_PIN                       17

//###############################################################################################################################
//   0   0   0 00000       0000  00000  000
//  0 0   0 0    0         0   0 0     0
// 00000   0     0         0000  00000 0  00
// 0   0  0 0    0         0   0 0     0   0
// 0   0 0   0 00000       0   0 00000  000
//###############################################################################################################################
#define ADDR_INSTR_ADDR                 0x00
#define ADDR_INSTR_DATA_IN              0x01
#define ADDR_INSTR_DATA_OUT             0x02 // Read only
#define ADDR_DATA_ADDR                  0x03
#define ADDR_DATA_DATA_IN               0x04
#define ADDR_DATA_DATA_OUT              0x05 // Read only
#define ADDR_CACHE_CONTROL              0x06
// [0] Instr read start     [1] Data read start     [2] Instr read done      [3] Data read done
// [4] DDLS read start      [5] DDLS read done      [7:6] STREAM_SEL
#define ADDR_BOOT_ADDR                  0x07
#define ADDR_MTVEC_ADDR                 0x08
#define ADDR_RISCV_START                0x09
#define ADDR_RISCV_OFFSET               0x0A
#define ADDR_RISCV_RUN_CYCLE            0x0B
#define ADDR_RISCV_STATUS               0x0C
#define ADDR_RISCV_ERRCNT               0x0D // Read only
#define ADDR_DELAY_SEL                  0x0E
#define ADDR_DDLS_OFFSET                0x0F
#define ADDR_DDLS_ADDR                  0x10
#define ADDR_DDLS_DATA_IN               0x11
#define ADDR_DDLS_DATA_OUT              0x12
#define ADDR_CLK_CONTROL                0x13
#define ADDR_MAIN_DCO_CONTROL           0x14
// [7:4] COARSE             [3:0] FINE
#define ADDR_RISCV_RUN_CYCLE_OFFSET     0x15
#define ADDR_SCAN_OUT                   0x16
#define ADDR_SCAN_OUT_OFFSET            0x17
#define ADDR_CORE_CYCLE_PRIMARY         0x1C
#define ADDR_CORE_CYCLE_SECONDARY       0x1D
#define ADDR_HEATER_P_SEL               0x22
#define ADDR_HEATER_P_IN                0x23
#define ADDR_HEATER_S_SEL               0x24
#define ADDR_HEATER_S_IN                0x25
#define ADDR_HEATER_CONTROL             0x26
// [0] LOAD_HEATER_S        [1] LOAD_HEATER_P       [2] LOAD_HEATER_TOP     [4:3] N/A
// [5] VOLT_SENSOR_EN       [6] TEMP_SENSOR_EN      [7] GLOBAL_ENABLE
#define ADDR_TEMP_SENSOR_SEL            0x27
#define ADDR_ODOMETER_SEL_7_0           0x29
#define ADDR_ODOMETER_SEL_15_8          0x2A
#define ADDR_ODOMETER_IN                0x2B
#define ADDR_ODOMETER_CONTROL           0x2C
#define ADDR_ODOMETER_BIT_CNT_7_0       0x2D
#define ADDR_ODOMETER_BIT_CNT_15_8      0x2E
#define ADDR_DEBUG_FREQ_OUT_SEL         0x30
#define ADDR_MAIN_DCO_EN                0x3F

int set_pin_mode();
int chip_reset();
void clk_toggle();
void clk_toggle_half_period(int half_period);
int set_data_2_output();
int set_data_2_input();
int check_switch_data_port(int switch_mode);
int write_stream(int addr, int data);
int read_stream(int addr);
