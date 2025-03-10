#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include "setup.h"
#include "wiringPi.h"

// initialize pin IO setting
int set_pin_mode(){
    // digital outputs
    pinMode(ACLK_PIN, OUTPUT);
    pinMode(ARESETB_PIN, OUTPUT);
    pinMode(AWRITEB_PIN, OUTPUT);
    pinMode(AADDR_0_PIN, OUTPUT);
    pinMode(AADDR_1_PIN, OUTPUT);
    pinMode(AADDR_2_PIN, OUTPUT);
    pinMode(AADDR_3_PIN, OUTPUT);
    pinMode(AADDR_4_PIN, OUTPUT);
    pinMode(AADDR_5_PIN, OUTPUT);
    pinMode(AVALID_PIN, OUTPUT);
    pinMode(WVALID_PIN, OUTPUT);
    pinMode(RREADY_PIN, OUTPUT);
    pinMode(RSEL_PIN, OUTPUT);
    // trigate selection, default is in data write mode
    digitalWrite(RSEL_PIN, LOW);
    digitalWrite(AWRITEB_PIN, LOW);
    // digital inputs
    pinMode(AREADY_PIN, INPUT);
    pinMode(WREADY_PIN, INPUT);
    pinMode(RVALID_PIN, INPUT);
    // data pins, default is in data write mode
    // it is due to limited pad
    pinMode(DATA_0_PIN, OUTPUT);
    pinMode(DATA_1_PIN, OUTPUT);
    pinMode(DATA_2_PIN, OUTPUT);
    pinMode(DATA_3_PIN, OUTPUT);
    pinMode(DATA_4_PIN, OUTPUT);
    pinMode(DATA_5_PIN, OUTPUT);
    pinMode(DATA_6_PIN, OUTPUT);
    pinMode(DATA_7_PIN, OUTPUT);
    // pwm pins
    pinMode(PWM_A_PIN, OUTPUT);
    pinMode(PWM_B_PIN, OUTPUT);
    return 0;
}

//###############################################################################################################################
// 0000  00000  000  00000 00000
// 0   0 0     0     0       0  
// 0000  00000  000  00000   0  
// 0   0 0         0 0       0 
// 0   0 00000  000  00000   0  
//###############################################################################################################################
int chip_reset(){
    printf("Resetting ...\n");
    set_data_2_output(); // default data output mode
    // initialize all output digital pins to LOW
    digitalWrite(AADDR_0_PIN, LOW);
    digitalWrite(AADDR_1_PIN, LOW);
    digitalWrite(AADDR_2_PIN, LOW);
    digitalWrite(AADDR_3_PIN, LOW);
    digitalWrite(AADDR_4_PIN, LOW);
    digitalWrite(AADDR_5_PIN, LOW);
    digitalWrite(AVALID_PIN, LOW);
    digitalWrite(WVALID_PIN, LOW);
    digitalWrite(DATA_0_PIN, LOW);
    digitalWrite(DATA_1_PIN, LOW);
    digitalWrite(DATA_2_PIN, LOW);
    digitalWrite(DATA_3_PIN, LOW);
    digitalWrite(DATA_4_PIN, LOW);
    digitalWrite(DATA_5_PIN, LOW);
    digitalWrite(DATA_6_PIN, LOW);
    digitalWrite(DATA_7_PIN, LOW);
    digitalWrite(PWM_A_PIN, LOW);
    digitalWrite(PWM_B_PIN, LOW);

    // toggle reset
    clk_toggle();
    digitalWrite(ARESETB_PIN, LOW);
    clk_toggle();
    digitalWrite(ARESETB_PIN, HIGH);
    clk_toggle();

    // turn off global heater enable / dco / freq_out
    write_stream(ADDR_RISCV_START, 0b00000000);
    write_stream(ADDR_HEATER_CONTROL, 0x00);
    write_stream(ADDR_MAIN_DCO_EN, 0x00);
    write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 0x00);
    clk_toggle();
    digitalWrite(ARESETB_PIN, LOW);
    clk_toggle();
    digitalWrite(ARESETB_PIN, HIGH);
    clk_toggle();
    return 0;
}

//###############################################################################################################################
//  000  0      000   000  0   0
// 0     0     0   0 0     0 00
// 0     0     0   0 0     00
// 0     0     0   0 0     0 00
//  000  00000  000   000  0   0
//###############################################################################################################################
void clk_toggle(){
    for (int i = 0; i < CLK_HALF_PERIOD; i++) {
        __asm__("nop\n\t");
    }
    digitalWrite(ACLK_PIN, HIGH);
    for (int i = 0; i < CLK_HALF_PERIOD; i++) {
        __asm__("nop\n\t");
    }
    digitalWrite(ACLK_PIN, LOW);
}

// clk period with custom input value
void clk_toggle_half_period(int half_period){
    for (int i = 0; i < half_period; i++) {
        __asm__("nop\n\t");
    }
    digitalWrite(ACLK_PIN, HIGH);
    for (int i = 0; i < half_period; i++) {
        __asm__("nop\n\t");
    }
    digitalWrite(ACLK_PIN, LOW);
}

// trigate setup to output mode
int set_data_2_output(){
    digitalWrite(RSEL_PIN, LOW);
    digitalWrite(AWRITEB_PIN, LOW);
    
    pinMode(DATA_0_PIN, OUTPUT);
    pinMode(DATA_1_PIN, OUTPUT);
    pinMode(DATA_2_PIN, OUTPUT);
    pinMode(DATA_3_PIN, OUTPUT);
    pinMode(DATA_4_PIN, OUTPUT);
    pinMode(DATA_5_PIN, OUTPUT);
    pinMode(DATA_6_PIN, OUTPUT);
    pinMode(DATA_7_PIN, OUTPUT);

    return 0;
}

// trigate setup to input mode
int set_data_2_input(){
    pinMode(DATA_0_PIN, INPUT);
    pinMode(DATA_1_PIN, INPUT);
    pinMode(DATA_2_PIN, INPUT);
    pinMode(DATA_3_PIN, INPUT);
    pinMode(DATA_4_PIN, INPUT);
    pinMode(DATA_5_PIN, INPUT);
    pinMode(DATA_6_PIN, INPUT);
    pinMode(DATA_7_PIN, INPUT);
    
    // Trigate to read mode
    digitalWrite(RSEL_PIN, HIGH);
    digitalWrite(AWRITEB_PIN, HIGH);

    return 0;
}

// check data port status
// if switch mode = 0 switch to write mode if it is in read mode currently
// if swtich mode = 1 swithc to read mode if it is in write mode currently
int check_switch_data_port(int switch_mode){
    if (digitalRead(RSEL_PIN) == 0){ // currently in write mode
        if (switch_mode == 1){
            printf("Switching to read mode...\n");
            set_data_2_input();
        } else if (switch_mode != 0){
            printf("Error, exiting...\n");
            exit(1);
        }
    }
    else { // currently in read mode
        if (switch_mode == 0){
            printf("Swithching to write mode...\n");
            set_data_2_output();
        } else if (switch_mode != 1){
            printf("Error, exiting...\n");
            exit(1);
        }
    }

    clk_toggle();

    return 0;
}

//###############################################################################################################################
// 0   0 0000  00000 00000 00000        000  00000 0000  00000   0   0   0
// 0   0 0   0   0     0   0           0       0   0   0 0      0 0  00 00
// 0 0 0 0000    0     0   00000        000    0   0000  00000 00000 0 0 0
// 0 0 0 0   0   0     0   0               0   0   0   0 0     0   0 0   0
//  0 0  0   0 00000   0   00000        000    0   0   0 00000 0   0 0   0
//###############################################################################################################################
// axi write stream to addr and data
// addr is 6 bit
// data is 8 bit
int write_stream(int addr, int data){
    set_data_2_output();
    // allocate address to correct pin
    if (addr & (1 << 0)) digitalWrite(AADDR_0_PIN, HIGH); else digitalWrite(AADDR_0_PIN, LOW);
    if (addr & (1 << 1)) digitalWrite(AADDR_1_PIN, HIGH); else digitalWrite(AADDR_1_PIN, LOW);
    if (addr & (1 << 2)) digitalWrite(AADDR_2_PIN, HIGH); else digitalWrite(AADDR_2_PIN, LOW);
    if (addr & (1 << 3)) digitalWrite(AADDR_3_PIN, HIGH); else digitalWrite(AADDR_3_PIN, LOW);
    if (addr & (1 << 4)) digitalWrite(AADDR_4_PIN, HIGH); else digitalWrite(AADDR_4_PIN, LOW);
    if (addr & (1 << 5)) digitalWrite(AADDR_5_PIN, HIGH); else digitalWrite(AADDR_5_PIN, LOW);

    // allocate data to correct pin
    if (data & (1 << 0)) digitalWrite(DATA_0_PIN, HIGH); else digitalWrite(DATA_0_PIN, LOW);
    if (data & (1 << 1)) digitalWrite(DATA_1_PIN, HIGH); else digitalWrite(DATA_1_PIN, LOW);
    if (data & (1 << 2)) digitalWrite(DATA_2_PIN, HIGH); else digitalWrite(DATA_2_PIN, LOW);
    if (data & (1 << 3)) digitalWrite(DATA_3_PIN, HIGH); else digitalWrite(DATA_3_PIN, LOW);
    if (data & (1 << 4)) digitalWrite(DATA_4_PIN, HIGH); else digitalWrite(DATA_4_PIN, LOW);
    if (data & (1 << 5)) digitalWrite(DATA_5_PIN, HIGH); else digitalWrite(DATA_5_PIN, LOW);
    if (data & (1 << 6)) digitalWrite(DATA_6_PIN, HIGH); else digitalWrite(DATA_6_PIN, LOW);
    if (data & (1 << 7)) digitalWrite(DATA_7_PIN, HIGH); else digitalWrite(DATA_7_PIN, LOW);

    // check axi address ready
    while(digitalRead(AREADY_PIN) == 0){
        clk_toggle();
    }
    digitalWrite(AVALID_PIN, HIGH);
    clk_toggle();
    digitalWrite(AVALID_PIN, LOW);

    // check axi write ready
    while(digitalRead(WREADY_PIN) == 0){
        clk_toggle();
    }
    digitalWrite(WVALID_PIN, HIGH);
    clk_toggle();
    digitalWrite(WVALID_PIN, LOW);
    // extra clock toggle
    clk_toggle();
    clk_toggle();
    return 0;
}

//###############################################################################################################################
// 0000  00000    0   0000        000  00000 0000  00000   0   0   0
// 0   0 0       0 0  0   0      0       0   0   0 0      0 0  00 00
// 0000  00000  00000 0   0       000    0   0000  00000 00000 0 0 0
// 0   0 0      0   0 0   0          0   0   0   0 0     0   0 0   0
// 0   0 00000  0   0 0000        000    0   0   0 00000 0   0 0   0
//###############################################################################################################################
// axi read stream from addr
// addr is 6 bit
// data is 8 bit
int read_stream(int addr){
    set_data_2_input();
    // allocate address to correct pin
    if (addr & (1 << 0)) digitalWrite(AADDR_0_PIN, HIGH); else digitalWrite(AADDR_0_PIN, LOW);
    if (addr & (1 << 1)) digitalWrite(AADDR_1_PIN, HIGH); else digitalWrite(AADDR_1_PIN, LOW);
    if (addr & (1 << 2)) digitalWrite(AADDR_2_PIN, HIGH); else digitalWrite(AADDR_2_PIN, LOW);
    if (addr & (1 << 3)) digitalWrite(AADDR_3_PIN, HIGH); else digitalWrite(AADDR_3_PIN, LOW);
    if (addr & (1 << 4)) digitalWrite(AADDR_4_PIN, HIGH); else digitalWrite(AADDR_4_PIN, LOW);
    if (addr & (1 << 5)) digitalWrite(AADDR_5_PIN, HIGH); else digitalWrite(AADDR_5_PIN, LOW);

    // check axi address ready
    while(digitalRead(AREADY_PIN) == 0){
        clk_toggle();
    }
    digitalWrite(AVALID_PIN, HIGH);
    clk_toggle();
    digitalWrite(AVALID_PIN, LOW);
    digitalWrite(RREADY_PIN, HIGH);
    clk_toggle();

    // check read data is valid
    while(digitalRead(RVALID_PIN) == 0){
        clk_toggle();
    }

    int data_read = (digitalRead(DATA_7_PIN) << 7) +
                    (digitalRead(DATA_6_PIN) << 6) +
                    (digitalRead(DATA_5_PIN) << 5) +
                    (digitalRead(DATA_4_PIN) << 4) +
                    (digitalRead(DATA_3_PIN) << 3) +
                    (digitalRead(DATA_2_PIN) << 2) +
                    (digitalRead(DATA_1_PIN) << 1) +
                    (digitalRead(DATA_0_PIN) << 0);

    digitalWrite(RREADY_PIN, LOW);
    clk_toggle();

    return data_read;
}