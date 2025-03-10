#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <signal.h>
#include <softPwm.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "setup.h"
#include "mult_mat.h"
#include "chip_operation.h"
#include "odometer_control.h"

//###############################################################################################################################
// 0   0 0000  00000 00000 00000        000     0     000   0   0 00000
// 0   0 0   0   0     0   0           0       0 0   0      0   0 0
// 0 0 0 0000    0     0   00000       0      00000  0      00000 00000
// 0 0 0 0   0   0     0   0           0      0   0  0      0   0 0
//  0 0  0   0 00000   0   00000        000   0   0   000   0   0 00000
//###############################################################################################################################
// writes lines from mem_file to selected cache from start_addr to end of the mem_file
// if mem_file is longer than cache depth, it will overflow and overwrite from 00000000
// addr width is 11 bits
// cache depth is 2048
// data width is 32 bits
// [0] Instr read start     [1] Data read start     [2] Instr read done      [3] Data read done
// [4] DDLS read start      [5] DDLS read done      [7:6] STREAM_SEL
int write_cache(const char *mem_file, int cache_sel, int core_sel, int start_addr){
    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_START, 0x00);

    // if core_sel = 0, primary core only
    // if core_sel = 1, secondary core only
    // if core_sel = 2 or not 0 or 1, both primary and secondary cores
    int core_mask = 0b11000000;
    if (core_sel == 0){
        // printf("Primary core selected\n");
        core_mask = 0b01000000;
    } else if (core_sel == 1){
        // printf("Secondary core selected\n");
        core_mask = 0b10000000;
    } else {
        // default case
        // printf("Both Primary and Secondary cores selected\n");
        //core_mask = 0b11000000;
    }
    write_stream(ADDR_CACHE_CONTROL, core_mask);

    // if cache_sel = 0, instr cache
    // if cache_sel = 1 default, data_cache 
    int addr_cache_data_in = ADDR_DATA_DATA_IN;
    int addr_cache_addr = ADDR_DATA_ADDR;
    if (cache_sel == 0){
        // printf("Instruction cache selected\n");
        addr_cache_data_in = ADDR_INSTR_DATA_IN;
        addr_cache_addr = ADDR_INSTR_ADDR;
    } else {
        // default case
        // printf("Data cache selected\n");
        //addr_cache_data_in = ADDR_DATA_DATA_IN;
        //addr_cache_addr = ADDR_DATA_ADDR;
    }

    write_stream(ADDR_RISCV_OFFSET, 0x00);
    // starting address is start_addr
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(addr_cache_addr, addr);
    }

    printf("Cache Streaming In...\n");
    // Open mem_file and read lines
    FILE *file;
    char line[9]; // buffer to hold a line from mem_file (8 digits + 1 for null)
    file = fopen(mem_file, "r");
    if (file == NULL){
        printf("Error Opening File...\n");
        return 1;
    }
    int line_count = 0;
    while (fgets(line, sizeof(line), file)) {
        line[strcspn(line, "\n")] = '\0'; // remove the newline character if it exists

        if (strlen(line) == 0){
            continue;
        } // skip empty line

        //printf("Line %d read: '%s' length: %zu\n", line_count, line, strlen(line));
        unsigned long full_data = (unsigned long)strtoul(line, NULL, 16);
        //printf("Interpreted as: %lu (0x%lX)\n", full_data, full_data);
        write_stream(ADDR_RISCV_OFFSET, 0x00);
        for (int i = 0; i < 4; i++){
            
            unsigned int data = (full_data >> (8 * i)) & 0xFF;
            write_stream(addr_cache_data_in, data);
        }
        
    }

    fclose(file);
    return 0;
}

// reset cache data to 32'b0 from start_addr to end_addr
int clear_data_cache(int start_addr, int end_addr){
    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_START, 0x00);
    // clear data cache from start_addr to end_addr on both primary and secondary
    if (start_addr > end_addr) {
        printf("Invalid Address Value\n");
        return 1;
    }
    int num_lines = end_addr - start_addr;

    write_stream(ADDR_CACHE_CONTROL, 0b11000000);

    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_OFFSET, 0x00);
    // starting address is start_addr
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(ADDR_DATA_ADDR, addr);
    }

    for (int i = 0; i < num_lines+1; i++){
        for (int i = 0; i < 4; i++){
            write_stream(ADDR_DATA_DATA_IN, 0x00);
        }
    }

    return 0;
}

//###############################################################################################################################
// 0000  00000   0   0000         000     0     000   0   0 00000
// 0   0 0      0 0  0   0       0       0 0   0      0   0 0
// 0000  00000 00000 0   0       0      00000  0      00000 00000
// 0   0 0     0   0 0   0       0      0   0  0      0   0 0
// 0   0 00000 0   0 0000         000   0   0   000   0   0 00000
//###############################################################################################################################
// read data from selected cache from start_addr to sen_addr and write to mem_file
// addr width is 11 bits
// cache depth is 2048
// data width is 32 bits
int read_cache(const char *mem_file, int cache_sel, int core_sel, int start_addr, int end_addr){
    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_START, 0x00);

    // if core_sel = 0, primary core only
    // if core_sel = 1, secondary core only default
    // cannot select both cores for read mode
    int core_mask = 0b10000000;
    if (core_sel == 0){
        // printf("Primary core selected\n");
        core_mask = 0b01000000;
    } else {
        // printf("Secondary core selected\n");
        //core_mask = 0b10000000;
    }
    write_stream(ADDR_CACHE_CONTROL, core_mask);

    // if cache_sel = 0, instr cache
    // if cache_sel = 1 default, data_cache 
    int addr_cache_data_out = ADDR_DATA_DATA_OUT;
    int addr_cache_addr = ADDR_DATA_ADDR;
    int read_start_mask = 0b00000010;
    int read_done_mask = 0b00001000;
    if (cache_sel == 0){
        // printf("Instruction cache selected\n");
        addr_cache_data_out = ADDR_INSTR_DATA_OUT;
        addr_cache_addr = ADDR_INSTR_ADDR;
        read_start_mask = 0b00000001;
        read_done_mask = 0b00000100;
    } else {
        // default case
        // printf("Data cache selected\n");
        //addr_cache_data_in = ADDR_DATA_DATA_IN;
        //addr_cache_addr = ADDR_DATA_ADDR;
    }

    write_stream(ADDR_RISCV_OFFSET, 0x00);
    // starting address is start_addr
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(addr_cache_addr, addr);
    }
    // number of lines to read is end_address - start_addr
    if (start_addr > end_addr) {
        printf("Invalid Address Value\n");
        return 1;
    }
    int num_lines = end_addr - start_addr;
    printf("Cache Streaming Out...\n");

    FILE *file = fopen(mem_file, "w");
    if (file == NULL){
        printf("Error Opening File...\n");
        return 1;
    }

    for (int i = 0; i < num_lines+1; i++){
        write_stream(ADDR_CACHE_CONTROL, read_start_mask | core_mask);
        while((read_stream(ADDR_CACHE_CONTROL) & read_done_mask) != read_done_mask){
            clk_toggle();
        }
        int data[4];
        for (int i = 0; i < 4; i++){
            data[i] = read_stream(addr_cache_data_out);
        }
        fprintf(file, "%02X%02X%02X%02X\n", data[3], data[2], data[1], data[0]);
    }


    fclose(file);
    return 0;
}

// read data stored at target SRAM and return them as long*
unsigned long* read_cache_as_array(int cache_sel, int core_sel, int start_addr, int end_addr){
    // compare to plain read_cache it will return pointer which holding the values it read
    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_START, 0x00);

    // if core_sel = 0, primary core only
    // if core_sel = 1, secondary core only default
    // cannot select both cores for read mode
    int core_mask = 0b10000000;
    if (core_sel == 0){
        // printf("Primary core selected\n");
        core_mask = 0b01000000;
    } else {
        // printf("Secondary core selected\n");
        //core_mask = 0b10000000;
    }
    write_stream(ADDR_CACHE_CONTROL, core_mask);

    // if cache_sel = 0, instr cache
    // if cache_sel = 1 default, data_cache 
    int addr_cache_data_out = ADDR_DATA_DATA_OUT;
    int addr_cache_addr = ADDR_DATA_ADDR;
    int read_start_mask = 0b00000010;
    int read_done_mask = 0b00001000;
    if (cache_sel == 0){
        // printf("Instruction cache selected\n");
        addr_cache_data_out = ADDR_INSTR_DATA_OUT;
        addr_cache_addr = ADDR_INSTR_ADDR;
        read_start_mask = 0b00000001;
        read_done_mask = 0b00000100;
    } else {
        // default case
        // printf("Data cache selected\n");
        //addr_cache_data_in = ADDR_DATA_DATA_IN;
        //addr_cache_addr = ADDR_DATA_ADDR;
    }

    write_stream(ADDR_RISCV_OFFSET, 0x00);
    // starting address is start_addr
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(addr_cache_addr, addr);
    }
    // number of lines to read is end_address - start_addr
    if (start_addr > end_addr) {
        printf("Invalid Address Value...\n");
        unsigned long *cache_out = (unsigned long *)malloc(1 * sizeof(unsigned long));
        cache_out[0] = 1;
        return cache_out;
    }
    int num_lines = end_addr - start_addr;
    printf("Cache Streaming Out...\n");
    // allocate memory enough to hold cache read data
    unsigned long *cache_out = (unsigned long *)malloc((num_lines+1) * sizeof(unsigned long));

    for (int i = 0; i < num_lines+1; i++){
        write_stream(ADDR_CACHE_CONTROL, read_start_mask | core_mask);
        while((read_stream(ADDR_CACHE_CONTROL) & read_done_mask) != read_done_mask){
            clk_toggle();
        }
        int data[4];
        for (int i = 0; i < 4; i++){
            data[i] = read_stream(addr_cache_data_out);
        }
        cache_out[i] = (unsigned long)data[0] |
                    ((unsigned long)data[1] << 8) |
                    ((unsigned long)data[2] << 16) |
                    ((unsigned long)data[3] << 24);
    }

    return cache_out;
}

//###############################################################################################################################
// 0   0 0000  00000 00000 00000       0000  0000  0      000   
// 0   0 0   0   0     0   0           0   0 0   0 0     0      
// 0 0 0 0000    0     0   00000       0   0 0   0 0      000  
// 0 0 0 0   0   0     0   0           0   0 0   0 0         0 
//  0 0  0   0 00000   0   00000       0000  0000  00000  000  
//###############################################################################################################################
// DDLS SRAM width is 256 bits
// DDLS SRAM depth is 1024
// DDLS SRAM stores mismatch data between primary and secondary cores
// Use python version
/*
int write_ddls(int start_addr){
    // for debug purpose only
    // gets modified based on debug purpose
    write_stream(ADDR_RISCV_START, 0x00);
    write_stream(ADDR_DDLS_OFFSET, 0x00);
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(ADDR_DDLS_ADDR, addr);
    }
    printf("DDLS Streaming In...\n");
    write_stream(ADDR_DDLS_OFFSET, 0x00);
    for (int line = 0; line < 16; line++){
        int data = (line << 4) + line;
        printf("%X\n", data);
        for (int i = 0; i < 32; i++){
            write_stream(ADDR_DDLS_DATA_IN, data);
        }
    }
    
    return 0;
}
*/

//###############################################################################################################################
// 0000  00000   0   0000        0000  0000  0      000   
// 0   0 0      0 0  0   0       0   0 0   0 0     0      
// 0000  00000 00000 0   0       0   0 0   0 0      000  
// 0   0 0     0   0 0   0       0   0 0   0 0         0 
// 0   0 00000 0   0 0000        0000  0000  00000  000  
//###############################################################################################################################
// DDLS SRAM width is 256 bits
// DDLS SRAM depth is 1024
// DDLS SRAM stores mismatch data between primary and secondary cores
// get error count (mismatch number) and read number of stored data based on it
unsigned long long int get_error_count(){
    unsigned long long int error_count = 0;
    write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 0x00);
    error_count = (unsigned long long int)read_stream(ADDR_RISCV_ERRCNT);
    // because each data read is 8bit need multiple readstream
    // read lsb -> msb
    error_count = error_count + (unsigned long long int)(read_stream(ADDR_RISCV_ERRCNT)<<8);
    error_count = error_count + (unsigned long long int)(read_stream(ADDR_RISCV_ERRCNT)<<16);
    error_count = error_count + (unsigned long long int)(read_stream(ADDR_RISCV_ERRCNT)<<24);
    error_count = error_count + ((unsigned long long int)read_stream(ADDR_RISCV_ERRCNT)<<32);

    return error_count;
}

int read_ddls(const char *mem_file, int start_addr, int end_addr){
    // Set SRAM write mode to stream
    write_stream(ADDR_RISCV_START, 0x00);
    write_stream(ADDR_DDLS_OFFSET, 0x00);
    // starting address is start_addr
    int addr = 0xFF;
    for (int i = 0; i < 4; i++){
        addr = (start_addr >> (8*i)) & 0xFF;
        write_stream(ADDR_DDLS_ADDR, addr);
    }
    
    FILE *file = fopen(mem_file, "w");
    if (file == NULL){
        printf("Error Opening File...\n");
        return 1;
    }

    // number of lines to read is end_address - start_addr
    if (start_addr > end_addr) {
        printf("Invalid Address Value...\n");
        fclose(file); // Close file no need to write anything
        return 1;
    }
    printf("DDLS Streaming Out...\n");
    int num_lines = end_addr - start_addr;
    
    write_stream(ADDR_DDLS_OFFSET, 0x00);
    for (int i = 0; i < num_lines+1; i++){
        write_stream(ADDR_CACHE_CONTROL, 0b00010000); // ddls read command
        while((read_stream(ADDR_CACHE_CONTROL) & 0b00100000) != 0b00100000){
            clk_toggle();
        }
        int data[32];
        for (int i = 0; i < 32; i++){
            data[i] = read_stream(ADDR_DDLS_DATA_OUT);
        }
        fprintf(file, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X\n",
        data[31], data[30], data[29], data[28], data[27], data[26], data[25], data[24], 
        data[23], data[22], data[21], data[20], data[19], data[18], data[17], data[16],
        data[15], data[14], data[13], data[12], data[11], data[10], data[9], data[8],
        data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
    }

    fclose(file);
    return 0;
}

// scan_out direct access to register values and control signals
// detailed information on each bit can be found on RTL
// Use python version
/*
unsigned int read_scan_out(int start_bit){
    unsigned int scan_out;
    // scan_out is total of 536 bit
    // read only partial from start_bit to size of int where start_bit is multiple of byte
    if (start_bit > 504){
        printf("Neet do adjust start_bit\n");
        return 0;
    }
    // force start_bit to be size of byte
    int modified_start_bit = start_bit - (start_bit % 8);
    // each scan_out offset is 8bit
    // size of unsigned int is 4 bytes -> 32bit
    int start_offset = modified_start_bit / 8;
    write_stream(ADDR_SCAN_OUT_OFFSET, start_offset);


    return scan_out;
}
*/

//###############################################################################################################################
// 0   0 00000   0   00000 00000 0000
// 0   0 0      0 0    0   0     0   0       
// 00000 00000 00000   0   00000 0000 
// 0   0 0     0   0   0   0     0   0 
// 0   0 00000 0   0   0   00000 0   0   
//###############################################################################################################################
// heater is noise injection tool
// causes IR drop by high frequency oscillations
int program_heater(int heater_in, int first_in){
    // based on heater in first heater in number of heaters are on
    // first in is the first injector to be enabled
    // after that heater_in is the number of injector from there

    // ADDR_HEATER_CONTROL
    // {GLOBAL_ENABLE, TEMP_SENSOR_EN, VOLT_SENSOR_EN, 2'b00, LOAD_HEATER_TOP, LOAD_HEATER_P, LOAD_HEATER_S}
    write_stream(ADDR_HEATER_S_SEL, 0x00); // Start writing from the heater 0
    //ADDR_HEATER_S_IN writes 8 heaters at a time total 84 heaters
    write_stream(ADDR_HEATER_CONTROL, 0x00); // Make sure global_enable and load are set to 0
    int heater_in_temp = heater_in;
    int first_row = 0;
    // start writing based on heater_in
    for (int i = 0; i < 11; i++){
        // Check if first_in
        if (first_in <= i*8){
            // Check how many are left in this row
            if (first_row == 0){
                printf("First row\n");
                first_row = 1;
                int first_row_heater_in = 0;
                if (heater_in > 8){
                    first_row_heater_in= (i+1)*8 - first_in;
                    write_stream(ADDR_HEATER_S_IN, 0xFF);
                    heater_in_temp = heater_in_temp - 8;
                } else {
                    first_row_heater_in= heater_in - (first_in - i*8);
                    write_stream(ADDR_HEATER_S_IN, (1<<first_row_heater_in)-1 );
                    heater_in_temp = heater_in_temp - first_row_heater_in;
                }
            }
            else {
                if (heater_in_temp >= 8) {
                    write_stream(ADDR_HEATER_S_IN, 0xFF);
                    heater_in_temp = heater_in_temp - 8;
                } else {
                    write_stream(ADDR_HEATER_S_IN, (1<<heater_in_temp)-1 );
                    heater_in_temp = 0;
                }
            }
        } else {
            write_stream(ADDR_HEATER_S_IN, 0x00);
        }
        
    }

    write_stream(ADDR_HEATER_CONTROL, 0x01); // load secondary heater, global_enable still 0
    digitalWrite(PWM_A_PIN, LOW);
    digitalWrite(PWM_B_PIN, LOW);

    return 0;
}

//###############################################################################################################################
//  000  0000  0   0
// 0     0   0 0   0
// 0     0000  0   0
// 0     0     0   0
//  000  0      000
//###############################################################################################################################
// RISC-V cpu control and read status

unsigned long long get_core_cycle(int core_sel){
    unsigned long long core_cycle = 0;
    //write_stream(ADDR_MAIN_DCO_EN, 0x00);
    write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 0x00);
    // if core_sel = 0, primary core only
    // if core_sel = 1, secondary core only default
    // cannot select both cores for read mode
    int addr_core_cycle = ADDR_CORE_CYCLE_SECONDARY;
    if (core_sel == 0){
        // printf("Primary core selected\n");
        addr_core_cycle = ADDR_CORE_CYCLE_PRIMARY;
    } else {
        // printf("Secondary core selected\n");
        //core_mask = 0b10000000;
    }
    core_cycle = read_stream(addr_core_cycle);
    core_cycle = core_cycle + (read_stream(addr_core_cycle) << 8);
    core_cycle = core_cycle + (read_stream(addr_core_cycle) << 16);
    core_cycle = core_cycle + (read_stream(addr_core_cycle) << 24);
    core_cycle = core_cycle + ((unsigned long long)read_stream(addr_core_cycle) << 32);
    core_cycle = core_cycle + ((unsigned long long)read_stream(addr_core_cycle) << 40);
    return core_cycle;
}

void dco_ddls_setup(int dco_control, int ddls_delay){
    // enable dco as RISCV clock input
    // RISCV can receive clock from either dco generated or external clock input
    write_stream(ADDR_RISCV_START, 0b00000000);
    write_stream(ADDR_CLK_CONTROL, 0b00000000);
    write_stream(ADDR_CLK_CONTROL, 0b01000000);
    write_stream(ADDR_MAIN_DCO_CONTROL, dco_control);
    write_stream(ADDR_MAIN_DCO_EN, 0x01);
    write_stream(ADDR_DELAY_SEL, ddls_delay);
}

// Under modificiation
int check_cpu_end(int rpi_clk_limit, int inject_cnt, long *inject_time, long *inject_lasting_time){
    // check if cpu program has terminated
    // for this irdrop experiment
    // cpu is set to run for specifc run cycle
    struct timeval start_program; // program start time
    struct timeval start_injection, end_injection; // time between injection
    long injection_seconds, injection_microseconds;

    // if the cpu does not terminate after certain rpi_clk_limit then terminate without
    // CPU done signal
    int rpi_clk_cnt = 0;
    write_stream(ADDR_HEATER_CONTROL, 0x80); // turn on global enable
    gettimeofday(&start_program, NULL);
    // injection
    while((read_stream(ADDR_RISCV_STATUS) & 0b00000001) != 0b00000001){
        clk_toggle_half_period(1000); // slightly faster toggle
        rpi_clk_cnt++;
        if (rpi_clk_cnt >= rpi_clk_limit){
            return 1; // injection cannot be done
        }
        if (rpi_clk_cnt == inject_cnt){
            gettimeofday(&start_injection, NULL);
            digitalWrite(PWM_A_PIN, HIGH);
            digitalWrite(PWM_B_PIN, HIGH);
            clk_toggle();
            digitalWrite(PWM_A_PIN, LOW);
            digitalWrite(PWM_B_PIN, LOW);
            gettimeofday(&end_injection, NULL);
        }
    }
    write_stream(ADDR_MAIN_DCO_EN, 0x00);
    injection_seconds = start_injection.tv_sec - start_program.tv_sec;
    injection_microseconds = start_injection.tv_usec - start_program.tv_usec;
    *inject_time = (injection_seconds * 1000000) + injection_microseconds;

    injection_seconds = end_injection.tv_sec - start_injection.tv_sec;
    injection_microseconds = end_injection.tv_usec - start_injection.tv_usec;
    *inject_lasting_time = (injection_seconds * 1000000) + injection_microseconds;
    printf("RPI CLK CNT %d\n", rpi_clk_cnt);
    if (rpi_clk_limit == 0 & (rpi_clk_cnt <= inject_cnt)){
        return 1;
    }

    return 0;
}

void cpu_start(unsigned long long riscv_run_cycle, int dco_control){
    // boots up CPU with frequency determined by dco_control
    // if riscv_run_cycle > 0 then it will stop based on riscv_run_cycle
    // if riscv_run_cycle = 0 then disable run_cycle and terminate using pre allocated CPU instruction
    if (riscv_run_cycle == 0){
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 0b00000000);
    } else {
        write_stream(ADDR_RISCV_RUN_CYCLE_OFFSET, 0b10000000);
        // stream in number of cycle to be run
        for (int i = 0; i < 8; i++){
            write_stream(ADDR_RISCV_RUN_CYCLE, ((riscv_run_cycle >> (8*i)) & 0xFF));
        }
    }
    // booting up
    write_stream(ADDR_RISCV_OFFSET, 0x00);
    write_stream(ADDR_BOOT_ADDR, 0x00);
    write_stream(ADDR_BOOT_ADDR, 0x00);
    write_stream(ADDR_BOOT_ADDR, 0x00);
    write_stream(ADDR_BOOT_ADDR, 0x00);
    write_stream(ADDR_MTVEC_ADDR, 0x00);
    write_stream(ADDR_MTVEC_ADDR, 0x00);
    write_stream(ADDR_MTVEC_ADDR, 0x00);
    write_stream(ADDR_MTVEC_ADDR, 0x00);
    write_stream(ADDR_RISCV_START, 0b00000000);
    write_stream(ADDR_CLK_CONTROL, 0b00000000);
    // starting clock
    write_stream(ADDR_CLK_CONTROL, 0b01000000);
    write_stream(ADDR_MAIN_DCO_CONTROL, dco_control);
    write_stream(ADDR_MAIN_DCO_EN, 0x01);
    write_stream(ADDR_DELAY_SEL, 0x00);
    write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 0x00);
    // cpu reset steps
    write_stream(ADDR_RISCV_START, 0b00000000);
    clk_toggle();
    write_stream(ADDR_RISCV_START, 0b00000010);
    clk_toggle();
    write_stream(ADDR_RISCV_START, 0b00001010);
    clk_toggle();
    write_stream(ADDR_RISCV_START, 0b00001110);
    clk_toggle();
    sleep(1);
    write_stream(ADDR_RISCV_START, 0b00001111);
    clk_toggle();
}

//###############################################################################################################################
// 00000 00000  000  00000 00000 0   0  000
//   0   0     0       0     0   00  0 0
//   0   00000  000    0     0   0 0 0 0  00
//   0   0         0   0     0   0  00 0   0
//   0   00000  000    0   00000 0   0  000
//###############################################################################################################################
// chip basic operation tests
// axi interface and SRAMs
int axi_register_verification(){
    printf("AXI Read/Write Functionality Verification...\n");
    write_stream(ADDR_DELAY_SEL, 0x04);
    write_stream(ADDR_ODOMETER_SEL_7_0, 0xA9);
    if ((0x04 != read_stream(ADDR_DELAY_SEL)) | (0xA9 != read_stream(ADDR_ODOMETER_SEL_7_0))){
        printf("Error: AXI Read/Write Failed...\n");
        return 1;
    } else{
        printf("AXI Read/Write Pass...\n");        
    }
    return 0;
}

int cache_verification(){
    printf("Cache Read/Write Functionality Verification...\n");
    // write to instr cache, data cache and ddls cache and read back
    // check all of them function correctly
    write_stream(ADDR_RISCV_START, 0x00);
    // if core_sel = 0, primary core only
    // if core_sel = 1, secondary core only
    // if core_sel = 2 or not 0 or 1, both primary and secondary cores
    int core_mask = 0b11000000;
    unsigned long lines[] = {0x00002197, 0x80018193, 0x00001517, 0xff850513};
    int line_length = sizeof(lines) / sizeof(lines[0]);  
    
    // Instruction cache stream in
    printf("Cache streaming in...\n");
    write_stream(ADDR_CACHE_CONTROL, core_mask);
    write_stream(ADDR_RISCV_OFFSET, 0x00);
    write_stream(ADDR_INSTR_ADDR, 0x00);
    write_stream(ADDR_INSTR_ADDR, 0x00);
    write_stream(ADDR_INSTR_ADDR, 0x00);
    write_stream(ADDR_INSTR_ADDR, 0x00);
    //write_stream(ADDR_RISCV_OFFSET, 0x00);
    for (int line = 0; line < line_length; line++){
        for (int i = 0; i < 4; i++){
            int data = (lines[line] >> (8 * i)) & 0xFF;
            write_stream(ADDR_INSTR_DATA_IN, data);
        }
    }
    
    // Data cache stream in
    printf("Cache streaming in...\n");
    write_stream(ADDR_RISCV_OFFSET, 0x00);
    write_stream(ADDR_DATA_ADDR, 0x00);
    write_stream(ADDR_DATA_ADDR, 0x00);
    write_stream(ADDR_DATA_ADDR, 0x00);
    write_stream(ADDR_DATA_ADDR, 0x00);
    //write_stream(ADDR_RISCV_OFFSET, 0x00);
    for (int line = 0; line < line_length; line++){
        for (int i = 0; i < 4; i++){
            int data = (lines[line] >> (8 * i)) & 0xFF;
            write_stream(ADDR_DATA_DATA_IN, data);
        }
    }

    // Read back and compare with the golden
    // Use read_cache_as_array
    int instr_primary_flag = 0;
    int instr_secondary_flag = 0;
    int data_primary_flag = 0;
    int data_secondary_flag = 0;
    // primary core, instr cache
    unsigned long *read_data = read_cache_as_array(0, 0, 0, 3);
    for (int line = 0; line < line_length; line++){         
        if (read_data[line] != ((lines[line]))){
            instr_primary_flag = 1;
        }
    }

    // secondary core, instr cache
    read_data = read_cache_as_array(0, 1, 0, 3);
    for (int line = 0; line < line_length; line++){   
        if (read_data[line] != ((lines[line]))){
            instr_secondary_flag = 1;
        }
    }

    // primary core, data cache
    read_data = read_cache_as_array(1, 0, 0, 3);
    for (int line = 0; line < line_length; line++){   
        if (read_data[line] != ((lines[line]))){
            data_primary_flag = 1;
        }
    }

    // secondary core, data cache
    read_data = read_cache_as_array(1, 1, 0, 3);
    for (int line = 0; line < line_length; line++){   
        if (read_data[line] != ((lines[line]))){
            data_secondary_flag = 1;
        }
    }

    if (instr_primary_flag || instr_secondary_flag) {
        printf("Error: Instruction Cache Read/Write Failed...\n");
    } else {
        printf("Instruction Cache Read/Write Pass...\n");
    }

    if (data_primary_flag || data_secondary_flag) {
        printf("Error: Data Cache Read/Write Failed...\n");
    } else {
        printf("Data Cache Read/Write Pass...\n");
    }

    return 0;
}

//###############################################################################################################################
// 0   0   0   00000 0   0
// 00 00  0 0    0   00  0
// 0 0 0 00000   0   0 0 0
// 0   0 0   0   0   0  00
// 0   0 0   0 00000 0   0
//###############################################################################################################################
void handle_sigint(int sig);
int rpi_setup(int argc, char *argv[]){
    // register the signal handler for SIGINT (ctrl+c)
    signal(SIGINT, handle_sigint);
    // wiring pi setup using bcm gpio mode pin number
    if (wiringPiSetupGpio() == -1){
        printf("Setup Error\n");
        return -1;
    }
    set_pin_mode();
    chip_reset();

    // handle arguments
    // -t: runs a verification test program to check if AXI and Cache read/write are valid
    // -r: read a text file holding run information
    //      first line holds a mem_file to read, data cache start_addr, end_addr to clear
    
    if (argc >= 2){
        if (strcmp(argv[1], "-t") == 0){
            axi_register_verification();
            cache_verification();
        }
        if (strcmp(argv[1], "-r") == 0){
            run_stress_test(0xCA, 0x50);
        }
        if (strcmp(argv[1], "-f") == 0){
            max_frequency_test(0xC2, 0xC5);
        }
    }
    return 0;
}

void rpi_terminate(){
    chip_reset();
}

// interrupt handler
// ctrl+c
void handle_sigint(int sig){
    printf("\n........................\nResetting and Exiting\n........................\n");
    write_stream(ADDR_MAIN_DCO_EN, 0x00);
    unsigned long long primary_core_cycle = get_core_cycle(0);
    unsigned long long secondary_core_cycle =get_core_cycle(1);
    printf("%llu, %llu\n", primary_core_cycle, secondary_core_cycle);
    unsigned long long int error_count = get_error_count();
    if (error_count > 1024){
        read_ddls("ddls_result.mem", 0, 1024 - 1);
    } else {
        read_ddls("ddls_result.mem", 0, error_count - 1);
    }
    rpi_terminate();
    exit(0);
}

int make_experiment_directory(int test){
    char dir_name[100];
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);

    // 0: stress test
    // 1: max frequency
    if (test == 0){
        strftime(dir_name, sizeof(dir_name), "stress_test_%m%d%y_%H%M", tm_info);
    } else if(test == 1){
        strftime(dir_name, sizeof(dir_name), "max_frequency_%m%d%y_%H%M", tm_info);
    } else {
        // default
        strftime(dir_name, sizeof(dir_name), "default_%m%d%y_%H%M", tm_info);
    }
    // create directory
    if (mkdir(dir_name, 0755) == -1){
        printf("Error Creating Directory...\n");
        return 1;
    }

    if (chdir(dir_name) == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }

    return 0;
}

// reference program to compare against RISC-V result
void rpi_matmult(unsigned long correct_read[25+1]){
    int seed = 5;
    int firstMatrix[SIZE][SIZE];
    int secondMatrix[SIZE][SIZE];
    int resultMatrix[SIZE][SIZE];
    srand(seed);
    for (int riscv_loop = 0; riscv_loop < 25; riscv_loop++){
        for (int i = 0; i < SIZE; i ++){
            for (int j = 0; j < SIZE; j++){
                firstMatrix[i][j] = rand() % 50;
                secondMatrix[i][j] = rand() % 50;
            }
        }
        multiplyMatrices(firstMatrix, secondMatrix, resultMatrix);
        correct_read[(25)-riscv_loop] = resultMatrix[0][0];
    }
    correct_read[26] = 25;
}

int max_frequency_test(int start_dco_contol, int end_dco_control){
    unsigned long correct_read[26] = {1895, 4357, 2421, 4403, 4290, 3418, 3417, 2722,
        4269, 3241, 1530, 4352, 3468, 2163, 3910, 1520, 1854, 2941, 3115, 4386, 1230, 3044,
        1744, 4264, 2474, 24};
    const char *mem_file = "./mem_file/mat_mult_true_rand/mat_mult_true_rand.mem";
    long inject_time;
    long inject_last_time;
    // new directory
    make_experiment_directory(1);
    FILE *error_summary_file = fopen("max_frequency_test_summary.csv", "w");
    fprintf(error_summary_file, "DCO Control,DDLS Error Cnt,Primary Correct Cnt,Secondary Correct Cnt\n");
    for (int dco_control = start_dco_contol; dco_control <= end_dco_control; dco_control = dco_control + 1){
        char buffer[100];
        // reset everything so previous results do not contamitate next one
        chip_reset();
        clear_data_cache((0x7F8>>2)-25, (0x7F8>>2));
        write_cache(mem_file, 0, 0, 0);
        write_cache(mem_file, 0, 1, 0);
        write_cache(mem_file, 1, 0, 0);
        write_cache(mem_file, 1, 1, 0);
        write_stream(ADDR_HEATER_CONTROL, 0x00);
        // cpu start
        cpu_start(277174600, dco_control);

        if (check_cpu_end(100000, 0, &inject_time, &inject_last_time)){
            printf("CPU did not terminate properly...\n");
        }
        printf("Core cycle: %llu\n", get_core_cycle(0));
        printf("Core cycle: %llu\n", get_core_cycle(1));

        unsigned long *primary_read = read_cache_as_array(1,0,(0x7F8>>2)-25,(0x7F8>>2));
        unsigned long *secondary_read = read_cache_as_array(1,1,(0x7F8>>2)-25,(0x7F8>>2));
        int primary_correct_cnt = 0;
        int secondary_correct_cnt = 0;
        for (int i = 0; i < 26; i++){
            //fprintf(cache_result, "%lu, %lu, %lu\n", correct_read[i], primary_read[i], secondary_read[i]);
            if (correct_read[i] == primary_read[i]){
                primary_correct_cnt++;
            }
            if (correct_read[i] == secondary_read[i]){
                secondary_correct_cnt++;
            }
        }

        unsigned long long int error_count = get_error_count();
        printf("Error number: %llu\n", error_count);
        // mismatch write
        sprintf(buffer, "0V9_dco_%02X.mem", dco_control);
        if (error_count >= 20){
            read_ddls(buffer, 0, 19);
        } else {
            read_ddls(buffer, 0, error_count-1);
        }

        fprintf(error_summary_file, "%X,%llu,%u,%u\n", dco_control, error_count, primary_correct_cnt, secondary_correct_cnt);
    }
    fclose(error_summary_file);
    rpi_terminate();
    if (chdir("..") == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }
    return 0;
}

int run_stress_test(int start_dco_control, int end_dco_control){
    unsigned long correct_read[26] = {1895, 4357, 2421, 4403, 4290, 3418, 3417, 2722,
        4269, 3241, 1530, 4352, 3468, 2163, 3910, 1520, 1854, 2941, 3115, 4386, 1230, 3044,
        1744, 4264, 2474, 24};
    struct timeval start, end;
    struct timeval heat_on;
    long seconds, microseconds, total_microseconds;
    long inject_time;
    long inject_lasting_time;
    const char *mem_file = "./mem_file/mat_mult_true_rand/mat_mult_true_rand.mem";
    
    make_experiment_directory(0);
    FILE *error_summary_file = fopen("stress_test_summary.csv", "w");
    fprintf(error_summary_file, "Program Time [us], DDDLS Delay, Heater In,Inject Cnt,Error Cnt,Primary Correct Cnt,Secondary Correct Cnt,Inject Time [us],Inject On Time [us]\n");

    // heater control loop
    int ddls_delay = 0;
    for (int heater_in = 15; heater_in = 43; heater_in = heater_in + 1){
        for (int first_in = 0; first_in < 1; first_in = first_in + 42){
            system("python3 ./psu_control/psu_control.py");
            // inject cnt loop
            for (int inject_cnt = 10; inject_cnt < 1000; inject_cnt = inject_cnt + 24){
                char buffer[100];
                // new directory for ech injection
                sprintf(buffer, "first_in_%02d_heater_in_%02d_inject_cnt_%02d", first_in, heater_in, inject_cnt);
                if (mkdir(buffer, 0755) == -1){
                    printf("Error Creating Directory...\n");
                    return 1;
                }
                if (chdir(buffer) == -1){
                    printf("Error Changing Directory...\n");
                    return 1;
                }

                chip_reset();
                clear_data_cache((0x7F8>>2)-25, (0x7F8>>2));
                write_cache(mem_file, 0, 2, 0);
                write_cache(mem_file, 1, 2, 0);

                // program noise injector
                program_heater(heater_in, first_in);
                write_stream(ADDR_HEATER_CONTROL, 0x80); // turn on global enable
                cpu_start(277174600,start_dco_control);
                if(check_cpu_end(0, inject_cnt, &inject_time, &inject_lasting_time)){
                    printf("CPU did not terminate properly...\n");
                }
                printf("Core cycle: %llu\n", get_core_cycle(0));
                printf("Core cycle: %llu\n", get_core_cycle(1));

                unsigned long *primary_read = read_cache_as_array(1,0,(0x7F8>>2)-25,(0x7F8>>2));
                unsigned long *secondary_read = read_cache_as_array(1,1,(0x7F8>>2)-25,(0x7F8>>2));

                FILE *cache_result = fopen("cache_mem", "w");
                if (cache_result == NULL){
                    printf("Error Opening File...\n");
                    return 1;
                }
                fprintf(cache_result, "Correct Read, Primary Read, Secondary Read\n");
                int primary_correct_cnt = 0;
                int secondary_correct_cnt = 0;

                for (int i = 0; i < 26; i++){
                    fprintf(cache_result, "%lu, %lu, %lu\n", correct_read[i], primary_read[i], secondary_read[i]);
                    if (correct_read[i] == primary_read[i]){
                        primary_correct_cnt++;
                    }
                    if (correct_read[i] == secondary_read[i]){
                        secondary_correct_cnt++;
                    }
                }
                
                fclose(cache_result);

                unsigned long long int error_count = get_error_count();
                printf("Error number: %llu\n", error_count);
                //gettimeofday(&start, NULL);
                if (error_count >= 50){
                    read_ddls("ddls_result.mem", 0, 49);
                } else {
                    read_ddls("ddls_result.mem", 0, error_count-1);
                }

                if (chdir("..") == -1){
                    printf("Error Changing Directory...\n");
                    return 1;
                }
                fprintf(error_summary_file, "%lu,%d,%d,%d,%llu,%u,%u,%u,%u\n", 0, first_in, heater_in, inject_cnt, error_count, primary_correct_cnt, secondary_correct_cnt, inject_time, inject_lasting_time);
            }
        }
    }
    fclose(error_summary_file);
    rpi_terminate();
    if(chdir("..") == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }
    return 0;
}

void dco_clock_test(){
    for (int dco_control = 0x00; dco_control <= 0xF; dco_control = dco_control + 0x1){
        chip_reset();
        write_stream(ADDR_MAIN_DCO_CONTROL, dco_control);
        write_stream(ADDR_CLK_CONTROL, 0x02);
        write_stream(ADDR_DEBUG_FREQ_OUT_SEL, 0x00);
        write_stream(ADDR_MAIN_DCO_EN, 0x01);
        clk_toggle();
        clk_toggle();
        clk_toggle();
        clk_toggle();
        clk_toggle();
        printf("Running DCO 0x%X\n",dco_control);
        getchar();
        write_stream(ADDR_MAIN_DCO_EN, 0x00);
        clk_toggle();
        clk_toggle();
        clk_toggle();
        clk_toggle();
        clk_toggle();
    }
}

int make_rare_event_dir(int dco_control, float vdd_test){
    char dir_name[100];
    char time_str[50];
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    strftime(time_str, sizeof(time_str), "%m%d%y_%H%M", tm_info);
    snprintf(dir_name, sizeof(dir_name), "rare_dco_%X_v_%.1f_%s", dco_control, vdd_test, time_str);

    if (mkdir(dir_name, 0755) == -1){
        printf("Error Creating Directory...\n");
        return 1;
    }

    if (chdir(dir_name) == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }
    return 0;
}

int program_injector_list(int *injector_list, int injector_count){
    const int NUM_BITS = 84; // total number of injectors
    const int BITS_PER_STREAM = 8; // number of bits per AXI data stream
    const int NUM_STREAMS = (NUM_BITS + BITS_PER_STREAM - 1) / BITS_PER_STREAM;

    // only enabling secondary core injectors
    write_stream(ADDR_HEATER_S_SEL, 0x00); // start enable write from injector 0
    write_stream(ADDR_HEATER_CONTROL, 0x00);

    int *streams = (int *)calloc(NUM_STREAMS, sizeof(int));
    if (streams == NULL){
        printf("Memory allocation failed...\n");
        return 1;
    }

    // set the bits based on the input positions
    for (int i = 0; i < injector_count; i++){
        int pos = injector_list[i]; // get the current position
        if (pos < 0 || pos > NUM_BITS){
            printf("Invalid injector valud: %d, skipping\n", pos);
            continue;
        }
        int stream_index = pos / BITS_PER_STREAM; // determine the stream index
        int bit_position = pos % BITS_PER_STREAM; // determine the bit position in the stream
        streams[stream_index] |= (1<<bit_position); // set the bit
    }

    // actual streaming in 
    for (int i = 0; i < NUM_STREAMS; i++){
        printf("Streams %d: %02X ", i, streams[i]);
        write_stream(ADDR_HEATER_S_IN, streams[i]);
    }
    printf("\n");

    // load heater
    // global enable still at 0
    write_stream(ADDR_HEATER_CONTROL, 0x01);
    digitalWrite(PWM_A_PIN, LOW); // make sure heater is not active
    digitalWrite(PWM_B_PIN, LOW);
    free(streams);
    return 0;
}

int run_deterministic_injection_rare_test(int dco_control, int inject_length){
    const int NUM_INJECTOR_ROW = 12;
    const int INJECTOR_PER_ROW = 7;

    const unsigned long long CPU_TOTAL_CYCLE = 0xFFFFFFFFFFF;
    
    char dir_name[100];
    char time_str[50];
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    strftime(time_str, sizeof(time_str), "%m%d%y_%H%M", tm_info);
    snprintf(dir_name, sizeof(dir_name), "DCO%X_Inject%d_%s", dco_control, inject_length, time_str);



    if (mkdir(dir_name, 0755) == -1){
        printf("Error Creating Directory...\n");
        return 1;
    }
    if (chdir(dir_name) == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }

    // instruction file
    const char *mem_file = "./mem_file/matmult_while/riscv.inst";

    // power reset
    system("python3 ./psu_control/psu_operation_off.py");
    system("python3 ./psu_control/psu_operation_measure.py");
    FILE *error_result = fopen("error_result.csv", "w");
    if (error_result == NULL){
        printf("Error Opening File...\n");
        return 1;
    }
    fprintf(error_result,"DCO,Injector Row,Inject Time,Inject Length,Error Count,Primary Data Current,Primary Data Total,Secondary Data Current,Secondary Data Total,Primary Core Cycle,Secondary Core Cycle\n");
    for (int injector_row = 0; injector_row < NUM_INJECTOR_ROW; injector_row++){
        snprintf(dir_name, sizeof(dir_name), "injector_row_%02d", injector_row);
        if (mkdir(dir_name, 0755) == -1){
            printf("Error Creating Directory...\n");
            return 1;
        }
        if (chdir(dir_name) == -1){
            printf("Error Changing Directory...\n");
            return 1;
        }
    
        int first_injector = injector_row * 7;
        int injector_list[] = {first_injector, first_injector+2, first_injector+4};
        int injector_count = sizeof(injector_list)/sizeof(injector_list[0]);

        // Find out if first_injector is even or odd, it will determine which PWM enable to turn on
        // even numbered injectors are controlled by PWM_A_PIN
        // odd numbered injectors are controlled by PWM_B_PIN
        int PWM_EN_PIN = PWM_A_PIN;
        if (first_injector % 2 != 0) {
            PWM_EN_PIN = PWM_B_PIN;
        }
        

        // loop through and inject noise at different points.
        for (int inject_time = 500; inject_time < 10000; inject_time = inject_time + 50){
            snprintf(dir_name, sizeof(dir_name), "injector_time_%02d", inject_time);
            if (mkdir(dir_name, 0755) == -1){
                printf("Error Creating Directory...\n");
                return 1;
            }
            if (chdir(dir_name) == -1){
                printf("Error Changing Directory...\n");
                return 1;
            }

            // start testing
            chip_reset();
            clear_data_cache((0x784>>2), (0x7FC>2)); // this clears data cache where outputs of this specific testbench will be stored
            write_cache(mem_file, 0, 2, 0);
            write_cache(mem_file, 1, 2, 0);
            program_injector_list(injector_list, injector_count);

            uint64_t primary_core_cycle = 0;
            uint64_t secondary_core_cycle = 0;
            unsigned long long int error_count = 0;
            unsigned long long int error_count_temp = 0;
            int cycles_no_increase_error_count = 0;

            //turn on global enable
            write_stream(ADDR_HEATER_CONTROL, 0x80);
            // start cpu
            cpu_start(CPU_TOTAL_CYCLE, dco_control);
            // multiple clock toggle to process axi streming in
            clk_toggle();
            clk_toggle();
            clk_toggle();
            clk_toggle();
            clk_toggle();

            // wait til time to inject
            for (int i = 0; i < inject_time; i++){
                // each "nop" is about 4~5 ns                    
                __asm__("nop\n\t");
                //digitalWrite(PWM_B_PIN, LOW);
            }
            digitalWrite(PWM_EN_PIN, HIGH);
            // each PWM_PIN digital Write is about
            // 16~30 ns
            for (int i = 0; i < inject_length; i++){
                __asm__("nop\n\t");
                //printf("Wait for the end\n");
            }
            digitalWrite(PWM_EN_PIN, LOW);

            for (int wait_cnt = 0; wait_cnt < 100; wait_cnt++){
                clk_toggle();
            }
            write_stream(ADDR_HEATER_CONTROL, 0x00);
            write_stream(ADDR_MAIN_DCO_EN, 0x00);
            error_count = get_error_count();
            primary_core_cycle = get_core_cycle(0);
            secondary_core_cycle = get_core_cycle(1);
            printf("Core cycle: %llu\n", primary_core_cycle);
            printf("Core cycle: %llu\n", secondary_core_cycle);

            unsigned long *primary_read = read_cache_as_array(1, 0, (0x7F4>>2),(0x7F8>>2));
            unsigned long *secondary_read = read_cache_as_array(1, 1, (0x7F4>>2),(0x7F8>>2));
            

            //fprintf(error_result,"
            //0:DCO, 1:Injector Row, 1:Inject Time, 2:Inject Length, 3:Error Count,
            //4:Primary Data Current, 5:Primary Data Total,
            //6:Secondary Data Current, 7:Secondary Data Total,
            //8:Primary Core Cycle,9:Secondary Core Cycle\n");
            fprintf(error_result, "%02X,%d,%d,%d,%d,%d,%d,%d,%d,%llu,%llu\n", dco_control, injector_row, inject_time, inject_length, error_count, primary_read[1], primary_read[0], secondary_read[1], secondary_read[0], primary_core_cycle, secondary_core_cycle);

            //get ddls result
            if (error_count > 100){
                read_ddls("ddls_result.mem", 0, 100-1);
            } else {
                read_ddls("ddls_result.mem", 0, error_count-1);
            }
            if (chdir("..") == -1){
                printf("Error Changing Directory...\n");
                return 1;
            }
        }
    }
    chip_reset();
    if (chdir("..") == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }

    //power down
    fclose(error_result);
    system("python3 ./psu_control/psu_operation_off.py");
}


int run_rare_event_test(){
    time_t start_time, end_time;

    int hours, minutes;
    double time_diff;

    // instructions for RISCV test
    const char *mem_file = "./mem_file/matmult_while/riscv.inst";;

    int start_dco_control = 0xCA; // Main VDD at 1V, F = 1GHz
    // make new directory
    make_rare_event_dir(start_dco_control, 1.0);

    chip_reset();
    clear_data_cache((0x7F8)-25, (0x7F8>>2));

    write_cache(mem_file, 0, 2, 0);
    write_cache(mem_file, 1, 2, 0);

    write_stream(ADDR_HEATER_CONTROL, 0x00);
    unsigned long long primary_core_cycle;
    unsigned long long secondary_core_cycle;
    unsigned long long int error_count;

    FILE *error_result = fopen("rare_event_error_result.csv", "w");
    if (error_result == NULL){
        printf("Error Opening File...\n");
        return 1;
    }
    fprintf(error_result, "Primary Core Cycle,Cumulative Error Count\n");
    time(&start_time);
    cpu_start(0, start_dco_control);
    
    // custom copy of check_cpu_end
    int sub_cnt = 0;
    while((read_stream(ADDR_RISCV_STATUS) & 0b00000010) != 0b00000010){
        clk_toggle();
        sub_cnt++;
        if (sub_cnt == 9999){
            sub_cnt = 0;
            error_count = get_error_count();
            printf("llu\n", error_count);
        }
    }
    // turn off clock
    write_stream(ADDR_MAIN_DCO_EN, 0x00);
    time(&end_time);
    
    primary_core_cycle = get_core_cycle(0);
    secondary_core_cycle = get_core_cycle(1);
    printf("Core cycle: %llu\n", primary_core_cycle);
    printf("Core cycle: %llu\n", secondary_core_cycle);

    // grab data cache to check stored data
    unsigned long *primary_read = read_cache_as_array(1, 0, (0x7F8>>2)-25,(0x7F8>>2));
    unsigned long *secondary_read = read_cache_as_array(1, 0, (0x7F8>>2)-25,(0x7F8>>2));
    FILE *cache_result = fopen("data_cache_result.mem", "w");
    if (cache_result == NULL){
        printf("Error Opening File....\n");
        return 1;
    }
    fprintf(cache_result, "Primary Core,Secondary Core\n");
    fprintf(cache_result,"%lu,%lu\n",primary_core_cycle,secondary_core_cycle);
    for (int i = 0; i < 26; i++){
        fprintf(cache_result,"%lu,%lu\n",primary_read[i],secondary_read[i]);
    }
    // time in program
    // not totally needed can be commented out
    time_diff = difftime(end_time, start_time);
    hours = (int)(time_diff/3600);
    minutes = (int)((time_diff - (hours*3600))/60);
    printf("Time passed: %d hours and %d minutes\n", hours, minutes);
    fprintf(cache_result, "%d:%d\n",hours, minutes);
    fclose(cache_result);

    // ERROR COUNT (MISMATCH BETWEEN TWO CORES)
    // return ddls read for error count
    error_count = get_error_count();
    if (error_count > 1024){
        // too big to print out all
        read_ddls("ddls_result.mem", 0, 1024-1);    
    } else {
        read_ddls("ddls_result.mem", 0, error_count-1);
    }
    fclose(error_result);
    chip_reset();
    if (chdir("..") == -1){
        printf("Error Changing Directory...\n");
        return 1;
    }
    return 0;
}