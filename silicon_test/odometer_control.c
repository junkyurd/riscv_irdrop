#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "setup.h"
#include "odometer_control.h"

//###############################################################################################################################
//  000  0000   000  0   0 00000 00000 00000 0000
// 0   0 0   0 0   0 00 00 0       0   0     0   0       
// 0   0 0   0 0   0 0 0 0 00000   0   00000 0000 
// 0   0 0   0 0   0 0   0 0       0   0     0   0 
//  000  0000   000  0   0 00000   0   00000 0   0   
//###############################################################################################################################
// Odometer setup
// 21 odometers per core
// odometer 0-20 at primary odometer 21-41 at secondary
// writes same odometer control to start_odometer to end_odometer
int odometer_setup(int start_odometer, int end_odometer, int odometer_control){
    if (start_odometer > end_odometer){
        printf("Invalid Odometer Value...\n");
        return 1;
    }
    int num_odometer = end_odometer - start_odometer;
    write_stream(ADDR_ODOMETER_SEL_7_0, start_odometer);

    printf("Setting Odometers...\n");
    for (int i = 0; i < num_odometer+1; i++){
        write_stream(ADDR_ODOMETER_IN, odometer_control);
    }

    return 0;
}

// Odometer control load
int odometer_load(){
    write_stream(ADDR_ODOMETER_CONTROL, 0b00000000); // odometer resetb = 0
    write_stream(ADDR_ODOMETER_CONTROL, 0b10000000); // odometer resetb = 1
    printf("Loading Odometers...\n");
    write_stream(ADDR_ODOMETER_CONTROL, 0b10000001); // load = 1

    return 0;
}

// Odometer measurement trigger and wait for meas_wait_sec until reset meas_trig back to 0
int odometer_meas_trig(int meas_wait_usec){
    // meas_wait_usec is micro second
    struct timespec ts;
    ts.tv_sec = meas_wait_usec / 1000000; // convert mircoseconds to seconds
    ts.tv_nsec = (meas_wait_usec % 1000000) * 1000; // convert remaining to nanoseconds
    printf("Starting Odometers...\n");
    write_stream(ADDR_ODOMETER_CONTROL, 0b10000010); // meas_trig = 1
    nanosleep(&ts, NULL);
    write_stream(ADDR_ODOMETER_CONTROL, 0b10000000); // meas_trig = 0
    
    return 0;
}

// Odometer bit count read
// Reads and writes bit count to mem_file from start_odometer to end_odometer
int odometer_bc_read(const char *mem_file, int start_odometer, int end_odometer){
    // number of odometers to read is end_odometer - start_odometer
    if (start_odometer > end_odometer) {
        printf("Invalid Odometer Value...\n");
        return 1;
    }
    int num_odometer = end_odometer - start_odometer;
    int odometer_sel = start_odometer;

    printf("Odometer BC Reading...\n");
    FILE *file = fopen(mem_file, "w");
    if (file == NULL){
        printf("Error Opening File...\n");
        return 1;
    }

    for (int odometer = 0; odometer < num_odometer+1; odometer++){
        write_stream(ADDR_ODOMETER_SEL_7_0, odometer_sel);
        write_stream(ADDR_ODOMETER_CONTROL, 0b10000100); // bit count read hit
        int bitcount = (read_stream(ADDR_ODOMETER_BIT_CNT_15_8) << 8) + (read_stream(ADDR_ODOMETER_BIT_CNT_7_0));
        fprintf(file, "%d, %d\n", odometer_sel, bitcount);
        odometer_sel++;    
    }    
    fclose(file);
    return 0;
}