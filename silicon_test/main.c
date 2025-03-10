#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <wiringPi.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "setup.h"
#include "chip_operation.h"

int main(int argc, char *argv[]){
    
    // turn on power
    system("python3 ./psu_control/psu_operation_on.py");

    rpi_setup(argc,argv);


    // turn off power
    system("python3 ./psu_control/psu_operation_off.py");
}