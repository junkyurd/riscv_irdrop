#include <stdlib.h>
#include "mult_mat.h"
int main() __attribute__((optimize("O0")));

int main(){
  //int seed = 5;
  ////////////////////////////////////////////////////////////////////////////////
  // Memory Mapping
  // 1. Output
  volatile unsigned int *mem_stop_trig = (volatile unsigned int*) (0x000007FC) ; //Memory location for activating the stop signal
  volatile unsigned int *mem_sol_out_a = (volatile unsigned int*) (0x000007F8) ;
  volatile unsigned int *mem_sol_out_b = (volatile unsigned int*) (0x000007F4) ;
  volatile unsigned int *mem_sol_out_c = (volatile unsigned int*) (0x000007F0) ;
  volatile unsigned int *mem_sol_out_d = (volatile unsigned int*) (0x000007EC) ;
  volatile unsigned int *mem_sol_out_e = (volatile unsigned int*) (0x000007E8) ;
  volatile unsigned int *mem_sol_out_f = (volatile unsigned int*) (0x00000784) ;

  ////////////////////////////////////////////////////////////////////////////////

  int firstMatrix[SIZE][SIZE];
  int secondMatrix[SIZE][SIZE];
  int resultMatrix[SIZE][SIZE];
  srand(5); // Seed for rand number to keep things same for primary and secondary.
  *mem_sol_out_a = 0;
  *mem_sol_out_b = 0;
  *mem_sol_out_c = 0;
  *mem_sol_out_d = 0;
  *mem_sol_out_e = 0;
  *mem_sol_out_f = 0;
  int tempresult = 0;
  int sumresult = 0;
  // initial no op
  for (unsigned int i = 0; i < 1000; i++){
  	__asm__("nop\n\t");
  }
  //for (int riscv_loop = 0; riscv_loop < 25; riscv_loop++){
  while(1){  
  // make matrix from random number generator
    for (int i = 0; i < SIZE; i++){
      for (int j = 0; j < SIZE; j++){
        //firstMatrix[i][j] = (seed*2+riscv_loop)%50;
        //secondMatrix[i][j] = (seed*4+riscv_loop+1)%50;
        firstMatrix[i][j] = rand() % 50;
        secondMatrix[i][j] = rand() % 50;
      }
    }

    multiplyMatrices(firstMatrix, secondMatrix, resultMatrix);
    tempresult = resultMatrix[0][0];
    sumresult = sumresult + tempresult;
    *mem_sol_out_a = tempresult;
    *mem_sol_out_b = sumresult;
    
    //if (*mem_sol_out_a >= 999999){
//	    *mem_sol_out_a = 0;
//	    if (*mem_sol_out_b >= 999999){
//		    *mem_sol_out_b = 0;
//		    if (*mem_sol_out_c >= 999999){
//			    *mem_sol_out_c = 0;
//		    } else{
//			   *mem_sol_out_c = *mem_sol_out_c + 1; 
//		    }
//	    } else{
//		    *mem_sol_out_b = *mem_sol_out_b + 1;
//	    }
  //  } else{
//	    *mem_sol_out_a = *mem_sol_out_a + 1;
  //  }
    //*mem_sol_out_a = *mem_sol_out_a + 1;
    //*(mem_sol_out_b - riscv_loop) = resultMatrix[0][0];

    
  }
  for (unsigned int i = 0; i < 1000; i++) {
      __asm__("nop\n\t");
  }
  //*mem_stop_trig = 0x00000005;
  return 0;
}
