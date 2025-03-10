#include "mult_mat.h"
#include <stdio.h>

void multiplyMatrices(int firstMatrix[SIZE][SIZE], int secondMatrix[SIZE][SIZE], int resultMatrix[SIZE][SIZE]){
    for (int i = 0; i < SIZE; i++){
        for (int j = 0; j < SIZE; j++){
            resultMatrix[i][j] = 0;
            for (int k = 0; k < SIZE; k++){
                resultMatrix[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }
    //printf("%d\n",firstMatrix[0][0]);

}


