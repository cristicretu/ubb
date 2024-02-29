#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    int **x  = malloc(sizeof(int*) * 10); 
    for (int i = 0; i < 10; ++i) {
        x[i] = malloc(sizeof(int) * 10);
    }


    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            x[i][j] = i * j;
        }
    }

    for (int i = 0; i < 10; ++i, printf("\n")) {
        for (int j = 0; j < 10; ++j) {
            printf("%d ", x[i][j]);
        }
    }

    /* for (int i = 0; i < 10; ++i) { */
    /*     free(x[i]); */
    /* } */

    /* free(x); */
    return 0;
}
