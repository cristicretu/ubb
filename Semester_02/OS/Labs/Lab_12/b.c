#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <sys/time.h>

typedef struct {
    int col;
} data;
int **matrix;
int rows, cols;


void *messi(void *arg) {
    data *d = (data *)arg; 
    int sum = 0;

    for (int i = 0; i < rows; ++i) {
        sum += matrix[i][d->col];
    }

    printf("Sum of column %d is %d\n", d->col, sum);
    int *result = malloc(sizeof(int));
    *result = sum;
    return result;
}

int main(int argc, char **argv) {

    FILE *f = fopen(argv[1], "r");
    if (f == NULL) {
        perror("Error opening file");
        exit(2);
    }

    rows = 0, cols = 0;
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), f) != NULL) {
        if (rows == 0) {
            for (int i = 0; buffer[i] != '\0'; i++) {
                if (buffer[i] == ' ' && buffer[i+1] != ' ' && buffer[i+1] != '\0') {
                    cols++;
                }
            }
            cols++;
        }
        rows++;
    }

    matrix = (int **)malloc(rows * sizeof(int *));
    for (int i = 0; i < rows; i++) {
        matrix[i] = (int *)malloc(cols * sizeof(int));
    }

    rewind(f);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            fscanf(f, "%d", &matrix[i][j]);
        }
    }
    fclose(f);

    struct timeval tv1, tv2;
    gettimeofday(&tv1, NULL);

    int *sum_cols = (int *)malloc(cols * sizeof(int));

    for (int j = 0; j < cols; ++j) {
        sum_cols[j] = 0;
        for (int i = 0; i < rows; ++i) {
            sum_cols[j] += matrix[i][j];
        }
    }

    for (int j = 0; j < cols; ++j) {
        printf("Sum of column %d is %d\n", j, sum_cols[j]);
    }

    gettimeofday(&tv2, NULL);
    printf("Total time = %f seconds\n", (double)(tv2.tv_usec - tv1.tv_usec)  / 1000000 + (double) (tv2.tv_sec - tv1.tv_sec));

    pthread_t *threads = (pthread_t *)malloc(cols * sizeof(pthread_t));
    data *args = (data *)malloc(cols * sizeof(data));

    gettimeofday(&tv1, NULL);

    for (int j = 0; j < cols; ++j) {
        args[j].col = j;
        pthread_create(&threads[j], NULL, messi, &args[j]);
    }

    int sum = 0;
    for (int j = 0; j < cols; ++j) {
        void *result;
        pthread_join(threads[j], &result);
        sum += *(int *)result;
        free(result);
    }

    gettimeofday(&tv2, NULL);
    printf("Sum is %d\n", sum);
    printf("Total time = %f seconds\n", (double)(tv2.tv_usec - tv1.tv_usec) / 1000000 + (double)(tv2.tv_sec - tv1.tv_sec));

    for (int i = 0; i < rows; ++i) {
        free(matrix[i]);
    }

    free(matrix);
    free(threads);
    free(args);
    return 0;
}

