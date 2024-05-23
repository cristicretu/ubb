#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

typedef struct {
    int **matrix;
    int rows;
    int col_index;
} thread_arg;

void *sum_column(void *arg) {
    thread_arg *data = (thread_arg *)arg;
    int sum = 0;
    for (int i = 0; i < data->rows; i++) {
        sum += data->matrix[i][data->col_index];
    }
    printf("Sum of column %d: %d\n", data->col_index, sum);
    return NULL;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <matrix>\n", argv[0]);
        exit(1);
    }

    FILE *f = fopen(argv[1], "r");
    if (f == NULL) {
        perror("Error opening file");
        exit(2);
    }

    int rows = 0, cols = 0;
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), f) != NULL) {
        if (rows == 0) { // Count columns in the first row
            for (int i = 0; buffer[i] != '\0'; i++) {
                if (buffer[i] == ' ' && buffer[i+1] != ' ' && buffer[i+1] != '\0') {
                    cols++;
                }
            }
            cols++; // Account for the last number before newline
        }
        rows++;
    }

    int **matrix = (int **)malloc(rows * sizeof(int *));
    for (int i = 0; i < rows; i++) {
        matrix[i] = (int *)malloc(cols * sizeof(int));
    }

    rewind(f);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fscanf(f, "%d", &matrix[i][j]);
        }
    }

    fclose(f);

    pthread_t *threads = (pthread_t *)malloc(cols * sizeof(pthread_t));
    thread_arg *args = (thread_arg *)malloc(cols * sizeof(thread_arg));

    clock_t start, end;
    double sequential_time, threaded_time;

    // Sequential execution
    start = clock();
    for (int j = 0; j < cols; j++) {
        int sum = 0;
        for (int i = 0; i < rows; i++) {
            sum += matrix[i][j];
        }
        printf("Sequential Sum of column %d: %d\n", j, sum);
    }
    end = clock();
    sequential_time = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Sequential execution time: %f seconds\n", sequential_time);

    // Threaded execution
    start = clock();
    for (int j = 0; j < cols; j++) {
        args[j].matrix = matrix;
        args[j].rows = rows;
        args[j].col_index = j;
        pthread_create(&threads[j], NULL, sum_column, &args[j]);
    }

    for (int j = 0; j < cols; j++) {
        pthread_join(threads[j], NULL);
    }
    end = clock();
    threaded_time = ((double) (end - start)) / CLOCKS_PER_SEC;
    printf("Threaded execution time: %f seconds\n", threaded_time);

    for (int i = 0; i < rows; i++) {
        free(matrix[i]);
    }
    free(matrix);
    free(threads);
    free(args);

    

    return 0;
}
