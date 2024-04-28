#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

typedef struct {
    int *row;
    int cols;
} thread_arg;

void *sum_row(void *arg) {
    thread_arg *data = (thread_arg *)arg;
    int sum = 0;
    for (int i = 0; i < data->cols; i++) {
        sum += data->row[i];
    }
    printf("Sum: %d\n", sum);
    return NULL;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: <matrix>\n");
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

    pthread_t *threads = (pthread_t *)malloc(rows * sizeof(pthread_t));
    thread_arg *args = (thread_arg *)malloc(rows * sizeof(thread_arg));

    for (int i = 0; i < rows; i++) {
        args[i].row = matrix[i];
        args[i].cols = cols;
        pthread_create(&threads[i], NULL, sum_row, &args[i]);
    }

    for (int i = 0; i < rows; i++) {
        pthread_join(threads[i], NULL);
    }

    for (int i = 0; i < rows; i++) {
        free(matrix[i]);
    }
    free(matrix);
    free(threads);
    free(args);

    return 0;
}
