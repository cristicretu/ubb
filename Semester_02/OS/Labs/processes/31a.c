#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include "pthread_barrier.h"

typedef struct {
    int id;
    pthread_barrier_t *barrier;
} ThreadData;

void* generate_random_numbers(void *arg) {
    ThreadData *data = (ThreadData*)arg;
    pthread_barrier_wait(data->barrier); // Wait for all threads to be created

    srand(time(NULL) + data->id);
    int number;

    while (1) {
        number = rand() % 111112; // Generate number between 0 and 111111
        printf("Thread %d: %d\n", data->id, number);

        if (number % 1001 == 0) {
            printf("Thread %d found the number: %d\n", data->id, number);
            exit(0); // Exit program once number divisible by 1001 is found
        }
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <number_of_threads>\n", argv[0]);
        return 1;
    }

    int N = atoi(argv[1]);
    if (N <= 0) {
        fprintf(stderr, "Number of threads must be greater than 0\n");
        return 1;
    }

    pthread_t threads[N];
    ThreadData threadData[N];
    pthread_barrier_t barrier;

    pthread_barrier_init(&barrier, NULL, N);

    for (int i = 0; i < N; i++) {
        threadData[i].id = i;
        threadData[i].barrier = &barrier;
        if (pthread_create(&threads[i], NULL, generate_random_numbers, &threadData[i]) != 0) {
            perror("pthread_create");
            return 1;
        }
    }

    for (int i = 0; i < N; i++) {
        pthread_join(threads[i], NULL); // Wait for all threads to finish
    }

    pthread_barrier_destroy(&barrier);

    return 0;
}
