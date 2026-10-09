/*
 * Write a C program that creates N threads (N is a command line argument).
Each thread will receive a unique index I from the main process between 0 and N-1 inclusively.
Each thread will execute M rounds (M also a command line argument) with the following steps:
    - generate 2 random numbers (A - between 1 and 20, B - between 1 and 3) and will add A*B to a thread local variable and will print its new value
    - increment the appropriate position in a shared frequency vector for the values of A
    - increment the appropriate position in a shared frequency vector for the values of B
    - wait for all the threads to finish their current round before beginning a new one
Once all threads terminate the main process will print the frequency vectors for A and B.
*/
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "pthread_barrier.h"
#include <string.h>

typedef struct {
    int *a;
    int *b;
    int m;
    int sum;
    pthread_mutex_t *mutex;
    pthread_barrier_t *barrier;
} data;

void *f(void *args) {
    data d = *(data *)args;

    for (int i = 0; i < d.m; ++i) {
        pthread_barrier_wait(d.barrier);
        int a = rand() % 20 + 1, b = rand() % 3 + 1;
        d.sum += a * b;
        printf("Generated numbers: [%d] and [%d] ->> The local sum is now %d\n", a,b, d.sum);
        pthread_mutex_lock(d.mutex);
        d.a[a - 1] += 1;
        d.b[b - 1] += 1;
        pthread_mutex_unlock(d.mutex);
    } 

    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: %s N M\n", argv[0]);
        return 1;
    }
    int n = atoi(argv[1]), m = atoi(argv[2]);

    srand(time(NULL));

    pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));
    data *args = (data *)malloc(n * sizeof(data));
    pthread_barrier_t barrier;
    pthread_barrier_init(&barrier, NULL, n);

    pthread_mutex_t mutex;
    pthread_mutex_init(&mutex, NULL);

    int *a = (int *)malloc(20 * sizeof(int));
    int *b = (int *)malloc(3 * sizeof(int));
    memset(a, 0, 20 * sizeof(int));
    memset(b, 0, 3 * sizeof(int));

    for (int i = 0; i < n; ++i) {
        args[i].a = a;
        args[i].b = b;
        args[i].sum = 0;
        args[i].mutex = &mutex;
        args[i].barrier = &barrier;
        args[i].m = m;
        pthread_create(&threads[i], NULL, f, &args[i]);
    }

    for (int i = 0; i < n; ++i) {
        pthread_join(threads[i], NULL);
    }

    for (int i = 0; i < 20; ++i) {
        printf("%d ", a[i]);
    }
    printf("\n");

    for (int i = 0; i < 3; ++i) {
        printf("%d ", b[i]);
    }
    printf("\n");

    free(threads);
    free(args);
    free(a);
    free(b);
    pthread_barrier_destroy(&barrier);
    pthread_mutex_destroy(&mutex);
    return 0;
}
