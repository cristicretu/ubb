/*
 *  Write a C program that takes as command line arguments 2 numbers: N and M. The program will simulate a thread race that have to pass through M checkpoints. Through each checkpoint the threads must pass one at a time (no 2 threads can be inside the same checkpoint). Each thread that enters a checkpoint will wait between 100 and 200 milliseconds (usleep(100000) makes a thread or process wait for 100 milliseconds) and will print a message indicating the thread number and the checkpoint number, then it will exit the checkpoint. Ensure that no thread will try to pass through a checkpoint until all threads have been created.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <unistd.h>

typedef struct {
    int id, m;
    pthread_mutex_t *mutex;
    pthread_barrier_t *barrier;
} data;

void *f(void *args) {
    data d = *(data *)args;
    pthread_barrier_wait(d.barrier);

    for (int i = 0; i < d.m; i++) {
        pthread_mutex_lock(&d.mutex[i]);
        printf("Thread %d entered checkpoint %d\n", d.id, i);
        usleep((rand() % 101 + 100) * 1000);
        pthread_mutex_unlock(d.mutex);
    }
    printf("Thread %d finished\n", d.id);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        perror("dumb");
        exit(1);
    }

    int n = atoi(argv[1]); // number of threads
    int m = atoi(argv[2]); // number of checkpoints
                           
    pthread_t *threads = malloc(n * sizeof(pthread_t));
    pthread_mutex_t *mutex = malloc(m * sizeof(pthread_mutex_t));
    pthread_barrier_t barrier;
    pthread_barrier_init(&barrier, NULL, n);

    data *args = malloc(n * sizeof(data));
    for (int i = 0; i < n; i++) {
        args[i].id = i;
        args[i].m = m;
        args[i].mutex = mutex;
        args[i].barrier = &barrier;
        pthread_create(&threads[i], NULL, f, &args[i]);
    }

    for (int i = 0; i < n; i++) {
        pthread_join(threads[i], NULL);
    }

    for (int i = 0; i < m; i++) {
        pthread_mutex_destroy(&mutex[i]);
    }

    free(threads);
    free(mutex);
    free(args);
    pthread_barrier_destroy(&barrier);

    return 0;
}
