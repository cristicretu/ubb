#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <ctype.h>
#include <semaphore.h>
#include <unistd.h>
#include "pthread_barrier.h"

typedef struct {
    int id, n;
    pthread_barrier_t *barrier;
    sem_t *sems;
} data;

void *f(void *arg) {
    data d = *((data *) arg);
    pthread_barrier_wait(d.barrier);

    for (int i = 0; i < d.n; ++i) {
        sem_wait(&d.sems[i]);
        printf("Thread %d, entered checkpoint %d\n", d.id, i);
        int n = (random() % 101 + 100);
        usleep(n * 1000);
        sem_post(&d.sems[i]);
    }
    printf("Thread %d has finished\n", d.id);

    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Usage: ./a.out <n>:int\n");
        exit(1);
    }
    int n = atoi(argv[1]);

    int m = 1;
    for (int i = 0; i < n; ++i) {
        m *= 2;
    }
    pthread_barrier_t barrier;
    sem_t *sems = (sem_t *)malloc(n * sizeof(sem_t));
    pthread_t *threads = (pthread_t *)malloc(m * sizeof(pthread_t));

    pthread_barrier_init(&barrier, NULL, m);
    data *args = malloc(m * sizeof(data));

    int step = 2;
    for (int i = 0; i < n; ++i) {
        sem_init(&sems[i], 0, m / step);
        step *= 2;
    }

    for (int i = 0; i < m; ++i) {
        args[i].n = n,
        args[i].id = i,
        args[i].sems = sems,
        args[i].barrier = &barrier;
        pthread_create(&threads[i], NULL, f, (void *)&args[i]);
    }

    for (int i = 0; i < m; ++i) {
        pthread_join(threads[i], NULL);
    }

    pthread_barrier_destroy(&barrier);
    for (int i = 0; i < n; ++i) {
        sem_destroy(&sems[i]);
    }

    free(args);
    free(threads);
    free(sems);
    return 0;
}
