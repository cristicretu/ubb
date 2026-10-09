#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include "pthread_barrier.h"

typedef struct {
    int id;
    int *flag;
    pthread_mutex_t *mutex;
    pthread_barrier_t *barrier;
} data;

void *f(void *arg) {
    data d = *(data *)arg;
    pthread_barrier_wait(d.barrier);

    while (1) {
        int a = rand() % 111112;
        pthread_mutex_lock(d.mutex);
        if (*d.flag == 0) {
            printf("Thread %d: %d\n", d.id, a);
            if (a % 1001 == 0) {
                *d.flag = 1;
                break;
            }
        } else {
            break;
        }
        pthread_mutex_unlock(d.mutex);
        usleep(100);
    }
    pthread_mutex_unlock(d.mutex);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Usage: %s <number>\n", argv[0]);
        return 1;
    }

    int n = atoi(argv[1]);

    pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));
    pthread_barrier_t *barrier = malloc(sizeof(pthread_barrier_t));
    pthread_barrier_init(&barrier, NULL, n);
    pthread_mutex_t *mutex = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(&mutex, NULL);

    data *d = malloc(n * sizeof(data));
    int *flag = malloc(sizeof(int));
    *flag = 0;

    for (int i = 0; i < n; i++) {
        d[i].id = i;
        d[i].flag = flag;
        d[i].mutex = mutex;
        d[i].barrier = barrier;
        pthread_create(&threads[i], NULL, f, &d[i]);
    }

    for (int i = 0; i < n; i++) {
        pthread_join(threads[i], NULL);
    }

    pthread_barrier_destroy(&barrier);
    pthread_mutex_destroy(&mutex);
    free(d);
    free(flag);
    free(mutex);
    free(barrier);
    free(threads);

    return 0;
}
