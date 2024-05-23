#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "pthread_barrier.h"

pthread_mutex_t mutex;
pthread_barrier_t barrier;

typedef struct {
    int id;
    int *arr;
    int sum;
} data;

int size = 3;

void *populate(void *arg) {
    data *d = (data *)arg; 
    for (int i = 0; i < size; ++i) {
        d->arr[i] = rand() % 101;
        printf("%d ", d->arr[i]);
    }
    printf("\n");
    pthread_barrier_wait(&barrier);
    return NULL;
}

void *process(void *arg) {
    data *d = (data *)arg;
    pthread_barrier_wait(&barrier);
    d->sum = 0;
    for (int i = 0; i < size; ++i) {
        pthread_mutex_lock(&mutex);
        if (d->arr[i] != 0) {
            d->sum += d->arr[i];
            d->arr[i] = 0;
            pthread_mutex_unlock(&mutex);
            usleep(10000);
            
        } else {
            pthread_mutex_unlock(&mutex);
        } 
    }
    printf("Thread[%d] has the local sum %d\n", d->id, d->sum);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        perror("dumb");
        exit(1);
    }

    srand(time(NULL));

    int n = atoi(argv[1]);
    pthread_t threads[n][3];
    data args[n][3];

    int arrs[n][size];
    pthread_mutex_init(&mutex, NULL);
    pthread_barrier_init(&barrier, NULL, 3);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 3; ++j) {
            args[i][j].id = i * 3 + j;
            args[i][j].arr = arrs[i];
        }

        pthread_create(&threads[i][0], NULL, populate, &args[i][0]);
        pthread_create(&threads[i][1], NULL, process, &args[i][1]);
        pthread_create(&threads[i][2], NULL, process, &args[i][2]);
        
        pthread_join(threads[i][0], NULL);
        pthread_join(threads[i][1], NULL);
        pthread_join(threads[i][2], NULL);
    }

    pthread_mutex_destroy(&mutex);
    pthread_barrier_destroy(&barrier);

    return 0;
}
