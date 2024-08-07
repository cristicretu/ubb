// create N threads
// on startup each thread generates a value and store it in a shared array
// after each thread generated its value, each thread attemts to steal 10% from another thread's value

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"

pthread_barrier_t barrier;
pthread_mutex_t *mutex;
int *arr;
int n;

void *f(void *arg) {
    int index = *((int *)arg);
    arr[index] = rand() % 1000;
    printf("arr[%d] = %d\n", index, arr[index]);
    pthread_barrier_wait(&barrier);
    int other;
    while (1) {
        other = rand() % n;
        if (other != index) {
            break;
        }
    }
    int steal = arr[other] / 10;
    pthread_mutex_lock(&mutex[index]);
    pthread_mutex_lock(&mutex[other]);
    arr[other] -= steal;
    arr[index] += steal;
    pthread_mutex_unlock(&mutex[index]);
    pthread_mutex_unlock(&mutex[other]);
    printf("After steal: arr[%d] = %d\n", index, arr[index]);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <n>\n", argv[0]);
        return 1;
    }
    n = atoi(argv[1]);

    pthread_t th[n];
    pthread_barrier_init(&barrier, NULL, n);
    mutex = (pthread_mutex_t *)malloc(n * sizeof(pthread_mutex_t));
    for (int i = 0; i < n; i++) {
        pthread_mutex_init(&mutex[i], NULL);
    }
    int *index = (int *)malloc(sizeof(int));
    arr = (int *)malloc(n * sizeof(int));
    for (int i = 0; i < n; i++) {
        index[i] = i;
        if (-1 == pthread_create(&th[i], NULL, f, &index[i])) {
            perror("pthread_create");
            return 1;
        }
    }

    for (int i = 0; i < n; i++) {
        if (-1 == pthread_join(th[i], NULL)) {
            perror("pthread_join");
            return 1;
        }
    }

    pthread_barrier_destroy(&barrier);
    for (int i = 0; i < n; i++) {
        pthread_mutex_destroy(&mutex[i]);
    }
    free(index);
    free(arr);
    return 0;
}
