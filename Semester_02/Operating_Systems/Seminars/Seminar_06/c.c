// create N threads
// on startup each thread generates a value and store it in a shared array
// after each thread generated its value, each thread attemts to steal 10% from another thread's value

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <semaphore.h>

pthread_mutex_t *mutex;
int n;

sem_t sem_1, sem_2, sem_3;

void *f(void *arg) {
    int index = *((int *)arg);
    printf("thread %d started\n", index);
    sem_wait(&sem_1);
    printf("Thread %d passed semaphore 1\n", index);
    sem_wait(&sem_2);
    printf("Thread %d passed semaphore 2\n", index);
    sem_wait(&sem_3);
    printf("Thread %d passed semaphore 3\n", index);
    printf("thread %d unlocks sem_1\n", index);
    sem_post(&sem_1);
    printf("thread %d unlocks sem_2\n", index);
    sem_post(&sem_2);
    printf("thread %d unlocks sem_3\n", index);
    sem_post(&sem_3);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <n>\n", argv[0]);
        return 1;
    }
    n = atoi(argv[1]);

    sem_init(&sem_1, 0, n/2);
    sem_init(&sem_2, 0, n/4);
    sem_init(&sem_3, 0, n/8);

    pthread_t th[n];
    mutex = (pthread_mutex_t *)malloc(n * sizeof(pthread_mutex_t));
    for (int i = 0; i < n; i++) {
        pthread_mutex_init(&mutex[i], NULL);
    }
    int *index = (int *)malloc(sizeof(int));
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

    sem_destroy(&sem_1);
    sem_destroy(&sem_2);
    sem_destroy(&sem_3);


    for (int i = 0; i < n; i++) {
        pthread_mutex_destroy(&mutex[i]);
    }
    free(index);
    return 0;
}
