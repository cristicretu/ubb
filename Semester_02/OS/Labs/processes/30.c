/*
 * Relay: Create a C program that reads a number n from the standard input and created 4 * n threads. The threads will be split into teams of 4. In each team the threads will be numbered from 0 and will run according to the relay rules:
- Thread 0 from each team starts, waits (usleep) for 100 and 200 milliseconds, then passes the control to thread 1
- Thread 1 waits between 100 and 200 milliseconds then passes the control to thread 2
- Thread 2 waits between 100 and 200 milliseconds then passes the control to thread 3
- Thread 3 waits between 100 and 200 milliseconds, then prints a message indicating that the team has finished, then terminates
The team from which thread 3 terminates first is considered the winning team. Use appropriate synchronization mechanisms.
*/
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <unistd.h>

typedef struct {
    pthread_mutex_t *mtx;
    pthread_cond_t *cond;
    pthread_barrier_t *barrier;
    int index;
    int team;
    int *curr_index;
} data;

void *f(void *args) {
    data d = *(data *)args;
    pthread_barrier_wait(d.barrier);

    pthread_mutex_lock(d.mtx);
    while (d.index != *d.curr_index) {
        pthread_cond_wait(d.cond, d.mtx);
    }
    usleep((rand() % 101 + 100) * 1000);
    (*d.curr_index)++;

    if (d.index == 3) {
        printf("Team %d has finished\n", d.team);
    } else {
        pthread_cond_broadcast(d.cond);
    }
    pthread_mutex_unlock(d.mtx);
    return NULL;
}

int main(int argc, char **argv) {
    int n;
    scanf("%d", &n);

    if (n <= 0) {
        perror("you dumb");
        exit(1);
    }

    pthread_t **threads = (pthread_t **)malloc(n * sizeof(pthread_t *));
    pthread_mutex_t **mtx = (pthread_mutex_t **)malloc(n * sizeof(pthread_mutex_t *));
    pthread_cond_t **cond = (pthread_cond_t **)malloc(n * sizeof(pthread_cond_t *));
    data **args = (data **)malloc(n * sizeof(data *));

    for (int i = 0; i < n; i++) {
        threads[i] = (pthread_t *)malloc(n * sizeof(pthread_t));
        mtx[i] = (pthread_mutex_t *)malloc(n * sizeof(pthread_mutex_t));
        pthread_mutex_init(mtx[i], NULL);
        cond[i] = (pthread_cond_t *)malloc(n * sizeof(pthread_cond_t));
        pthread_cond_init(cond[i], NULL);
        args[i] = (data *)malloc(n * sizeof(data));
    }

    pthread_barrier_t *b = (pthread_barrier_t *)malloc(sizeof(pthread_barrier_t));
    pthread_barrier_init(b, NULL, n * 4);

    for (int i = 0; i < n; i++) {
        int *curr_index = (int *)malloc(sizeof(int));
        *curr_index = 0;
        for (int j = 0; j < 4; j++) {
            args[i][j].mtx = mtx[i];
            args[i][j].cond = cond[i];
            args[i][j].barrier = b;
            args[i][j].index = j;
            args[i][j].team = i;
            args[i][j].curr_index = curr_index;
            pthread_create(&threads[i][j], NULL, f, &args[i][j]);
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < 4; j++) {
            pthread_join(threads[i][j], NULL);
        }
    }

    for (int i = 0; i < n; i++) {
        pthread_mutex_destroy(mtx[i]);
        pthread_cond_destroy(cond[i]);
        free(threads[i]);
        free(mtx[i]);
        free(cond[i]);
        free(args[i]);
    }

    pthread_barrier_destroy(b);
    free(threads);
    free(mtx);
    free(cond);
    free(args);
    free(b);
    return 0;
}
