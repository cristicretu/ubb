/*Write a C program that reads a number n from standard input and creates n threads, numbered from 0 to n - 1. Each thread places a random number between 10 and 20 on the position indicated by its id in an array of integers. After all threads have placed their number in the array, each thread repeats the following: 
- Checks if the number on its own position is greater than 0. 
- If yes, it substracts 1 from all numbers of the array, except the one on its own position. 
- If not, the thread terminates. 
- If there are no numbers in the array that are greater than 0, except the number on the thread's index position, the thread terminates. 
After all threads terminate, the main process prints the array of integers. Use appropriate synchronization mechanisms. 
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include "pthread_barrier.h"

typedef struct {
    int *array;
    int n;
    int id;
    pthread_mutex_t *m;
    pthread_barrier_t *b;
} thread_data;

void *thread_function(void *arg) {
    thread_data d = *(thread_data *)arg;
    d.array[d.id] = rand() % 11 + 10;
    pthread_barrier_wait(d.b); /// wait for all threads to finish writing to the array
    
    while (1) {
        pthread_mutex_lock(d.m);
        int ok = 0;
        for (int i = 0; i < d.n; i++) {
            if (d.array[i] > 0 && i != d.id) {
                ok = 1;
                break;
            }
        }
        if (!ok) {
            pthread_mutex_unlock(d.m);
            break;
        }
        pthread_mutex_unlock(d.m);

        pthread_mutex_lock(d.m);
        for (int i = 0; i < d.n; i++) {
            if (i != d.id) {
                d.array[i]--;
            }
        }
        pthread_mutex_unlock(d.m);
    }

    return NULL;
}

int main() {
    int n;
    scanf("%d", &n);

    int *array = (int *)malloc(n * sizeof(int));
    thread_data *data = (thread_data *)malloc(n * sizeof(thread_data));
    pthread_mutex_t *m = malloc(sizeof(pthread_mutex_t));
    pthread_mutex_init(m, NULL);
    pthread_barrier_t *b = malloc(sizeof(pthread_barrier_t));
    pthread_barrier_init(b, NULL, n);
    pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));

    for (int i = 0; i < n; i++) {
        data[i].array = array;
        data[i].n = n;
        data[i].id = i;
        data[i].m = m;
        data[i].b = b;
        pthread_create(&threads[i], NULL, thread_function, &data[i]);
    }

    for (int i = 0; i < n; i++) {
        pthread_join(threads[i], NULL);
    }

    for (int i = 0; i < n; i++) {
        printf("%d ", array[i]);
    }

    pthread_mutex_destroy(m);
    pthread_barrier_destroy(b);
    free(data);
    free(m);
    free(b);
    free(threads);
    free(array);
    return 0;
}
