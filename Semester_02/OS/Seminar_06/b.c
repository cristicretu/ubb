// 2 threads
// 1 thread produces values and adds them in an array until it's full (array has n)
// once the array is full the second thread does something with the array, clears it and allows thread 1 to continue
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

int arr[10];
int full = 0;
int repeats = 100;
pthread_cond_t cond;
pthread_mutex_t mutex;

void *producer(void *arg) {
    int n = repeats;
    while(n) {
        pthread_mutex_lock(&mutex);
        while(full) {
            pthread_cond_wait(&cond, &mutex);
        }
        for (int i = 0; i < 10; i++) {
            arr[i] = rand() % 100;
        }
        full = 1;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mutex);
        --n;
    }
    return NULL;
}

void *consumer(void *arg) {
    int n = repeats;
    while(n) {
        pthread_mutex_lock(&mutex);
        while(!full) {
            pthread_cond_wait(&cond, &mutex);
        }
        for (int i = 0; i < 10; i++) {
            printf("%4d ", arr[i]);
        }
        printf("\n");
        full = 0;
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mutex);
        --n;
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    srand(time(NULL));
    pthread_cond_init(&cond, NULL);
    pthread_mutex_init(&mutex, NULL);
    pthread_t th[2];
    pthread_create(&th[0], NULL, producer, NULL);
    pthread_create(&th[1], NULL, consumer, NULL);

    for (int i = 0; i < 2; i++) {
        pthread_join(th[i], NULL);
    }

    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);
    return 0;
}
