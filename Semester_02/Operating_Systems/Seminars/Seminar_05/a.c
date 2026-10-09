#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>

pthread_mutex_t mtx;

// read write lock
pthread_rwlock_t rwlock;

typedef struct {
    char *s;
    int id;
} data;

int n;

void *r(void *arg) {
    for (int i = 0; i < 10; ++i) {
        pthread_rwlock_rdlock(&rwlock);
        printf("%d\n", n);
        pthread_rwlock_unlock(&rwlock);
    }
    return NULL;
}

void *w(void *arg) {
    for (int i = 0; i < 1000; ++i) {
        pthread_rwlock_wrlock(&rwlock);
        n++;
        pthread_rwlock_unlock(&rwlock);
    }
    return NULL;
}


int main(int argc, char *argv[]) {
    pthread_mutex_init(&mtx, NULL);
    int r_size = 5;
    int w_size = 2;

    pthread_t r_th[r_size];
    pthread_t w_th[w_size];

    pthread_rwlock_init(&rwlock, NULL);

    for (int i = 0; i < w_size; i++) {
        if (pthread_create(&w_th[i], NULL, w, NULL)) {
            perror("pthread_create");
        }
    }

    for (int i = 0; i < r_size; i++) {
        if (pthread_create(&r_th[i], NULL, r, NULL)) {
            perror("pthread_create");
        }
    }

    for (int i = 0; i < w_size; i++) {
        if (pthread_join(w_th[i], NULL)) {
            perror("pthread_join");
        }
    }

    for (int i = 0; i < r_size; i++) {
        if (pthread_join(r_th[i], NULL)) {
            perror("pthread_join");
        }
    }

    pthread_mutex_destroy(&mtx);
    pthread_rwlock_destroy(&rwlock);
    printf("%d\n", n);

    return 0;
}
