#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

typedef struct {
    int n;
    int x;
    int *found;
    pthread_mutex_t *mtx;
    pthread_cond_t *cond;
} data; 

void *f(void *a) {
    data d = *(data *)a;
    while (1) {
        pthread_mutex_lock(d.mtx);
        if (*d.found) {
            pthread_mutex_unlock(d.mtx);
            return NULL;
        }

        int m = rand() % 1000;
        printf("thread %d guessed %d\n", d.n, m);
        if (m == d.x) {
            *d.found = 1;
            printf("thread %d FUCKING WON \n", d.n);
            pthread_cond_broadcast(d.cond);
            pthread_mutex_unlock(d.mtx);
            return NULL;
        }

        pthread_mutex_unlock(d.mtx);

        usleep(10000);

    }
}

void *f2(void *a) {
    data d = *(data *)a;
    pthread_mutex_lock(d.mtx);
    while (*d.found == 0) {
        pthread_cond_wait(d.cond, d.mtx);
    }
    pthread_mutex_unlock(d.mtx);
    return NULL;
}

int main(int argc, char **argv) {
    int n = 0;
    printf("n = ");
    scanf("%d", &n);

    srand(time(NULL));

    int x = rand() % 1000;
    printf("x = %d\n", x);

    pthread_t *threads = (pthread_t *)malloc((n + 1)* sizeof(pthread_t));
    data *d = (data *)malloc(n * sizeof(data));

    pthread_mutex_t mtx;
    pthread_mutex_init(&mtx, NULL);

    pthread_cond_t cond;
    pthread_cond_init(&cond, NULL);

    int ghici = -1;
    int found = 0;

    for (int i = 0; i < n; i++) {
        d[i].n = i;
        d[i].x = x;
        d[i].mtx = &mtx;
        d[i].cond = &cond;
        d[i].found = &found;
        pthread_create(&threads[i], NULL, f, &d[i]);
    }

    d[n].n = n;
    d[n].x = x;
    d[n].mtx = &mtx;
    d[n].cond = &cond;
    d[n].found = &found;
    pthread_create(&threads[n], NULL, f2, &d[n]);

    for (int i = 0; i < n; i++) {
        pthread_join(threads[i], NULL);
    }

    pthread_join(threads[n], NULL);

    free(threads);
    free(d);
    pthread_mutex_destroy(&mtx);
    pthread_cond_destroy(&cond);

    return 0;

}
