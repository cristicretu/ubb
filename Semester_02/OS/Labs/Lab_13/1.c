/*
 *  Write a C program that creates N threads (N given as a command line argument). The threads will keep adding random numbers between -500 and +500 to a shared variable that initially has the value 0. The threads will terminate when the shared variable has an absolute value greater than 500. Ensure proper synchronization Print a message every time a thread modifies the variable.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

int ans = 0;

typedef struct {
    int id;
    int *a;
    pthread_mutex_t *mtx;
} data;

void *f(void *arg) {
    data d = *(data *)arg;
    while(1) {

        pthread_mutex_lock(d.mtx);

        if (abs(*d.a) > 500) {
            printf("thread %d finished\n", d.id);
            pthread_mutex_unlock(d.mtx);
            break;
        }
        int m = rand() % 1000 - 500;
        *(d.a) += m;
        printf("thread %d added %d fimmal result is %d\n", d.id, m, *d.a);
        pthread_mutex_unlock(d.mtx);
    }
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        perror("dumb");
        exit(1);
    }

    int n = atoi(argv[1]);

    pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));

    int *m = (int *)malloc(sizeof(int));
    *m = 0;

    pthread_mutex_t mtx;
    pthread_mutex_init(&mtx, NULL);

    srand(time(NULL));

    data *d = (data *)malloc(n * sizeof(data));

    for (int i = 0; i < n; i++) {
        d[i].id = i;
        d[i].a = m;
        d[i].mtx = &mtx;
        pthread_create(&threads[i], NULL, f, &d[i]);
    }

    for (int i = 0; i < n; i++) {
        pthread_join(threads[i], NULL);
    }

    printf("%d", *m);

    free(threads);
    return 0;
}
