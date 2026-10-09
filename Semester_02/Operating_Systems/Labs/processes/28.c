#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

typedef struct {
    int *array;
    int *sorted;
    int *n;
    pthread_cond_t *cond;
    pthread_mutex_t *mtx;
    int id;
} data;

void *f(void *args) {
    data d = *(data *)args;

    while (1) {
        pthread_mutex_lock(d.mtx);
        int good = 1;

        if (*(d.n) > 1) {
            for (int i = 1; i < *(d.n); ++i) {
                if (d.array[i] < d.array[i - 1]) {
                    good = 0;
                    break;
                }
            }
        }

        if (good) {
            *(d.sorted) = 1;
            pthread_mutex_unlock(d.mtx);
            pthread_cond_broadcast(d.cond);
            break;
        }

        int i = rand() % (*(d.n));
        int j = rand() % (*(d.n));

        if ((i != j) && ((i < j && d.array[i] > d.array[j]) || (i > j && d.array[i] < d.array[j])))  {
            int temp = d.array[i];
            d.array[i] = d.array[j];
            d.array[j] = temp;
        }
        pthread_mutex_unlock(d.mtx);
    }

    return NULL;
}

void *print(void *args) {
    data d = *(data *)args;

    while (1) {
        pthread_mutex_lock(d.mtx);
        if (*(d.sorted)) {
            pthread_mutex_unlock(d.mtx);
            break;
        }
        pthread_cond_wait(d.cond, d.mtx);
        pthread_mutex_unlock(d.mtx);
    }

    for (int i = 0; i < *(d.n); ++i, printf(" ")) {
        printf("%d", d.array[i]);
    }
    printf("\n");

    return NULL;
}

int main(int argc, char **argv) {
    int n;
    scanf("%d", &n);

    if (n <= 0) {
        printf("you dumb");
        return 0;
    }

    int *arr = (int *)malloc(n * sizeof(int));
    pthread_t *threads = (pthread_t *)malloc((n + 1) * sizeof(pthread_t));
    data *args = (data *)malloc((n + 1) * sizeof(data));
    pthread_cond_t cond;
    pthread_cond_init(&cond, NULL);
    pthread_mutex_t mtx;
    pthread_mutex_init(&mtx, NULL);

    srand(time(NULL));

    for (int i = 0; i < n; ++i, printf(" ")) {
        arr[i] = rand() % 1001;
        printf("%d", arr[i]);
    }
    printf("\n");

    int sorted = 0;

    for (int i = 0; i < n; ++i) {
        args[i].id = i;
        args[i].array = arr;
        args[i].sorted = &sorted;
        args[i].cond = &cond;
        args[i].n = &n;
        args[i].mtx = &mtx;
        
        pthread_create(&threads[i], NULL, f, &args[i]);
    }

    args[n].id = n;
    args[n].array = arr;
    args[n].sorted = &sorted;
    args[n].cond = &cond;
    args[n].n = &n;
    args[n].mtx = &mtx;

    pthread_create(&threads[n], NULL, print, &args[n]);

    for (int i = 0; i < n; ++i) {
        pthread_join(threads[i], NULL);
    }

    pthread_join(threads[n], NULL);

    free(arr);
    free(threads);
    free(args);
    pthread_mutex_destroy(&mtx);
    pthread_cond_destroy(&cond);
    return 0;
}
