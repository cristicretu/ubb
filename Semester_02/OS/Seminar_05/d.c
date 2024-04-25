#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>

pthread_mutex_t mtx;

typedef struct {
    char *s;
    int id;
} data;

int n;


void *f(void *arg) {
    for (int i = 0; i < 1000; i++) {
        pthread_mutex_lock(&mtx);
        n++;
        pthread_mutex_unlock(&mtx);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    int size = 1000;
    pthread_t threads[size];
    pthread_mutex_init(&mtx, NULL);

    for (int i = 0; i < size; i++) {
        /* args[i].s = argv[i + 1]; */
        /* args[i].id = i; */
        if (pthread_create(&threads[i], NULL, f, NULL)) {
            perror("pthread_create");
        }
    }

    for (int i = 0; i < size; i++) {
        if (pthread_join(threads[i], NULL)) {
            perror("pthread_join");
        }
    }

/*     for (int i = 1; i < argc; i++) { */
/*         printf("%s ", argv[i]); */
/*     } */

    pthread_mutex_destroy(&mtx);
    printf("%d\n", n);


    return 0;
}
