/*
 * Scrieti un program C care primeste un numar N ca argument la linia de comanda. Programul va crea N thread-uri si le va da cate un id distinct fiecaruia dintre ele. Thread-urile vor executa urmatorii pasi: 1. Fiecare thread genereaza 3 numere- intre 1 si 10 (inclusiv), le afiseaza pe ecran impreuna cu id-ul lui. 2. Fiecare thread asteapta ca toate celelalte thread-uri sa execute pasul 1. 3. Fiecare thread aduna numerele impare generate de el La • suma globala si scade din aceeasi suma toate numere le pare pe care le-a generat. Dupa ce toate thread-urile se incheie, procesul principal afiseaza valoarea sumei globale. Folositi mecanisme de sincronizare adecvate.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "pthread_barrier.h"

typedef struct {
    int id;
} data;

pthread_mutex_t mtx;
pthread_barrier_t b;
int sum = 0;

void *f(void *arg) {
    data d = *(data *)arg;

    int arr[3];
    for (int i = 0; i < 3; ++i) {
        arr[i]= rand() % 10 + 1;
    }

    printf("Thread %d has generated numbers %4d | %4d | %4d\n", d.id, arr[0], arr[1], arr[2]);;

    pthread_barrier_wait(&b);

    int add = 0, substract = 0;

    for (int i = 0; i < 3; ++i) {
        if (arr[i] & 1) {
            add += arr[i];
        } else {
            substract += arr[i];
        }
    }

    int total = add - substract;

    pthread_mutex_lock(&mtx);
    sum += total;
    pthread_mutex_unlock(&mtx);
    return NULL;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        perror("wrong number of arguments");
        exit(1);
    }

    int n = atoi(argv[1]);
    srand(time(NULL));

    pthread_mutex_init(&mtx, NULL);
    pthread_barrier_init(&b, NULL, n);
    pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));
    data *args = (data *)malloc(n * sizeof(data));

    for (int i = 0; i < n; ++i) {
        args[i].id = i;
        pthread_create(&threads[i], NULL, f, &args[i]);
    }

    for (int i = 0; i < n; ++i) {
        pthread_join(threads[i], NULL);
    }

    printf("Final sum is %d\n", sum);

    free(threads);
    free(args);
    pthread_mutex_destroy(&mtx);
    pthread_barrier_destroy(&b);
    return 0;
}
