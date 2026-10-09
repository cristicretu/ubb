/*
 * 4. Write a C program that creates N threads (N given as a command line argument). The main process opens a file F, provided as a command line argument (the file's contents are words of a maximum of 20 characters each separated by spaces). Each thread will take turns reading between 1 and 3 words from the file and concatenating them to a thread-local buffer until all the content of the file is read. Once the whole file is completely read, the threads return their local buffer and the main process will print the result from each thread. Ensure that each thread, after it does one reading pass, waits for the other threads to complete their reading attempt before starting a new reading pass.
 */
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <string.h>

typedef struct {
    int id;
    FILE *file;
    char *buf;
    pthread_barrier_t *b;
    pthread_mutex_t *m;
    int *mori;
} data;

typedef struct {
    char *buf;
} result;

void *func(void *arg) {
    data d = *(data *)arg;
    char *buf = malloc(21 * 3);
    buf[0]='\0';


    while (*d.mori == 0) {

        pthread_barrier_wait(d.b);
        int nrwords = rand() % 3 + 1;
        int citit = 0;

        pthread_mutex_lock(d.m);
        for (int i = 0; i < nrwords; i++) {
            char word[21];
            if (-1 == fscanf(d.file, "%s", word)) {
                *d.mori = 1;
                break;
            }
            strcat(buf, word);
            strcat(buf, " ");
            citit+=1;
        }
        pthread_mutex_unlock(d.m);

        /* printf("sunt%d, am citit cuvintele: %s\n",d.id, buf); */


        pthread_barrier_wait(d.b);

    }

    return buf;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        perror("you are very dumb");
        exit(1);
    }

    FILE *f= fopen(argv[1], "r");
    if (!f) {
        perror("you are very dumb");
        exit(1);
    }

    int n = atoi(argv[2]);

    pthread_barrier_t b;
    pthread_barrier_init(&b, NULL,  n);

    pthread_mutex_t m;
    pthread_mutex_init(&m, NULL);

    pthread_t *threads = malloc(n * sizeof(pthread_t));

    data *d = malloc(n * sizeof(data));

    int crapa = 0;

    for (int i = 0; i < n; i++) {
        d[i].file = f;
        d[i].b = &b;
        d[i].id = i;
        d[i].m = &m;
        d[i].mori = &crapa;
        pthread_create(&threads[i], NULL, func, &d[i]);
    }

    for (int i = 0; i < n; i++) {
        char *buf;
        pthread_join(threads[i], (void **)&buf);
        if (strlen(buf)) {
            printf("Thread %d: %s\n", i, buf);
        } 


        free(buf);
    }

    fclose(f);
    free(threads);
    pthread_barrier_destroy(&b);
    free(d);

    return 0;


}

