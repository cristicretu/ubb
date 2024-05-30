#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

#define SIZE 10 
/*
 *

27. Write a C program that takes two numbers, N and M, as arguments from the command line. The program creates N "generator" threads that generate random lowercase letters and append them to a string with 128 positions. The program will create an additional "printer" thread that that waits until all the positions of the string are filled, at which point it prints the string and clears it. The N "generator" threads must generate a total of M such strings and the "printer" thread prints each one as soon as it gets to length 128.
*/

typedef struct {
    int id;
    char *buf;
    int *pos;
    int maxSize;
    int *m;
    pthread_mutex_t *mutex;
    pthread_cond_t *cond;
} data;

void *gen(void *arg) {
    data d = *(data *)arg;

    while (1) {
        pthread_mutex_lock(d.mutex);
        if (*(d.m) == 0) {
            pthread_cond_broadcast(d.cond);
            pthread_mutex_unlock(d.mutex);
            break;
        }

        while (*(d.pos) == d.maxSize) {
            pthread_cond_broadcast(d.cond);
            pthread_cond_wait(d.cond, d.mutex);
        }

        char c = 'a' + rand() % 26;

        d.buf[*(d.pos)] = c;
        *(d.pos) += 1;
        pthread_mutex_unlock(d.mutex);
    }
    
    return NULL;
}

void *print(void *arg) {
    data d = *(data *)arg;

    while (1) {
        pthread_mutex_lock(d.mutex);
        if (*(d.m) == 0) {
            pthread_mutex_unlock(d.mutex);
            break;
        }

        while (*(d.pos) < d.maxSize) {
            pthread_cond_broadcast(d.cond);
            pthread_cond_wait(d.cond, d.mutex);
        }

        printf("%s\n", d.buf);
        memset(d.buf, 0, d.maxSize);

        *(d.pos) = 0;
        *(d.m) -= 1;

        pthread_mutex_unlock(d.mutex);
    }
    return NULL;
}

int main(int argc, char**argv) {
    if (argc != 3) {
        printf("Usage: %s N M\n", argv[0]);
        return 1;
    }

    int N = atoi(argv[1]);
    int M = atoi(argv[2]);

    pthread_t *threads = malloc(sizeof(pthread_t) * (N + 1));

    pthread_mutex_t mutex;
    pthread_cond_t cond;

    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);

    int pos = 0;
    char *buf = malloc(sizeof(char) * 128);

    data args[N + 1];

    for (int i = 0; i < N; i++) {
        args[i].id = i;
        args[i].buf = buf;
        args[i].pos = &pos;
        args[i].maxSize = SIZE;
        args[i].m = &M;
        args[i].mutex = &mutex;
        args[i].cond = &cond;

        if (0 != pthread_create(&threads[i], NULL, gen, &args[i])) {
           perror("pthread_create");
            return 1;
        }
    }

    args[N].id = N;
    args[N].buf = buf;
    args[N].pos = &pos;
    args[N].maxSize = SIZE;
    args[N].m = &M;
    args[N].mutex = &mutex;
    args[N].cond = &cond;

    if (0 != pthread_create(&threads[N], NULL, print, &args[N])) {
        perror("pthread_create");
        return 1;
    }


    for (int i = 0; i < N; i++) {
        if (0 != pthread_join(threads[i], NULL)) {
            perror("pthread_join");
            return 1;
        }
    }

    if (0 != pthread_join(threads[N], NULL)) {
        perror("pthread_join");
        return 1;
    }

    free(threads);
    free(buf);

    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);



    return 0;
}
