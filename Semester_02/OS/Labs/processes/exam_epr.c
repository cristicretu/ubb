/*
 * Create a C program that receives 2 command line arguments N - integer and F - filename.
 * The program opens file F and creates 3 * N threads simultaneously.
 * In each group of 3 consecutive threads, the threads wait for one another to start.
 * Each thread from group number K (for all K between 0 and N-1) reads one word (sequence of characters delimited by space) from the file and the length of the longest read word from that group is saved on index K in a shared array.
 * After all threads terminate, the main process prints the shared array.
 */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

typedef struct {
    FILE *file;
    int id;
int group;
    int *arr;
    int *ans;
    pthread_mutex_t *mtx;
    pthread_barrier_t *b;
} data;

void *functie(void *args) {
    data d = *(data *)args;

    pthread_barrier_wait(d.b);

    char *word = (char *)malloc(128 * sizeof(char));
    int size = fscanf(d.file, "%s", word);
    int length = strlen(word);
    free(word);

//    printf("Thread %d, from group %d, read word %s, size %d\n", d.id, d.group, word, length);

    pthread_mutex_lock(d.mtx);
    if (length > *(d.ans)) {
        *(d.ans) = length;
    }
    pthread_mutex_unlock(d.mtx);
    
    pthread_barrier_wait(d.b);
    
    pthread_mutex_lock(d.mtx);
    d.arr[d.group] = *(d.ans);
    pthread_mutex_unlock(d.mtx);

    return NULL;
}

int main(int argc, char**argv) {
    if (argc != 3) {
        perror("provide 3");
        exit(1);
    }

    int n = atoi(argv[1]);
    FILE *f = fopen(argv[2], "r");

    pthread_t *threads = (pthread_t *)malloc((n * 3) * sizeof(pthread_t));
    pthread_barrier_t *barriers = (pthread_barrier_t *)malloc(n * sizeof(pthread_barrier_t));
    data *args = (data *)malloc((n * 3) * sizeof(data));
    int *a = (int *)malloc(n * sizeof(int));
    memset(a, 0, n * sizeof(int));
    int *rasp = (int *)malloc(n * sizeof(int));
    memset(rasp, 0, n * sizeof(int));

    pthread_mutex_t mtx;
    pthread_mutex_init(&mtx, NULL);

    for (int i = 0; i < n; ++i) {
        pthread_barrier_init(&barriers[i], NULL, 3);
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 3; ++j) {
            int index = i * 3 + j;
            args[index].file = f;
            args[index].id = index;
            args[index].group = i;
            args[index].arr = a; 
            args[index].mtx = &mtx;
            args[index].b = &barriers[i];
            args[index].ans = &rasp[i];

            pthread_create(&threads[index], NULL, functie, &args[index]);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 3; ++j) {
            int index = i * 3 + j;

            pthread_join(threads[index], NULL);
        }
    }

    for (int i = 0; i < n; ++i, printf(" ")) {
        printf("%d", a[i]);
    }
    printf("\n");

    fclose(f);
    free(rasp);
    free(threads);
    for (int i = 0; i < n; ++i) {
        pthread_barrier_destroy(&barriers[i]);
    }
    free(barriers);
    pthread_mutex_destroy(&mtx);
    free(a);
    free(args);
    return 0;
}
