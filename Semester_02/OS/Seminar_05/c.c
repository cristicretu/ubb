#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <ctype.h>

typedef struct {
    char *s;
    int id;
} data;

void *upcase(void *arg) {
    data d = *((data *)arg);
    char *copy = (char *)malloc(strlen(d.s) * sizeof(char) + 1);
    strcpy(copy, d.s);
    for (int i = 0; i < strlen(copy); i++) {
        if (islower(copy[i])) {
            copy[i] = toupper(copy[i]);
        }
    }
    return copy;
}

int main(int argc, char *argv[]) {
    int size = argc - 1;
    pthread_t threads[size];

    data *args = (data *)malloc(size * sizeof(data));
    for (int i = 0; i < size; i++) {
        args[i].s = argv[i + 1];
        args[i].id = i;
        if (pthread_create(&threads[i], NULL, upcase, &args[i])) {
            perror("pthread_create");
        }
    }

    for (int i = 0; i < size; i++) {
        void *res;
        if (pthread_join(threads[i], &res)) {
            perror("pthread_join");
        }
        printf("%s ", (char *)res);
        free(res);
    }

/*     for (int i = 1; i < argc; i++) { */
/*         printf("%s ", argv[i]); */
/*     } */

    free(args);


    return 0;
}
