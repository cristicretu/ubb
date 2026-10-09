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
    for (int i = 0; i < strlen(d.s); i++) {
        if (islower(d.s[i])) {
            d.s[i] = toupper(d.s[i]);
        }
    }
    return NULL;
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
        if (pthread_join(threads[i], NULL)) {
            perror("pthread_join");
        }
    }

    for (int i = 1; i < argc; i++) {
        printf("%s ", argv[i]);
    }

    free(args);

    return 0;
}
