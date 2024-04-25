#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

#define BEARS 5
#define BEES 100 

int honey = 100;
pthread_mutex_t m;
pthread_cond_t c;

void* bear(void *a) {
    while (1) {
        pthread_mutex_lock(&m);
        if (honey >= 10) {
            printf("-");
            honey -= 10;
        } else {
            printf("!");
            pthread_cond_signal(&c);
        }
        pthread_mutex_unlock(&m);
    }
    return NULL;
}

void* bee(void *a) {
    while (1) {
        pthread_mutex_lock(&m);
        printf("+");
        honey += 1;
        pthread_mutex_unlock(&m);
    }
    return NULL;
}

void* ranger(void *a) {
    while (1) {
        pthread_mutex_lock(&m);
        while (honey >= 10) {
            pthread_cond_wait(&c, &m);
        }
        honey += 100;
        printf("H");
        pthread_mutex_unlock(&m);
    }
    return NULL;
}

int main(int argc, char **argv) {
    int i;
    pthread_t bees[BEES], bears[BEARS], ranger_t;

    pthread_mutex_init(&m, NULL);
    pthread_cond_init(&c, NULL);

    for (i = 0; i < BEES; i++) {
        pthread_create(&bees[i], NULL, bee, NULL);
    }

    for (i = 0; i < BEARS; i++) {
        pthread_create(&bears[i], NULL, bear, NULL);
    }

    pthread_create(&ranger_t, NULL, ranger, NULL);



    for (i = 0; i < BEES; i++) {
        pthread_join(bees[i], NULL);
    }

    for (i = 0; i < BEARS; i++) {
        pthread_join(bears[i], NULL);
    }

    pthread_join(ranger_t, NULL);

    pthread_mutex_destroy(&m);
    pthread_cond_destroy(&c);

    return 0;
}
