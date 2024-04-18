#include <stdio.h>
#include <pthread.h>

pthread_mutex_t m;
int turn = 0; // 0 for a, 1 for b
void *fa(void *a) {
    int i=0;
    while (i < 100) {
        /* pthread_mutex_lock(&m); */
        if (turn == 0) {
            printf("a\n");
            i++;
            turn = 1;
        }
        /* pthread_mutex_unlock(&m); */
    }
    return NULL;
}

void *fb(void *a) {
    int i=0;
    while (i < 100) {
        /* pthread_mutex_lock(&m); */
        if (turn == 1) {
            printf("b\n");
            i++;
            turn = 0;
        }
        /* pthread_mutex_unlock(&m); */
    }
    return NULL;
}


int main(int argc, char **argv) {
    pthread_t ta, tb;
    pthread_mutex_init(&m, NULL);
    pthread_create(&ta, NULL, fa, NULL);
    pthread_create(&tb, NULL, fb, NULL);
    pthread_join(ta, NULL);
    pthread_join(tb, NULL);
    pthread_mutex_destroy(&m);
    return 0;
}
