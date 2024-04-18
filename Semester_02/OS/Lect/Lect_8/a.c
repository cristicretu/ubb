#include <stdio.h>
#include <pthread.h>

pthread_mutex_t ma, mb;
void *fa(void *a) {
    int i=0;
    for (i=0; i<100; i++) {
        pthread_mutex_lock(&ma);
        printf("a\n");
        pthread_mutex_unlock(&mb);
    }
   return NULL;
}

void *fb(void *a) {
    int i=0;
    for (i=0; i<100; i++) {
        pthread_mutex_lock(&mb);
        printf("b\n");
        pthread_mutex_unlock(&ma);
    }
    return NULL;
}


int main(int argc, char **argv) {
    pthread_t ta, tb;
    pthread_mutex_init(&ma, NULL);
    pthread_mutex_init(&mb, NULL);
    pthread_mutex_lock(&mb);
    pthread_create(&ta, NULL, fa, NULL);
    pthread_create(&tb, NULL, fb, NULL);
    pthread_join(ta, NULL);
    pthread_join(tb, NULL);
    pthread_mutex_destroy(&ma);
    pthread_mutex_destroy(&mb);;
    return 0;
}
