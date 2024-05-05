#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

typedef struct {
    int a;
    int b;
    int id;
} Result;

void *f(void *arg) {
    srand(pthread_self());
    int n = rand() % 100 + 1;
    int m = rand() % 100 + 1;

    Result *ans = (Result *) malloc(sizeof(Result));
    ans->a = n;
    ans->b = m;
    ans->id = pthread_self();

    printf("Thread (%d), generated numbers: [ %d ] and [ %d ]\n", pthread_self(), n, m);
    return ans;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        perror("Usage: ./a.out <number>");
        exit(1);
    }

    int n = atoi(argv[1]);

    int p2c[2];

    if (-1 == pipe(p2c)) {
      perror("pipe"); 
      exit(1);
    }

    pid_t pid = fork();

    if (pid < 0) {
        perror("fork error");
        exit(1);
    } else if (pid == 0) {
        // we are in the child

        close(p2c[1]);

        for (int i = 0; i < n; ++i) {
            Result *r; 

            read(p2c[0], r, sizeof(Result));

            double avg = (double) (r->a + r->b) / 2.0;

            printf("C::> Thread(%d), average is %f\n", r->id, avg);
        }


        close(p2c[0]);
    } else {
        // we are in the parent

        close(p2c[0]);
        pthread_t *threads = (pthread_t *)malloc(n * sizeof(pthread_t));

        for (int i = 0; i < n; ++i) {
            pthread_create(&threads[i], NULL, f, NULL);
        }

        for (int i = 0; i < n; ++i) {
            Result *r;
            pthread_join(threads[i], (void**)&r);

            write(p2c[1], r, sizeof(Result));

            free(r);
        }

        close(p2c[1]);
        free(threads);
    }

    return 0;

}
