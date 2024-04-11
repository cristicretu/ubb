#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

int main(int argc, char *argv[]) {
    int p2c[2], c2p[2];
    srand(time(NULL));
    if (-1 == pipe(p2c)) {
        perror("pipe");
        exit(1);
    }
    if (-1 == pipe(c2p)) {
        perror("pipe");
        exit(1);
    }

    int f = fork();
    if (0 > f) {
        perror("fork");
        exit(1);
    } else if (0 == f) {
        close(p2c[1]);
        close(c2p[0]);

        int size;
        if (-1 == read(p2c[0], &size, sizeof(int))) {
            perror("read");
            close(p2c[0]);
            close(c2p[1]);
            exit(1);
        }

        int sum = 0;
        for (int i = 0; i < size; i++) {
            int x;
            if (-1 == read(p2c[0], &x, sizeof(int))) {
                perror("read");
                close(p2c[0]);
                close(c2p[1]);
                exit(1);
            }
            sum += x;
        }

        if (-1 == write(c2p[1], &sum, sizeof(int))) {
            perror("write");
            close(p2c[0]);
            close(c2p[1]);
            exit(1);
        }

        close(p2c[0]);
        close(c2p[1]);
        exit(0);
    } else {
        close(p2c[0]);
        close(c2p[1]);
        int n;
        scanf("%d", &n);

        if (n > 0) {
            int *arr = (int *) malloc(n * sizeof(int));

            for (int i = 0; i < n; i++) {
                arr[i] = rand() % 100;
                printf("%d ", arr[i]);
            }
            printf("\n");

            if (-1 == write(p2c[1], &n, sizeof(int))) {
                perror("write");
                close(p2c[1]);
                free(arr);
                wait(0);
                exit(1);
            }

            for (int i = 0; i < n; i++) {
                if (-1 == write(p2c[1], &arr[i], sizeof(int))) {
                    perror("write");
                    close(p2c[1]);
                    free(arr);
                    wait(0);
                    exit(1);
                }
            }
            free(arr);

            int sum = 0;
            if (-1 == read(c2p[0], &sum, sizeof(int))) {
                perror("read");
                close(p2c[1]);
                close(c2p[0]);
                wait(0);
                exit(1);
            }
            printf("Sum: %d\n", sum);

        }
        wait(NULL);
        close(p2c[1]);
        close(c2p[0]);
    }
    return 0;
}
