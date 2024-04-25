#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

int main(int argc, char *argv[]) {
    int c1_2p[2], c2_2p[2], p_2c1[2], p_2c2[2];
    if (-1 == pipe(c1_2p) || -1 == pipe(c2_2p) || -1 == pipe(p_2c1) || -1 == pipe(p_2c2)) {
        perror("pipe");
        exit(1);
    }
    int f1 = fork();
    if (-1 == f1) {
        perror("fork");
        exit(1);
    }
    if (0 == f1) {
        close(c1_2p[0]);
        close(c2_2p[0]);
        close(c2_2p[1]);
        close(p_2c1[1]);
        close(p_2c2[1]);
        close(p_2c2[0]);
        srand(getpid());
        int n = rand() % 100;
        if (-1 == write(c1_2p[1], &n, sizeof(n))) {
            perror("write");
            exit(1);
        }

        int res;
        if (-1 == read(p_2c1[0], &res, sizeof(res))) {
            perror("read");
            exit(1);
        }

        printf("Child 1: %d, %d\n", n, res);
        exit(0);
    } 

    int f2 = fork();
    if (-1 == f2) {
        perror("fork");
        exit(1);
    } else if (0 == f2) {
        close(c1_2p[0]);
        close(c2_2p[0]);
        close(c1_2p[1]);
        close(p_2c1[1]);
        close(p_2c2[1]);
        close(p_2c1[0]);
        srand(getpid());
        int n = rand() % 100;
        if (-1 == write(c2_2p[1], &n, sizeof(n))) {
            perror("write");
            exit(1);
        }

        int res;
        if (-1 == read(p_2c2[0], &res, sizeof(res))) {
            perror("read");
            exit(1);
        }

        printf("Child 2: %d, %d\n", n, res);
        exit(0);
    }

    close(c1_2p[1]);
    close(c2_2p[1]);

    int n1, n2;
    if (-1 == read(c1_2p[0], &n1, sizeof(int)) || -1 == read(c2_2p[0], &n2, sizeof(n2))) {
        perror("read");
        exit(1);
    }

    int res;
    if (n1 > n2) {
        res = 1;
    } else if (n1 == n2) {
        res = 0;
    } else {
        res = -1;
    }

    if (-1 == write(p_2c1[1], &res, sizeof(res)) || -1 == write(p_2c2[1], &res, sizeof(res))) {
        perror("write");
        exit(1);
    }

    close(p_2c1[1]);
    close(p_2c2[1]);
    close(c1_2p[0]);
    close(c2_2p[0]);
    close(p_2c1[0]);
    close(p_2c2[0]);

    wait(NULL);
    wait(NULL);
    return 0;
}
