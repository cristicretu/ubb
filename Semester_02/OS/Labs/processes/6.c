// c program that generates n random integers, sends them to a child via pipe, then the child calculates the average and sends the result back
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <time.h>

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <n>\n", argv[0]);
        return 1;
    }

    int n = atoi(argv[1]);

    if (n <= 0) {
        fprintf(stderr, "n must be a positive integer\n");
        return 1;
    }

    int p2c[2], c2p[2];

    if (pipe(p2c) == -1 || pipe(c2p) == -1) {
        perror("pipe");
        return 1;
    }

    pid_t pid = fork();

    if (pid == -1) {
        perror("fork");
        return 1;
    } else if (pid == 0) {
        // child
        close(p2c[1]);
        close(c2p[0]);

        int sum = 0;
        for (int i = 0; i < n; i++) {
            int x;
            if (read(p2c[0], &x, sizeof(x)) == -1) {
                perror("read");
                return 1;
            }
            sum += x;
        }

        double avg = (double) sum / n;

        if (write(c2p[1], &avg, sizeof(avg)) == -1) {
            perror("write");
            return 1;
        }

        close(p2c[0]);
        close(c2p[1]);
    } else {
        // parent
        
        close(p2c[0]);
        close(c2p[1]);

        srand(time(NULL));

        for (int i = 0; i < n; i++) {
            int x = rand() % 1000;
            if (write(p2c[1], &x, sizeof(x)) == -1) {
                perror("write");
                return 1;
            }
        }

        wait(0);
        double avg;
        if (read(c2p[0], &avg, sizeof(avg)) == -1) {
            perror("read");
            return 1;
        }

        printf("Average: %.2f\n", avg);

        close(p2c[1]);
        close(c2p[0]);
    }

    return 0;
}
