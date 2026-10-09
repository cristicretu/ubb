#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>


int main(int argc, char **argv) {
    if (argc < 2) {
        perror("At least one command pleas\n");
        exit(1);
    }

    struct timeval start, end;
    gettimeofday(&start, NULL);

    pid_t pid = fork();

    if (pid < 0) {
        perror("fork");
        exit(1);
    } else if (pid == 0) {

        if (-1 == execvp(argv[1], argv + 1)) {
            perror("execvp");
            exit(1);
        }
    }

    wait(0);
    gettimeofday(&end, NULL);
    printf("Time in microseconds: %ldms\n", ((end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec)/1000));
}
