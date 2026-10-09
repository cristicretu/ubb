#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
    if (argc != 2) {
        perror("at least 1 argument pleas");
        exit(1);
    }

    int n = atoi(argv[1]);
    for (int i = 0; i < n; i++) {
        int pid = fork();
        if (pid < 0) {
            perror("fork");
            exit(1);
        }
        if (pid == 0) {
            printf("kid nr.%d with id = %d, my parent id is %d\n", i, getpid(), getppid());
            exit(0);
        }
    }

    for (int i = 0; i < n; ++i) {
        wait(NULL);
    }

    return 0;
}
