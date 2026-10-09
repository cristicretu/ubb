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
        if (fork() == 0) {
            printf("i am child %d, my id is %d, my parent id is %d\n", i, getpid(), getppid());
        } else {
            wait(NULL);
            break;
        }
    }

    return 0;
}
