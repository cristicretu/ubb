#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main() {
    int pid;

    printf("before\n");
    pid = fork();
    if (pid == 0) {
        printf("child\n");
        exit(0);
    }  
    printf("parent\n");
    wait(0);
    return 0;
}
