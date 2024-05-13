#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>

pid_t c;

void p(int sig) {
    printf("Parnet terminating\n");
    kill(c, SIGUSR1);
    wait(0);
    exit(0);
}

void child(int sig){
    printf("child finioshing\n");
    exit(0);
}

void z(int sig) {
    printf("Parent waiting for child\n");
    wait(0);
}

int main(int argc, char ** argv) {
    c = fork();

    if (c < 0) {
        perror("on fork");
        exit(1);
    } else if (c == 0) {
        signal(SIGUSR1, child);

        while(1) {
            printf("Child working\n");
            sleep(3);
        }

        exit(0);
    } else {
        signal(SIGUSR1, p);
        signal(SIGCHLD, z);

        while(1) {
            printf("Parent working\n");
            sleep(2);
        }
    }
}

