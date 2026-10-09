#include <unistd.h>
#include <stdio.h>
#include <signal.h>

void handler(int sig) {
    printf("Signal %d received\n", sig);
}

int main(int argc, char *argv[]) {
    // signal()
    signal(SIGINT, handler);
    while(1) {
        usleep(1000 * 100);
    }
}
