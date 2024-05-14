#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>

int main(int argc, char **argv) {
    int a2b[2], b2a[2];

    if (-1 == pipe(a2b) || -1 == pipe(b2a)) {
        perror("error on pipes");
        exit(1);
    }

    pid_t a = fork();

    srand(time(NULL));
    if (a < 0) {
        perror("fork a error");
        exit(1);
    } else if (a == 0) {
        close(a2b[0]), close(b2a[1]);
        int a_num = rand() % 151 + 50;
        int running = 1;


        while (running) {
            a_num = (a_num & 1) ? a_num + 1 : a_num;
            printf("A sends %d number\n", a_num);
            if (-1 == write(a2b[1], &a_num, sizeof(int))) {
                perror("write to b");
                exit(1);
            }

            if (a_num < 5) running = 0;

            if (-1 == read(b2a[0], &a_num, sizeof(int))) {
                perror("read from b");
                exit(1);
            }
            printf("A received %d number\n", a_num);

            if (a_num <5) running = 0;
        }

        close(a2b[1]), close(b2a[0]);
        exit(0);
    }

    pid_t b = fork();

    if (b < 0) {
        perror("fork b err");
        exit(1);
    } else if (b == 0) {
        close(b2a[0]), close(a2b[1]);
        int b_num;
        int running = 1;

        while (running) {
            if (-1 == read(a2b[0], &b_num, sizeof(int))) {
                perror("read from a");
                exit(1);
            }
            printf("B received %d number\n", b_num);

            b_num /= 2;

            if (-1 == write(b2a[1], &b_num, sizeof(int))) {
                perror("write to a");
                exit(1);
            }
            printf("B sent %d number\n", b_num);
            if (b_num < 5) running = 0;
        }

        close(a2b[0]), close(b2a[1]);
        exit(0);
    }

    close(a2b[0]), close(a2b[1]);
    close(b2a[0]), close(b2a[1]);

    wait(NULL);
    wait(NULL);
    return 0;
}
