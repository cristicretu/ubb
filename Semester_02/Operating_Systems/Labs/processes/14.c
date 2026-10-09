#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
    int a2b[2], b2a[2];

    if (-1 == pipe(a2b) || -1 == pipe(b2a)) {
        perror("pipe");
        return 1;
    }

    int a = 0, b = 0;

    pid_t pid = fork();

    if (-1 == pid) {
        perror("fork");
        return 1;
    } else if (0 == pid) {
        if (b == 0) {
            srand(getpid());
            b = rand() % 900 + 100;
            printf("B has generated %d\n", b);

            write(b2a[1], &b, sizeof(b));
        }

        while (1) {
            read(a2b[0], &a, sizeof(a));
            int diff = abs(a - b);
            printf("B received %d; difference is %d\n", a, diff);
            write(b2a[1], &diff, sizeof(diff));

            if (diff <= 100) {
             break;
            }
        }
    } else {
        srand(getpid());
        int diff = 0;
        int cnt_a = 0;

        read (b2a[0], &diff, sizeof(diff));

        while (diff > 100) {
            a = rand() % 900 + 100;

            write(a2b[1], &a, sizeof(a));
            cnt_a++;

            read(b2a[0], &diff, sizeof(diff));
        }
        
        printf("Process a has generated %d numbers\n", cnt_a);

    }

    return 0;
}
