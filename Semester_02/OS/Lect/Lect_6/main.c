#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    int p2a[2], a2b[2], b2p[2], n;
    pipe(p2a); pipe(a2b); pipe(b2p);

    if (fork() == 0) { /// A
        close(p2a[1]); close(a2b[0]); close(b2p[0]); close(b2p[1]);

        while (1) {
            if (read(p2a[0], &n, sizeof(int)) < 0) break;
            if (n <= 0) break;
            n--;
            write(a2b[1], &n, sizeof(int));
        }

        close(p2a[0]); close(a2b[1]);
        exit(0);
    }

    if (fork() == 0) { /// B
        close(p2a[0]); close(p2a[1]); close(a2b[1]); close(b2p[0]);

        while (1) {
            if (read(a2b[1], &n, sizeof(int)) < 0) break;
            if (n <= 0) break;
            n--;
            write(b2p[1], &n, sizeof(int));
        }

        close(a2b[0]); close(b2p[1]);
        exit(0);
    }

    close(p2a[0]); close(a2b[1]); close(a2b[0]); close(b2p[1]);

    n = 7;
    write(p2a[1], &n, sizeof(int));

    while (1) {
        if (read(b2p[0], &n, sizeof(int)) < 0) break;
        if (n <= 0) break;
        n--;
        write(p2a[1], &n, sizeof(int));
    }

    close(p2a[1]); close(b2p[0]);
    wait(0); wait(0);

    return 0;
}
