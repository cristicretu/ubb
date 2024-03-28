#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc, char *argv[]) {
    struct timeval start, stop;
    gettimeofday(&start, NULL);
    int f = fork();

/*     if (-1 == f) { */
/*         perror("fork"); */
/*         exit(0); */
/*     } else if (0 == f) { */
/*         printf("child pid is %d\n", getpid()); */
/*     } else { */
/*         printf("parent pid is %d and the child id is %d\n", getpid(), f); */ 
/*     } */

    if (f == 0) {
        if (-1 == execvp(argv[1], argv + 1)) {
            perror("execvp");
            return 1;
        }
    } else if (f == -1) {
        perror("fork");
        return 1;
    }
    gettimeofday(&stop, NULL);

    printf("Time = %lf\n", (stop.tv_sec - start.tv_sec) + (stop.tv_usec - start.tv_usec) / 1000000.0);

    return 0;
}
