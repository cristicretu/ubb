#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

// one parent, two children, each children generates a random number, they give back the number to the parent
// the parent checks which number is bigger, and tells the children 1 if the number is bigger, 0 if equel, -1 if lower
// and the child prints its number and the the result of the comparison 
int main(int argc, char *argv[]) {
    int rna[2], rnb[2];
    pipe(rna);
    pipe(rnb);

    int child1 = fork();
    if (child1 == 0) {
        close(rna[0]);
        close(rnb[0]);
        close(rnb[1]);

        srand(time(NULL));

        int a = rand() % 100;
        write(rna[1], &a, sizeof(int));

        int result;
        read(rna[0], &result, sizeof(int));

        printf("Child 1: %d, %d\n", a, result);
        close(rna[1]);
        exit(0);
    }

    int child2 = fork();

    if (child2 == 0) {
        close(rna[0]);
        close(rna[1]);
        close(rnb[0]);

        int b = rand() % 100;
        write(rnb[1], &b, sizeof(int));

        srand(time(NULL));

        int result;
        read(rnb[0], &result, sizeof(int));

        printf("Child 2: %d, %d\n", b, result);
        close(rnb[1]);
        exit(0);
    }

    close(rna[1]);
    close(rnb[1]);

    close(rna[0]);
    close(rnb[0]);
    close(rna[1]);
    close(rnb[1]);

    wait(NULL);
    wait(NULL);

    return 0;
}
