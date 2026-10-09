#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char **argv) {
    int a2b[2], b2c[2], c2a[2];

    if (0 > pipe(a2b) || 0 > pipe(b2c) || 0 > pipe(c2a)) {
        perror("error on pipe");
        exit(1);
    }

    pid_t b = fork();
    if (b < 0) {
        perror("b errr");
        exit(1);
    } else if (b == 0) {
        close(b2c[0]);
        close(a2b[1]);
        int len;
        if (-1 == read(a2b[0], &len, sizeof(int))) {
            perror("on read b len");
            exit(1);
        }

        int *arr = (int *)malloc(len * sizeof(int));

        for (int i = 0; i < len; ++i) {
            if (-1 == read(a2b[0], &arr[i], sizeof(int))) {
                perror("read from a");
                exit(1);
            }

            printf("B got: %d\n", arr[i]);

            arr[i] += random() % 4 + 2;
        }

        if (-1 == write(b2c[1], &len, sizeof(int))) {
            perror("write len to c");
            exit(1);
        }

        for (int i = 0; i < len; ++i) {
            printf("sending %d to c\n", arr[i]);

            if (-1 == write(b2c[1], &arr[i], sizeof(int))) {
                perror("error sending to c");
                exit(1);
            }
        }

        free(arr);
        close(b2c[1]);
        exit(0);
    }

    pid_t c = fork();

    if (c < 0) {
        perror("god bless");
        exit(1);
    } else if (c == 0) {
        int len;
           if (-1 == read(b2c[0], &len, sizeof(int))) {
               perror("on read c len");
               exit(1);
           }
   
           int *arr = (int *)malloc(len * sizeof(int));
           int sum = 0;
   
           for (int i = 0; i < len; ++i) {
               if (-1 == read(b2c[0], &arr[i], sizeof(int))) {
                   perror("read from a");
                   exit(1);
               }
   
               printf("C got: %d\n", arr[i]);
   
               sum += arr[i];
           }
   
           if (-1 == write(c2a[1], &sum, sizeof(int))) {
               perror("sium");
               exit(1);
           }
   
           free(arr);
           exit(0);
    }

    int n;
    printf("n is ");
    scanf("%d", &n);
    int *arr = (int *)malloc(n * sizeof(int));

    for (int i = 0; i < n; ++i) {
        printf("a[%d] = ", i);
        scanf("%d", &arr[i]);
    }

    if (-1 == write(a2b[1], &n, sizeof(int))) {
        perror("a");
        exit(1);
    }

    for (int i = 0; i < n; ++i) {
        printf("A is sending to B: %d\n", arr[i]);

        if (-1 == write(a2b[1], &arr[i], sizeof(int))) {
            perror("wrror write a->b\n");
            exit(1);
        }
    }

    int sum;
    if (-1 == read(c2a[0], &sum, sizeof(int))) {
        perror("ok");
        exit(1);
    }

    printf("Sum is %d", sum);

    wait(0);
    wait(0);
    free(arr);
    return 0;
}
