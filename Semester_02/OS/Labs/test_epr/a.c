/*
 * Create two C processes (we will refer to them as A and B) that communicate via pipe. Process A keeps reading integers from standard input and sends them to process B. Process B determines the list of multiples of 7 that are smaller than the received number for each number, immediately after reading it from the pipe, and sends the list of multiples of 7 to process A. Process A prints the number of multiples of 7 and the multiples of 7 for each number as received from B. Both processes terminate after 0 is provided as input.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char **argv) {
    int a2b[2], b2a[2];

    if (-1 == pipe(a2b)) {
        perror("on pipe creatin a");
        exit(1);
    }

    if (-1 == pipe(b2a)) {
        perror("on pipe creatin b");
        exit(1);
    }

    int a = fork();
    if (a < 0) {
        perror("on fork creating a");
        exit(1);
    } else if (a == 0) {
        close(a2b[0]);
        close(b2a[1]); 
        
        while (1) {
            int n;
            scanf("%d", &n);

            int should_stop = (n == 0);
            if (-1 == write(a2b[1], &n, sizeof(int))) {
                perror("on write a 2 b");
                exit(1);
            }

            if (should_stop) {
                break;
            }
            
            int len;
            if (-1 == read(b2a[0], &len, sizeof(int))) {
                perror("on read the len from b");
                exit(1);
            }

            printf("Length of multiples of 7 is %d\n", len);

            int *arr = (int *)malloc(len * sizeof(int));
            if (-1 == read(b2a[0], arr, len * sizeof(int))) {
                perror("on read the array from b");
                exit(1);
            }
            
            if (len == 0) {
                free(arr);
                continue;
            }

            for (int i = 0; i < len; ++i) {
                printf("%d ", arr[i]);
            }
            printf("\n");

            free(arr);
        }
        
        close(a2b[1]);
        close(b2a[0]);
        exit(0);
    }

    int b = fork();
    if (b < 0) {
        perror("on fork creation b");
        exit(1);
    } else if (b == 0) {
        close(a2b[1]);
        close(b2a[0]);
        while(1) {
            int n;
            if (-1 == read(a2b[0], &n, sizeof(int))) {
                perror("on read from a into b");
                exit(1);
            }

            if (n == 0) {
                break;
            }

            int len = 0, x = 7; // always remember to initialize variables kids!
            while (x < n) {
                ++len;
                x += 7;
            }
           // printf("the length for number %d is %d\n", n, len);
            int old_len = len;

            int *arr = (int *)malloc(len * sizeof(int));

            if (-1 == write(b2a[1], &len, sizeof(int))) {

                perror("on write len to a");
                exit(1);
            }

            for (int i = 0; i < old_len; ++i) {
                arr[i] = 7 * (i + 1);
            }

            if (-1 == write(b2a[1], arr, old_len* sizeof(int))) {
                perror("on write array to a");
                exit(1);
            }

            free(arr);
        }

        close(a2b[0]);
        close(b2a[1]);
        exit(0);
    }


    wait(NULL);
    wait(NULL);

    close(a2b[0]), close(a2b[1]);
    close(b2a[0]), close(b2a[1]);
    return 0;
}

