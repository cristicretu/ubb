/*
a. Generate the first n prime numbers (n is a given natural number).
b. Given a vector of numbers, find the longest contiguous subsequence such that any two consecutive elements are relatively prime.
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

const int mxn = 2e6;

void print_menu()
{
    printf("1.generate the first N prime numbers\n");
    printf("2. find the longest contiguous subsequence such that any two consecutive elements are relatively prime\n");
    printf("0.Exit\n");
}

void read_natural_number(int *n)
{
    printf("Enter N (>0): ");
    scanf("%d", n);
    while (*n <= 0)
    {
        printf("The number must be positive\n");
        printf("Enter N (>0): ");
        scanf("%d", n);
    }
}

void read_vector(int *v, int n)
{
    for (int i = 0; i < n; ++i)
    {
        scanf("%d", &v[i]);
    }
}

void print_vector(int *v, int n)
{
    for (int i = 0; i < n; ++i)
    {
        printf("%d ", v[i]);
    }

    printf("\n");
}

bool relative_prime(int a, int b)
{
    if (!b)
    {
        return a == 1;
    }
    else
    {
        return relative_prime(b, a % b);
    }
}

void print_first_n_prime_numbers(int n)
{
    /*
    use the sieve of eratosthenes
    */
    int *ciur = (int *)malloc(mxn * sizeof(int));

    ciur[0] = ciur[1] = 1;
    for (int i = 4; i <= n; i += 2)
    {
        ciur[i] = 1;
    }

    for (int i = 3; i * i <= n; i += 2)
    {
        if (ciur[i] == 0)
        {
            for (int j = i * i; j <= n; j += i)
            {
                ciur[j] = 1;
            }
        }
    }

    for (int i = 1; i <= n; ++i)
    {
        if (ciur[i] == 0)
        {
            printf("%d ", i);
        }
    }
    printf("\n");

    free(ciur);
}

int find_longest_contiguous_subsequence(int *v, int n, int *start_index)
{

    *start_index = 0;

    int max_len = 1, curr_len = 1;
    int tmp = 0;
    for (int i = 1; i < n; ++i)
    {
        if (relative_prime(v[i], v[i - 1]))
        {
            curr_len++;
            if (curr_len > max_len)
            {
                max_len = curr_len;
                *start_index = tmp;
            }
        }
        else
        {
            curr_len = 1;
            tmp = i;
        }
    }
    return max_len;
}

int main()
{

    while (1)
    {
        print_menu();

        int option;
        printf("Choose an option: ");
        scanf("%d", &option);

        if (option == 1)
        {
            int n = 0;
            read_natural_number(&n);
            print_first_n_prime_numbers(n);
        }
        else if (option == 2)
        {
            int n = 0;
            read_natural_number(&n);
            int *v = (int *)malloc(n * sizeof(int));
            read_vector(v, n);

            // we need the start index and the length
            // we will get the length as the return of the function, and start index by ref
            int start_index = 0;
            int length = find_longest_contiguous_subsequence(v, n, &start_index);
            print_vector(v + start_index, length);
            free(v);
        }
        else if (option == 0)
        {
            printf("Exiting...\n");
            break;
        }
        else
        {
            printf("Invalid option\n");
        }
    }

    return 0;
}
