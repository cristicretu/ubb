#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

typedef struct {
    int nrDigits, nrChars, nrSpecials;
} Result;

void *calc(void *arg) {
    char *str = (char *) arg; 
    Result *result = malloc(sizeof(Result));

    result->nrDigits = 0;
    result->nrChars = 0;
    result->nrSpecials = 0;

    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] >= '0' && str[i] <= '9') {
            result->nrDigits++;
        } else if ((str[i] >= 'a' && str[i] <= 'z') || (str[i] >= 'A' && str[i] <= 'Z')) {
            result->nrChars++;
        } else {
            result->nrSpecials++;
        }
    }

    return result;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s arguments\n", argv[0]);
        return 1;
    }

    pthread_t *threads = (pthread_t *) malloc((argc - 1) * sizeof(pthread_t));
    Result *ans = (Result *) malloc(sizeof(Result));

    ans->nrDigits = 0;
    ans->nrChars = 0;
    ans->nrSpecials = 0;

    for (int i = 1; i < argc; i++) {
        pthread_create(&threads[i - 1], NULL, calc, argv[i]);
    }

    for (int i = 1; i < argc; i++) {
        Result *result;
        pthread_join(threads[i - 1], (void**) &result);

        ans->nrDigits += result->nrDigits;
        ans->nrChars += result->nrChars;
        ans->nrSpecials += result->nrSpecials;

        free(result);
    }

    printf("Digits: %d\n", ans->nrDigits);
    printf("Chars: %d\n", ans->nrChars);
    printf("Specials: %d\n", ans->nrSpecials);

    free(ans);
    free(threads);
    return 0;
}
