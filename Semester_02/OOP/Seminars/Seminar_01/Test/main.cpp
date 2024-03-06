#include <stdio.h>
#include "distances.h"
#include "math.h"

int main()
{
    double a = -5.0;

    printf("absolute value of a is %f\n", absolute(a));

    double x[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    double y[10] = {10, 9, 8, 7, 6, 5, 4, 3, 2, 1};

    printf("Manhattan distance: %f\n", Manhattan(x, y, 10));
    return 0;
}
