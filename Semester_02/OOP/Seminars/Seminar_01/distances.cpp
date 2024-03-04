#include "math.h"

double Manhattan(double x[10], double y[10], int n)
{
    double sum = 0;
    for (int i = 0; i < n; i++)
    {
        sum += absolute(diff(x[i], y[i]));
    }
    return sum;
}