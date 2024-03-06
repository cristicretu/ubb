#include <stdio.h>
#include "planet.h"

int main()
{
    Planet x;

    x = create_planet("Earth", "Terrestrial", 1.0);

    printf("Planet %s is a %s planet at a distance of %f AU\n", get_planet_name(&x), x.type, x.distance);

    return 0;
}