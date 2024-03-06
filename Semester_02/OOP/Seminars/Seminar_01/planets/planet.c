#include "planet.h"
#include <string.h>

char *get_planet_name(Planet *planet)
{
    return planet->name;
}

Planet create_planet(char *name, char *type, double distance)
{
    Planet planet;
    strcpy(planet.name, name);
    strcpy(planet.type, type);
    planet.distance = distance;
    return planet;
}