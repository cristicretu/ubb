#pragma once

typedef struct
{
    char name[50];
    char type[50];
    double distance;
} Planet;

char *get_planet_name(Planet *planet);

Planet create_planet(char *name, char *type, double distance);