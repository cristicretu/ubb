#include <stdbool.h>

#pragma once

typedef struct
{
    char *name;
    float concentration;
    int quantity;
    float price;
} Medicine;

Medicine *constructMedicine(char *name, float concentration, int quantity, float price);
void destroyMedicine(Medicine *medicine);
Medicine *copyMedicine(Medicine *medicine);
char *getName(Medicine *medicine);
float getConcentration(Medicine *medicine);
int getQuantity(Medicine *medicine);
float getPrice(Medicine *medicine);
int compareMedicine(Medicine *medicine1, Medicine *medicine2);
void setQuantity(Medicine *medicine, int quantity);
void setPrice(Medicine *medicine, float price);
void setName(Medicine *medicine, char *name);
void setConcentration(Medicine *medicine, float concentration);