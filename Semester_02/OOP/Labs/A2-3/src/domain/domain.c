#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../../headers/domain/domain.h"

Medicine *constructMedicine(char *name, float concentration, int quantity, float price)
{
    Medicine *medicine = (Medicine *)malloc(sizeof(Medicine));
    if (!medicine)
    {
        return NULL;
    }

    medicine->name = (char *)malloc(strlen(name) + 1);
    if (!medicine->name)
    {
        free(medicine);
        return NULL;
    }

    strcpy(medicine->name, name);
    medicine->concentration = concentration;
    medicine->quantity = quantity;
    medicine->price = price;

    return medicine;
}

Medicine *copyMedicine(Medicine *medicine)
/*
Creates a deep copy of a medicine.

:param medicine: Medicine* - the medicine to be copied
:return: Medicine* - the copy of the medicine
*/
{
    return constructMedicine(medicine->name, medicine->concentration, medicine->quantity, medicine->price);
}

void destroyMedicine(Medicine *medicine)
/*
Deallocates the memory for a medicine.

:param medicine: Medicine* - the medicine to be deallocated
*/
{
    if (medicine == NULL)
    {
        return;
    }
    if (medicine != NULL)
    {
        free(medicine->name);
        free(medicine);
    }
}

char *getName(Medicine *medicine)
{
    return medicine->name;
}
float getConcentration(Medicine *medicine)
{
    return medicine->concentration;
}
int getQuantity(Medicine *medicine)
{
    return medicine->quantity;
}
float getPrice(Medicine *medicine)
{
    return medicine->price;
}

void setName(Medicine *medicine, char *name)
{
    if (medicine->name != NULL)
    {
        free(medicine->name);
    }

    medicine->name = (char *)malloc(strlen(name) + 1);
    if (!medicine->name)
    {
        return;
    }

    strcpy(medicine->name, name);
}

void setConcentration(Medicine *medicine, float concentration)
{
    medicine->concentration = concentration;
}

void setQuantity(Medicine *medicine, int quantity)
{
    medicine->quantity = quantity;
}

void setPrice(Medicine *medicine, float price)
{
    medicine->price = price;
}

int compareMedicine(Medicine *medicine1, Medicine *medicine2)
/*
Compares two medicines based on name and concentration.
Used for sorting.

:param medicine1: Medicine* - the first medicine
:param medicine2: Medicine* - the second medicine

:return: int - 0 if the medicines are equal, -1 if the first medicine is smaller, 1 if the first medicine is greater
*/
{
    int nameComparison = strcmp(getName(medicine1), getName(medicine2));
    if (nameComparison == 0)
    {
        float concentrationDifference = getConcentration(medicine1) - getConcentration(medicine2);
        if (concentrationDifference < 0)
        {
            return -1;
        }
        else if (concentrationDifference > 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    return nameComparison;
}