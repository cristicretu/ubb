#include <stdlib.h>
#include <stdio.h>
#include "../../headers/domain/func_operation.h"
#include "../../headers/domain/domain.h"

FuncOperation *construct_operation(int operation_type, Medicine *medicine1, Medicine *medicine2)
{
    FuncOperation *operation = (FuncOperation *)malloc(sizeof(FuncOperation));
    if (!operation)
    {
        return NULL;
    }
    operation->operation_type = operation_type;
    operation->medicine1 = constructMedicine(getName(medicine1), getConcentration(medicine1), getQuantity(medicine1), getPrice(medicine1));
    operation->medicine2 = NULL;
    if (medicine2 != NULL)
    {
        operation->medicine2 = constructMedicine(getName(medicine2), getConcentration(medicine2), getQuantity(medicine2), getPrice(medicine2));
    }
    return operation;
}
void destroy_operation(FuncOperation *operation)
{
    destroyMedicine(operation->medicine1);
    if (operation->medicine2 != NULL)
    {
        destroyMedicine(operation->medicine2);
    }
    free(operation);
}
int get_operation(FuncOperation *operation)
{
    return operation->operation_type;
}
Medicine *get_medicine1(FuncOperation *operation)
{
    return operation->medicine1;
}
Medicine *get_medicine2(FuncOperation *operation)
{
    return operation->medicine2;
}