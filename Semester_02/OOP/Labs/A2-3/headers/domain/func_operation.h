#include "domain.h"

#pragma once

#define ADD 1
#define DELETE 2
#define UPDATE 3

typedef struct
{
    int operation_type;
    Medicine *medicine1, *medicine2;
} FuncOperation;

FuncOperation *construct_operation(int operation_type, Medicine *medicine1, Medicine *medicine2);
void destroy_operation(FuncOperation *operation);
int get_operation(FuncOperation *operation);
Medicine *get_medicine1(FuncOperation *operation);
Medicine *get_medicine2(FuncOperation *operation);