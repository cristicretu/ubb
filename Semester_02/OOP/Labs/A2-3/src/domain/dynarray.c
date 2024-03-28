#include <stdlib.h>
#include <stdio.h>
#include "../../headers/domain/dynarray.h"
#include "../../headers/domain/domain.h"

DynArray *create_array(int capacity, DestroyElemFunction destroyElemFunction)
{
    DynArray *array = (DynArray *)malloc(sizeof(DynArray));
    if (!array)
    {
        return NULL;
    }
    array->capacity = capacity;
    array->length = 0;
    array->elems = (TElem *)malloc(capacity * sizeof(TElem));
    if (!array->elems)
    {
        free(array);
        return NULL;
    }
    array->destroyElemFunction = destroyElemFunction;
    return array;
}

void swap_elems(DynArray *array, int i, int j)
{
    if (i < 0 || i >= get_length(array) || j < 0 || j >= get_length(array))
    {
        return;
    }
    TElem *elems = get_elems(array);
    TElem temp = elems[i];
    elems[i] = elems[j];
    elems[j] = temp;
}

void resize_array(DynArray *array)
/*
Resizes the array if it's full.

:param array: DynArray* - the array to be resized
*/
{
    array->capacity *= 2;
    TElem *aux = realloc(array->elems, array->capacity * sizeof(TElem));
    if (aux == NULL)
    {
        return;
    }
    array->elems = aux;
}

DestroyElemFunction get_destroy_elem_function(DynArray *array)
{
    return array->destroyElemFunction;
}

void destroy_array(DynArray *array)
{
    if (!array)
    {
        return;
    }

    for (int i = 0; i < get_length(array); ++i)
    {
        if (array->elems[i] != NULL)
        {
            array->destroyElemFunction(array->elems[i]);
        }
    }
    free(array->elems);
    array->elems = NULL;
    free(array);
}

void add_elem(DynArray *array, TElem elem)
{
    int length = get_length(array);
    int capacity = get_capacity(array);
    TElem *elems = get_elems(array);

    if (length == capacity)
    {
        resize_array(array);
    }

    elems[length] = elem;
    array->length++;
}

TElem get_elem(DynArray *array, int index)
{
    if (array == NULL)
    {
        return NULL;
    }

    if (index < array->length)
    {
        return array->elems[index];
    }
    return NULL;
}

TElem *get_elems(DynArray *array)
{
    return array->elems;
}

int get_length(DynArray *array)
{
    if (array == NULL)
    {
        return -1;
    }
    return array->length;
}

int get_capacity(DynArray *array)
{
    if (array == NULL)
    {
        return -1;
    }
    return array->capacity;
}

void set_elem(DynArray *array, int index, TElem elem)
{
    if (array == NULL)
    {
        return;
    }

    if (index < 0 || index >= array->length)
    {
        return;
    }

    DestroyElemFunction destroyElemFunction = get_destroy_elem_function(array);
    TElem *elems = get_elems(array);
    destroyElemFunction(elems[index]);
    array->elems[index] = elem;
}

void set_length(DynArray *array, int length)
{
    array->length = length;
}

void remove_elem(DynArray *array, int index)
{
    int capacity = get_capacity(array);
    TElem *elems = get_elems(array);

    if (index < 0 || index >= get_length(array))
    {
        return;
    }

    DestroyElemFunction destroyElemFunction = get_destroy_elem_function(array);
    destroyElemFunction(elems[index]);
    for (int i = index; i < get_length(array) - 1; ++i)
    {
        elems[i] = elems[i + 1];
    }

    elems[get_length(array) - 1] = NULL;
    set_length(array, get_length(array) - 1);
}

DynArray *copy_array(DynArray *array)
{
    DynArray *copy = create_array(get_capacity(array), array->destroyElemFunction);
    for (int i = 0; i < get_length(array); ++i)
    {
        TElem *elem = get_elem(array, i);
        add_elem(copy, copyMedicine((Medicine *)elem));
    }
    return copy;
}
