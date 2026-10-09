#pragma once

typedef void *TElem;
typedef void (*DestroyElemFunction)(TElem);

typedef struct
{
    TElem *elems;
    int length;
    int capacity;
    DestroyElemFunction destroyElemFunction;
} DynArray;

DynArray *create_array(int capacity, DestroyElemFunction destroyElemFunction);
void destroy_array(DynArray *array);
void add_elem(DynArray *array, TElem elem);
TElem get_elem(DynArray *array, int index);
int get_length(DynArray *array);
int get_capacity(DynArray *array);
TElem *get_elems(DynArray *array);
void set_elem(DynArray *array, int index, TElem elem);
void remove_elem(DynArray *array, int index);
int get_index(DynArray *array, TElem elem);
void set_length(DynArray *array, int length);
DynArray *copy_array(DynArray *array);
void swap_elems(DynArray *array, int i, int j);
