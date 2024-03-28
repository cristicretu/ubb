#include "../../headers/domain/func_operation.h"
#include "../../headers/repository/undo_repository.h"
#include "../../headers/domain/dynarray.h"
#include <stdlib.h>
#include <stdio.h>

UndoRepository *construct_undo_repository(int design_pattern)
{
    UndoRepository *undo_repository = (UndoRepository *)malloc(sizeof(UndoRepository));
    if (undo_repository == NULL)
    {
        return NULL;
    }

    undo_repository->undo_stack = NULL;
    undo_repository->redo_stack = NULL;

    if (design_pattern == 1)
    {
        undo_repository->undo_stack = create_array(1000, destroy_array);
        undo_repository->redo_stack = create_array(1000, destroy_array);
    }
    else if (design_pattern == 2)
    {
        undo_repository->undo_stack = create_array(1000, destroy_operation);
        undo_repository->redo_stack = create_array(1000, destroy_operation);
    }

    if (undo_repository->undo_stack == NULL || undo_repository->redo_stack == NULL)
    {
        free(undo_repository);
        return NULL;
    }
    return undo_repository;
}

DynArray *get_undo_operations(UndoRepository *undo_repository)
{
    return undo_repository->undo_stack;
}
DynArray *get_redo_operations(UndoRepository *undo_repository)
{
    return undo_repository->redo_stack;
}
FuncOperation *get_undo_operation(UndoRepository *undo_repository)
{
    return (FuncOperation *)get_elem(get_undo_stack(undo_repository), undo_stack_length(undo_repository) - 1);
}
FuncOperation *get_redo_operation(UndoRepository *undo_repository)
{
    return (FuncOperation *)get_elem(get_redo_stack(undo_repository), redo_stack_length(undo_repository) - 1);
}
void reset_redo_operations(UndoRepository *undo_repository)
{
    destroy_array(undo_repository->redo_stack);
    undo_repository->redo_stack = create_array(1000, destroy_operation);
}
void push_undo_operation(UndoRepository *undo_repository, FuncOperation *operation)
{
    add_elem(get_undo_stack(undo_repository), operation);
}
void push_redo_operation(UndoRepository *undo_repository, FuncOperation *operation)
{
    add_elem(get_redo_stack(undo_repository), operation);
}
void pop_undo_operation(UndoRepository *undo_repository)
{
    remove_elem(get_undo_stack(undo_repository), undo_stack_length(undo_repository) - 1);
}
void pop_redo_operation(UndoRepository *undo_repository)
{
    remove_elem(get_redo_stack(undo_repository), redo_stack_length(undo_repository) - 1);
}

void destroy_undo_repository(UndoRepository *undo_repository)
{
    destroy_array(undo_repository->undo_stack);
    destroy_array(undo_repository->redo_stack);
    free(undo_repository);
}

DynArray *get_undo_stack(UndoRepository *undo_repository)
{
    return undo_repository->undo_stack;
}

DynArray *get_redo_stack(UndoRepository *undo_repository)
{
    return undo_repository->redo_stack;
}

void create_undo_array(UndoRepository *undo_repository, DynArray *array)
{
    add_elem(get_undo_stack(undo_repository), array);
}

void create_redo_array(UndoRepository *undo_repository, DynArray *array)
{
    add_elem(get_redo_stack(undo_repository), array);
}

void remove_undo_array(UndoRepository *undo_repository)
{
    remove_elem(get_undo_stack(undo_repository), undo_stack_length(undo_repository) - 1);
}

void remove_redo_array(UndoRepository *undo_repository)
{
    remove_elem(get_redo_stack(undo_repository), redo_stack_length(undo_repository) - 1);
}

void reset_redo_array(UndoRepository *undo_repository)
{
    destroy_array(undo_repository->redo_stack);
    undo_repository->redo_stack = create_array(1000, destroy_array);
}

int undo_stack_length(UndoRepository *undo_repository)
{
    return undo_repository->undo_stack->length;
}

int redo_stack_length(UndoRepository *undo_repository)
{
    return undo_repository->redo_stack->length;
}
