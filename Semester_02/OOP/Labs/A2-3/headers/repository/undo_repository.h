#include "../domain/dynarray.h"
#include "../domain/func_operation.h"

#pragma once

typedef struct
{
    DynArray *undo_stack;
    DynArray *redo_stack;
} UndoRepository;

UndoRepository *construct_undo_repository(int design_pattern);
void destroy_undo_repository(UndoRepository *undo_repository);
DynArray *get_undo_stack(UndoRepository *undo_repository);
DynArray *get_redo_stack(UndoRepository *undo_repository);
int undo_stack_length(UndoRepository *undo_repository);
int redo_stack_length(UndoRepository *undo_repository);
void create_undo_array(UndoRepository *undo_repository, DynArray *array);
void create_redo_array(UndoRepository *undo_repository, DynArray *array);
void remove_undo_array(UndoRepository *undo_repository);
void remove_redo_array(UndoRepository *undo_repository);
void reset_redo_array(UndoRepository *undo_repository);

DynArray *get_undo_operations(UndoRepository *undo_repository);
DynArray *get_redo_operations(UndoRepository *undo_repository);
FuncOperation *get_undo_operation(UndoRepository *undo_repository);
FuncOperation *get_redo_operation(UndoRepository *undo_repository);
void reset_redo_operations(UndoRepository *undo_repository);
void push_undo_operation(UndoRepository *undo_repository, FuncOperation *operation);
void push_redo_operation(UndoRepository *undo_repository, FuncOperation *operation);
void pop_undo_operation(UndoRepository *undo_repository);
void pop_redo_operation(UndoRepository *undo_repository);