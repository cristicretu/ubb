#include "../repository/repository.h"
#include "../repository/undo_repository.h"
#include "../domain/domain.h"
#include "../domain/dynarray.h"

#pragma once

#define ASCENDING 1
#define DESCENDING -1

#define EXACT 0
#define FUZZY 1

typedef struct
{
    Repository *repository;
    UndoRepository *undo_repository;
    int design_pattern;
} Service;

Service *constructService(Repository *repository, UndoRepository *undo_repository, int design_pattern);
void destroyService(Service *service);
int addMedicineService(Service *service, Medicine *medicine);
int updateMedicineService(Service *service, Medicine *medicineToUpdate, Medicine *newMedicine);
int deleteMedicineService(Service *service, Medicine *medicine);
DynArray *filterMedicinesByString(Service *service, char *filter, int mode);
DynArray *filterMedicinesByQuantity(Service *service, int quantity, int order);
void generateNEntries(Service *service, int n);
bool checkFrequencies(char *name, char *filter);
int undoRedoService(Service *service, DynArray *undo_stack, DynArray *redo_stack);
UndoRepository *get_undo_repository(Service *service);
Repository *get_repository(Service *service);
int get_design_pattern(Service *service);
