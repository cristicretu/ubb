#include "../domain/dynarray.h"
#include "../domain/domain.h"

#pragma once

typedef struct
{
    DynArray *data;
} Repository;

Repository *constructRepository();
void destroyRepository(Repository *repository);
void addMedicineRepository(Repository *repository, Medicine *medicine);
void updateMedicineRepository(Repository *repository, int index, Medicine *medicine);
void deleteMedicineRepository(Repository *repository, int index);
int doesMedicineExist(Repository *repository, Medicine *medicine);
int getRepoLength(Repository *repository);
Medicine *getMedicine(Repository *repository, char *name, float concentration);
void sortMedicine(Repository *repository, int (*compare)(const Medicine *, const Medicine *), int order);
DynArray *get_data(Repository *repository);