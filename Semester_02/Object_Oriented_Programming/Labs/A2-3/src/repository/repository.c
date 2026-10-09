#include "../../headers/repository/repository.h"
#include "../../headers/domain/domain.h"
#include "../../headers/controller/service.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

Repository *constructRepository()
{
    Repository *repository = (Repository *)malloc(sizeof(Repository));
    if (!repository)
    {
        return NULL;
    }
    repository->data = create_array(1000, destroyMedicine);
    return repository;
}

DynArray *get_data(Repository *repository)
{
    return repository->data;
}

void destroyRepository(Repository *repository)
{
    if (!repository)
    {
        return;
    }
    destroy_array(get_data(repository));
    free(repository);
}
void addMedicineRepository(Repository *repository, Medicine *medicine)
{
    if (repository == NULL)
    {
        return;
    }
    add_elem(get_data(repository), medicine);
}
void updateMedicineRepository(Repository *repository, int index, Medicine *medicine)
{
    if (repository == NULL)
    {
        return;
    }

    set_elem(get_data(repository), index, medicine);
}
void deleteMedicineRepository(Repository *repository, int index)
{
    remove_elem(get_data(repository), index);
    return;
}
int doesMedicineExist(Repository *repository, Medicine *medicine)
{
    if (get_length(get_data(repository)) == 0)
    {
        return -1;
    }
    for (int i = 0; i < get_length(get_data(repository)); i++)
    {
        if (strcmp(getName(get_elem(get_data(repository), i)), getName(medicine)) == 0 && fabs(getConcentration(get_elem(get_data(repository), i)) - getConcentration(medicine)) < 0.1f)
        {
            return i;
        }
    }
    return -1;
}

int getRepoLength(Repository *repository)
{
    return get_length(get_data(repository));
}

Medicine *getMedicine(Repository *repository, char *name, float concentration)
{
    for (int i = 0; i < get_length(get_data(repository)); i++)
    {
        if (strcmp(getName(get_elem(get_data(repository), i)), name) == 0 && getConcentration(get_elem(get_data(repository), i)) == concentration)
        {
            return get_elem(get_data(repository), i);
        }
    }
    return NULL;
}

void sortMedicine(Repository *repository, int (*compare)(const Medicine *, const Medicine *), int order)
{
    DynArray *data = get_data(repository);
    for (int i = 0; i < get_length(data) - 1; ++i)
    {
        for (int j = i + 1; j < get_length(data); ++j)
        {
            Medicine *med1 = get_elem(data, i);
            Medicine *med2 = get_elem(data, j);

            int res = compare(med1, med2);

            if ((order == ASCENDING && res > 0) || (order == DESCENDING && res < 0))
            {
                swap_elems(data, i, j);
            }
        }
    }
}