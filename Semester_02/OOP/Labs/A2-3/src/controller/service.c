#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include "../../headers/controller/service.h"
#include "../../headers/repository/repository.h"
#include "../../headers/domain/domain.h"

Service *constructService(Repository *repository, UndoRepository *undo_repository, int design_pattern)
/*
Constructs a service, using the given repository and undo_repository

:param repository: the repository
:param undo_repository: the undo_repository
:param design_pattern: the design pattern
*/
{
    Service *service = (Service *)malloc(sizeof(Service));
    if (service == NULL)
    {
        return NULL;
    }
    service->repository = repository;
    service->undo_repository = undo_repository;
    service->design_pattern = design_pattern;
    return service;
}

UndoRepository *get_undo_repository(Service *service)
{
    return service->undo_repository;
}

Repository *get_repository(Service *service)
{
    return service->repository;
}

int get_design_pattern(Service *service)
{
    return service->design_pattern;
}

void destroyService(Service *service)
/*
Destroys the service

:param service: the service to be destroyed
*/
{
    destroyRepository(get_repository(service));
    destroy_undo_repository(get_undo_repository(service));
    free(service);
}
int addMedicineService(Service *service, Medicine *medicine)
/*
Adds the medicine to the repository, if it already exists, it will increase the quantity of the existing medicine

:param service: the service
:param medicine: the medicine to be added
:return: 0 if the medicine was added, 1 if the quantity was increased
*/
{
    int index = doesMedicineExist(get_repository(service), medicine);
    if (index != -1)
    {
        Medicine *existingMedicine = getMedicine(get_repository(service), getName(medicine), getConcentration(medicine));
        setQuantity(existingMedicine, getQuantity(existingMedicine) + getQuantity(medicine));
        return 1; // destroy the created medicine, one axists
    }
    if (get_design_pattern(service) == 1)
    {
        reset_redo_array(get_undo_repository(service));
        DynArray *copy = copy_array(get_data(get_repository(service)));
        create_undo_array(get_undo_repository(service), copy);
    }
    else
    {
        reset_redo_operations(get_undo_repository(service));
        FuncOperation *operation = construct_operation(DELETE, medicine, NULL);
        push_undo_operation(get_undo_repository(service), operation);
    }

    addMedicineRepository(get_repository(service), medicine);
    return 0; // all good
}
// int updateMedicineService(Service *service, Medicine *medicineToUpdate, char *newName, float newConcentration, int newQuantity, float newPrice)
int updateMedicineService(Service *service, Medicine *medicineToUpdate, Medicine *newMedicine)
/*
Updates the medicine in the repository, if it exists and only if the new values are different than the old ones

:param service: the service
:param medicineToUpdate: the medicine to be updated
:param newName: the new name
:param newConcentration: the new concentration
:param newQuantity: the new quantity
:param newPrice: the new price
:return: 1 if the medicine was updated, 0 if the medicine does not exist
*/
{
    int index = doesMedicineExist(get_repository(service), medicineToUpdate);
    destroyMedicine(medicineToUpdate);
    if (index == -1)
    {
        destroyMedicine(newMedicine);
        return 0;
    }

    if (get_design_pattern(service) == 1)
    {
        reset_redo_array(get_undo_repository(service));
        DynArray *copy = copy_array(get_data(get_repository(service)));
        create_undo_array(get_undo_repository(service), copy);
    }
    else if (get_design_pattern(service) == 2)
    {
        reset_redo_operations(get_undo_repository(service));
        Medicine *existingMedicine = get_elem(get_data(get_repository(service)), index);
        FuncOperation *operation = construct_operation(UPDATE, newMedicine, existingMedicine);
        push_undo_operation(get_undo_repository(service), operation);
    }

    updateMedicineRepository(get_repository(service), index, newMedicine);

    return 1;
}
int deleteMedicineService(Service *service, Medicine *medicine)
/*
Deletes the medicine from the repository, if it exists

:param service: the service
:param medicine: the medicine to be deleted

:return: 1 if the medicine was deleted, 0 if the medicine does not exist
*/
{
    int index = doesMedicineExist(get_repository(service), medicine);
    if (index != -1)
    {

        if (get_design_pattern(service) == 1)
        {
            reset_redo_array(get_undo_repository(service));
            DynArray *copy = copy_array(get_data(get_repository(service)));
            create_undo_array(get_undo_repository(service), copy);
        }
        else
        {
            reset_redo_operations(get_undo_repository(service));
            FuncOperation *operation = construct_operation(ADD, medicine, NULL);
            push_undo_operation(get_undo_repository(service), operation);
        }

        deleteMedicineRepository(get_repository(service), index);
        return 1;
    }
    return 0;
}
DynArray *filterMedicinesByString(Service *service, char *filter, int mode)
/*
Filters the medicines by the given string, sorting them in ascending order
2 modes: EXACT, FUZZY
EXACT: the name of the medicine must be exactly the same as the filter
FUZZY: the chars of the filter must be in the name of the medicine

:param service: the service
:param filter: the filter
:param mode: the mode

:return: the filtered medicines
*/
{
    DynArray *filtered = create_array(get_capacity(get_data(get_repository(service))), destroyMedicine);

    sortMedicine(get_repository(service), compareMedicine, ASCENDING);

    for (int i = 0; i < getRepoLength(get_repository(service)); ++i)
    {
        Medicine *medicine = get_elem(get_data(get_repository(service)), i);
        bool shouldAdd = false;

        if (mode == EXACT)
        {
            if (strcasecmp(getName(medicine), filter) == 0 || strcmp(filter, "") == 0)
            {
                shouldAdd = true;
            }
        }
        else if (mode == FUZZY)
        {
            if (checkFrequencies(getName(medicine), filter) || strcmp(filter, "") == 0)
            {
                shouldAdd = true;
            }
        }

        if (shouldAdd)
        {
            add_elem(filtered, medicine);
        }
    }
    return filtered;
}

bool checkFrequencies(char *name, char *filter)
/*
Used for the FUZZY mode, checks if the chars of the filter are in the name

:param name: the name
:param filter: the filter

:return: true if the chars of the filter are in the name
*/
{
    int nameLength = strlen(name);
    int filterLength = strlen(filter);

    int checker = 1;

    for (int i = 0; i < filterLength; ++i)
    {
        for (int j = 0; j < nameLength; ++j)
        {
            if (name[j] == filter[i])
            {
                checker++;
                break;
            }
        }
    }

    return checker == filterLength;
}

DynArray *filterMedicinesByQuantity(Service *service, int quantity, int order)
/*
Filters the medicines by the given quantity, sorting them in the given order

:param service: the service
:param quantity: the quantity
:param order: the order

:return: the filtered medicines
*/
{
    DynArray *filtered = create_array(get_capacity(get_data(get_repository(service))), destroyMedicine);

    sortMedicine(get_repository(service), compareMedicine, order);

    for (int i = 0; i < getRepoLength(get_repository(service)); ++i)
    {
        Medicine *medicine = get_elem(get_data(get_repository(service)), i);
        if (getQuantity(medicine) < quantity || quantity == -1)

        {
            add_elem(filtered, medicine);
        }
    }
    return filtered;
}
void generateNEntries(Service *service, int n)
/*
Generates n random entries in the repository

:param service: the service
:param n: the number of entries
*/
{
    srand(time(NULL));

    char names[][30] = {
        "Nurofen",
        "Aspacardin",
        "Quixx",
        "Hyabak",
        "Aspenter",
        "Paracetamol",
        "AntinevralgicForte",
        "Kavit",
        "Extraterestrii",
        "Xyfia"};

    for (int i = 0; i < 10; i++)
    {
        float concentration = (float)(rand() % 100) / 10 + 0.1;
        int quantity = rand() % 99 + 1;
        float price = (float)(rand() % 1000) / 10 + 0.1;
        Medicine *medicine = constructMedicine(names[i], concentration, quantity, price);
        addMedicineService(service, medicine);
    }
}

int undoRedoService(Service *service, DynArray *undo_stack, DynArray *redo_stack)
/*
Performs an undo on the service, based on the order of the operations
If undo, redo => it undoes
If redo, undo => it redoes

:param service: the service
:param undo_stack: the undo stack
:param redo_stack: the redo stack

:return: 1 if the undo/redo was performed, 0 otherwise
*/
{
    if (get_length(undo_stack) == 0)
    {
        return 0;
    }

    if (service->design_pattern == 1)
    {
        DynArray *last_operation = (DynArray *)get_elem(undo_stack, get_length(undo_stack) - 1);
        DynArray *copy = copy_array(last_operation);
        add_elem(redo_stack, copy);

        get_repository(service)->data = copy_array(last_operation);
        remove_elem(undo_stack, get_length(undo_stack) - 1);

        return 1;
    }
    else if (service->design_pattern == 2)
    {
        FuncOperation *last_operation = (FuncOperation *)get_elem(undo_stack, get_length(undo_stack) - 1);
        FuncOperation *copy = NULL;

        if (last_operation->operation_type == ADD)
        {
            addMedicineRepository(get_repository(service), copyMedicine(get_medicine1(last_operation)));
            copy = construct_operation(DELETE, get_medicine1(last_operation), NULL);
        }
        else if (last_operation->operation_type == DELETE)
        {
            deleteMedicineRepository(get_repository(service), doesMedicineExist(get_repository(service), get_medicine1(last_operation)));
            copy = construct_operation(ADD, get_medicine1(last_operation), NULL);
        }
        else if (last_operation->operation_type == UPDATE)
        {
            updateMedicineRepository(get_repository(service), doesMedicineExist(get_repository(service), get_medicine1(last_operation)), copyMedicine(get_medicine2(last_operation)));
            copy = construct_operation(UPDATE, get_medicine2(last_operation), get_medicine1(last_operation));
        }

        add_elem(redo_stack, copy);
        remove_elem(undo_stack, get_length(undo_stack) - 1);
        return 1;
    }

    return 0;
}