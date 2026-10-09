#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include "headers/ui/ui.h"
#include "headers/controller/service.h"
#include "headers/repository/repository.h"
#include "headers/repository/undo_repository.h"
#include "test_functions.h"

void run_all_tests()
{
    test_dynamic_array();
    test_medicine();
    test_service();
}

void test_dynamic_array()
{
    DynArray *array = create_array(1000, destroyMedicine);
    Medicine *medicine = constructMedicine("Paracetamol", 10.0f, 20.0f, 30.0f);
    add_elem(array, medicine);

    assert(get_length(array) == 1);

    Medicine *medicine2 = constructMedicine("Nurofen", 20.0f, 30.0f, 40.0f);
    add_elem(array, medicine2);

    Medicine *medicine3 = constructMedicine("Aspacardin", 30.0f, 40.0f, 50.0f);
    add_elem(array, medicine3);

    assert(get_length(array) == 3);

    remove_elem(array, 1);
    assert(get_length(array) == 2);

    destroy_array(array);

    printf("Dynamic array tests passed.\n");
}

void test_medicine()
{
    Medicine *medicine = constructMedicine("Paracetamol", 10.0f, 20.0f, 30.0f);
    assert(strcmp(getName(medicine), "Paracetamol") == 0);
    assert(getConcentration(medicine) == 10.0f);
    assert(getQuantity(medicine) == 20.0f);
    assert(getPrice(medicine) == 30.0f);
    destroyMedicine(medicine);

    // test copy
    Medicine *medicine2 = constructMedicine("Nurofen", 20.0f, 30.0f, 40.0f);
    Medicine *copy = copyMedicine(medicine2);
    assert(strcmp(getName(copy), "Nurofen") == 0);
    assert(getConcentration(copy) == 20.0f);
    assert(getQuantity(copy) == 30.0f);
    assert(getPrice(copy) == 40.0f);
    destroyMedicine(medicine2);
    destroyMedicine(copy);

    printf("Medicine tests passed.\n");
}

void test_service()
{
    Repository *repository = constructRepository();
    UndoRepository *undo_repository = construct_undo_repository(2);
    Service *service = constructService(repository, undo_repository, 2);
    assert(get_data(get_repository(service)) != NULL);
    assert(get_undo_operations(get_undo_repository(service)) != NULL);
    assert(get_redo_operations(get_undo_repository(service)) != NULL);

    generateNEntries(service, 10);
    assert(getRepoLength(get_repository(service)) == 10);

    Medicine *medicine = constructMedicine("Paracetamol", 101.0f, 20.0f, 30.0f);
    addMedicineService(service, medicine);
    assert(getRepoLength(get_repository(service)) == 11);

    Medicine *medicine2 = constructMedicine("Nurofen", 20.0f, 30.0f, 40.0f);
    addMedicineService(service, medicine2);
    assert(getRepoLength(get_repository(service)) == 12);

    Medicine *medicine3 = constructMedicine("Aspacardin", 30.0f, 40.0f, 50.0f);
    addMedicineService(service, medicine3);
    assert(getRepoLength(get_repository(service)) == 13);

    deleteMedicineService(service, medicine3);
    assert(getRepoLength(get_repository(service)) == 12);

    deleteMedicineService(service, medicine2);
    assert(getRepoLength(get_repository(service)) == 11);

    assert(undoRedoService(service, get_undo_operations(get_undo_repository(service)), get_redo_operations(get_undo_repository(service))) == 1);
    assert(getRepoLength(get_repository(service)) == 12);
    assert(undoRedoService(service, get_undo_operations(get_undo_repository(service)), get_redo_operations(get_undo_repository(service))) == 1);
    assert(undoRedoService(service, get_undo_operations(get_undo_repository(service)), get_redo_operations(get_undo_repository(service))) == 1);
    assert(getRepoLength(get_repository(service)) == 12);

    destroyService(service);

    printf("Service tests passed.\n");
}