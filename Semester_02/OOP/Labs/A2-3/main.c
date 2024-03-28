#include <stdio.h>
#include <stdlib.h>
#include "headers/ui/ui.h"
#include "headers/controller/service.h"
#include "headers/repository/repository.h"
#include "headers/repository/undo_repository.h"
#include "test_functions.h"

int main(int argc, char *argv[])
{
    run_all_tests();
    int mode = 2;
    Repository *repository = constructRepository();
    if (repository == NULL)
    {
        printf("Could not allocate memory for the repository\n");
        return 1;
    }
    UndoRepository *undo_repository = construct_undo_repository(mode);
    if (undo_repository == NULL)
    {
        printf("Could not allocate memory for the undo repository\n");
        free(repository);
        return 1;
    }
    Service *service = constructService(repository, undo_repository, mode);
    if (service == NULL)
    {
        printf("Could not allocate memory for the service\n");
        free(repository);
        free(undo_repository);
        return 1;
    }
    UI *ui = constructUI(service);
    if (ui == NULL)
    {
        printf("Could not allocate memory for the UI\n");
        free(repository);
        free(undo_repository);
        free(service);
        return 1;
    }
    runUI(ui);
    destroyUI(ui);
    return 0;
}