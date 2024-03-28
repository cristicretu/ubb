#include "../../headers/ui/ui.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

UI *constructUI(Service *service)
{
    UI *ui = (UI *)malloc(sizeof(UI));
    ui->service = service;
    return ui;
}
void destroyUI(UI *ui)
{
    destroyService(ui->service);
    free(ui);
}

Service *getService(UI *ui)
{
    return ui->service;
}

void readStringOrEmpty(char *buffer, int bufferLength)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;
    // clear the input buffer

    if (fgets(buffer, bufferLength, stdin) != NULL)
    {
        if (buffer[0] == '\n')
        {
            buffer[0] = '\0'; // only enter was pressed
        }
        else
        {
            size_t len = strlen(buffer);
            if (len > 0 && buffer[len - 1] == '\n') // remove \n if present
            {
                buffer[len - 1] = '\0';
            }
        }
    }
    else
    {
        buffer[0] = '\0';
    }
}

bool readNumber(void *output, int type)
{
    char buffer[100];
    bool isValid = false;
    char *endptr;

    do
    {
        if (fgets(buffer, sizeof(buffer), stdin) != NULL)
        {
            if (buffer[0] == '\n' || strspn(buffer, " \n") == strlen(buffer))
            {
                if (type == INT_TYPE)
                {
                    *(int *)output = -1;
                }
                else if (type == FLOAT_TYPE)
                {
                    *(float *)output = -1;
                }
                return true;
            }

            buffer[strcspn(buffer, "\n")] = 0;

            if (type == INT_TYPE)
            {
                long int value = strtol(buffer, &endptr, 10);
                if (*endptr == '\0' && value > 0)
                {
                    *(int *)output = (int)value;
                    isValid = true;
                }
            }
            else if (type == FLOAT_TYPE)
            {
                float value = strtof(buffer, &endptr);
                if (*endptr == '\0' && value > 0)
                {
                    *(float *)output = value;
                    isValid = true;
                }
            }
        }
        else
        {
            return false;
        }

        if (!isValid)
        {
            printf("Invalid input. Please enter a valid number (>0) or leave empty.\n");
        }
    } while (!isValid);

    return true;
}

void addMedicineUI(UI *ui)
{
    char name[100];
    float concentration;
    int quantity;
    float price;

    //! TODO validate input!!!!!!!!!

    printf("Enter name: ");
    scanf("%99s", name);
    printf("Enter concentration: ");
    scanf("%f", &concentration);
    printf("Enter quantity: ");
    scanf("%d", &quantity);
    printf("Enter price: ");
    scanf("%f", &price);

    Medicine *medicine = constructMedicine(name, concentration, quantity, price);

    if (addMedicineService(getService(ui), medicine) == 1)
    {
        printf("Medicine exits, quantity increased\n");
        destroyMedicine(medicine);
    }
    else
    {
        printf("Medicine added\n");
    }
}
void deleteMedicineUI(UI *ui)
{
    char name[100];
    float concentration;
    printf("Enter name: ");
    scanf("%99s", name);
    printf("Enter concentration: ");
    scanf("%f", &concentration);
    Medicine *medicine = constructMedicine(name, concentration, 0, 0);
    if (deleteMedicineService(getService(ui), medicine) == 1)
    {
        printf("Medicine deleted\n");
    }
    else
    {
        printf("Medicine not found\n");
    }
    destroyMedicine(medicine);
}
void updateMedicineUI(UI *ui)
{
    char name[100];
    float concentration;
    char newName[100] = "";
    float newConcentration = -1;
    int newQuantity = -1;
    float newPrice = -1;

    printf("Enter name: ");
    scanf("%99s", name);
    printf("Enter concentration: ");
    scanf("%f", &concentration);

    Medicine *medicineToUpdate = constructMedicine(name, concentration, 0, 0);

    int index = doesMedicineExist(get_repository(getService(ui)), medicineToUpdate);

    if (index == -1)
    {
        printf("Medicine not found.\n");
        destroyMedicine(medicineToUpdate);
        return;
    }

    printf("New name (leave empty if no change): ");
    readStringOrEmpty(newName, sizeof(newName));

    printf("New concentration (leave empty if no change): ");
    readNumber(&newConcentration, FLOAT_TYPE);

    printf("New quantity (leave empty if no change): ");
    readNumber(&newQuantity, INT_TYPE);

    printf("New price (leave empty if no change): ");
    readNumber(&newPrice, FLOAT_TYPE);

    if (strlen(newName) == 0 && newConcentration == -1 && newQuantity == -1 && newPrice == -1)
    {
        printf("No changes made\n");
        destroyMedicine(medicineToUpdate);
        return;
    }

    if (newConcentration == -1)
    {
        newConcentration = concentration;
    }

    if (newQuantity == -1)
    {
        newQuantity = getQuantity(get_elem(get_data(get_repository(getService(ui))), index));
    }

    if (newPrice == -1)
    {
        newPrice = getPrice(get_elem(get_data(get_repository(getService(ui))), index));
    }

    if (strlen(newName) == 0)
    {
        strcpy(newName, getName(get_elem(get_data(get_repository(getService(ui))), index)));
    }

    Medicine *newMedicine = constructMedicine(newName, newConcentration, newQuantity, newPrice);

    int result = updateMedicineService(getService(ui), medicineToUpdate, newMedicine);

    if (result == 1)
    {
        printf("Medicine updated.\n");
    }
    else
    {
        printf("Medicine not found.\n");
    }
}

void undoUI(UI *ui)
{
    if (undoRedoService(getService(ui), get_undo_operations(get_undo_repository(getService(ui))), get_redo_operations(get_undo_repository(getService(ui)))) == 1)
    {
        printf("Undo successful\n");
    }
    else
    {
        printf("Undo failed\n");
    }
}
void redoUI(UI *ui)
{
    if (undoRedoService(getService(ui), get_redo_operations(get_undo_repository(getService(ui))), get_undo_operations(get_undo_repository(getService(ui)))) == 1)
    {
        printf("Redo successful\n");
    }
    else
    {
        printf("Redo failed\n");
    }
}

void filterMedicinesByStringUI(UI *ui)
{
    char filter[100];
    int mode = 0;
    printf("Enter filter (press enter if no filter): ");
    readStringOrEmpty(filter, sizeof(filter));

    if (strlen(filter) != 0)
    {
        printf("Enter mode (exact - DEFAULT (leave empty), 1 - fuzzy): ");
        readNumber(&mode, INT_TYPE);
    }

    if (mode == -1)
    {
        mode = EXACT;
    }

    if (mode != EXACT && mode != FUZZY)
    {
        printf("Invalid mode\n");
        return;
    }

    DynArray *filtered = filterMedicinesByString(getService(ui), filter, mode);

    if (get_length(filtered) == 0)
    {
        printf("No medicines found\n");
        free(filtered->elems);
        free(filtered);
        return;
    }

    printMedicines(filtered);

    free(filtered->elems);
    free(filtered);
}
void filterMedicinesByQuantityUI(UI *ui)
{
    int quantity = -1;
    printf("Enter quantity: ");

    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;
    readNumber(&quantity, INT_TYPE);

    int order = 0;
    printf("Enter order (1 - ascending, -1 - descending): ");
    scanf("%d", &order);

    if (order != 1 && order != -1)
    {
        printf("Invalid order\n");
        return;
    }

    DynArray *filtered = filterMedicinesByQuantity(getService(ui), quantity, order);
    if (get_length(filtered) == 0)
    {
        printf("No medicines found\n");
        free(filtered->elems);
        free(filtered);
        return;
    }

    printMedicines(filtered);

    free(filtered->elems);
    free(filtered);
}

void printMedicines(DynArray *medicines)
{
    printf("%-20s | %-8s | %-5s | %-6s\n", "Medicine", "Conc.", "Qty", "Price (lei)");
    printf("------------------------------------------------------\n");
    for (int i = 0; i < get_length(medicines); i++)
    {
        Medicine *medicine = get_elem(medicines, i);
        printf("%-20s | %-8.2f | %-5d | %-6.2f\n", getName(medicine), getConcentration(medicine), getQuantity(medicine), getPrice(medicine));
    }
}
void runUI(UI *ui)
{
    generateNEntries(getService(ui), 10);

    bool RUNNING = true;
    const int ADD_MEDICINE = 1,
              DELETE_MEDICINE = 2,
              UPDATE_MEDICINE = 3,
              FILTER_MEDICINES_BY_STRING = 4,
              FILTER_MEDICINES_BY_QUANTITY = 5,
              UNDO = 6,
              REDO = 7,
              EXIT = 0;

    while (RUNNING)
    {
        printMenu();
        int command = -1;
        printf("Enter command: ");
        scanf("%d", &command);
        switch (command)
        {
        case ADD_MEDICINE:
            addMedicineUI(ui);
            break;
        case DELETE_MEDICINE:
            deleteMedicineUI(ui);
            break;
        case UPDATE_MEDICINE:
            updateMedicineUI(ui);
            break;
        case FILTER_MEDICINES_BY_STRING:
            filterMedicinesByStringUI(ui);
            break;
        case FILTER_MEDICINES_BY_QUANTITY:
            filterMedicinesByQuantityUI(ui);
            break;
        case UNDO:
            undoUI(ui);
            break;
        case REDO:
            redoUI(ui);
            break;
        case EXIT:
            RUNNING = false;
            break;
        default:
            printf("Invalid command\n");
            break;
        }
    }
}
void printMenu()
{
    printf("\n");
    printf("|===============================|\n");
    printf("1. Add a medicine\n");
    printf("2. Delete a medicine\n");
    printf("3. Update a medicine\n");
    printf("|===============================|\n");
    printf("4. Filter medicines by string\n");
    printf("5. Filter medicines by quantity\n");
    printf("|===============================|\n");
    printf("6. Undo\n");
    printf("7. Redo\n");
    printf("|===============================|\n");
    printf("0. Exit\n");
    printf("|===============================|\n");
    printf("\n");
}