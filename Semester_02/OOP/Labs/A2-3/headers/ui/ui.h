#include "../controller/service.h"
#include "../domain/dynarray.h"

#pragma once

#define INT_TYPE 0
#define FLOAT_TYPE 1

typedef struct
{
    Service *service;
} UI;

UI *constructUI(Service *service);
void destroyUI(UI *ui);
void runUI(UI *ui);
void printMenu();
void addMedicineUI(UI *ui);
void deleteMedicineUI(UI *ui);
void updateMedicineUI(UI *ui);
void filterMedicinesByStringUI(UI *ui);
void filterMedicinesByQuantityUI(UI *ui);
void undoUI(UI *ui);
void redoUI(UI *ui);
void readStringOrEmpty(char *buffer, int bufferLength);
void printMedicines(DynArray *medicines);
bool readNumber(void *output, int type);