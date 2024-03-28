#pragma once
#include "../domain/dyn_array.h"
#include "../repository/dog_type_repository.h"
#include "../service/service.h"

class UI {
 private:
  Service service;
  DogTypeRepository dogTypeRepository;
  typename DynArray<Dog>::Iterator dogIterator;
  bool iteratorSet = false;

 public:
  UI();
  ~UI();

  void run();
  void printAdminMenu();
  void printSelectModeMenu();

  void addDog();
  void removeDog();
  void updateDog();
  void listDogs();

  void printUserMenu();
  void printUserAdoptMenu();

  void adminUI(ushort command, ushort &mode);
  void userUI(ushort command, ushort &mode);
  void adoptUI(ushort command, ushort &mode);

  void userFilterDogs();
  void userGetAdoptedDogs();
};