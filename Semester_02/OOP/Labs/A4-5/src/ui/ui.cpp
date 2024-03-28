#include "../../headers/ui/ui.h"

#include <iomanip>
#include <iostream>

#include "../../headers/repository/dog_type_repository.h"
#include "../../headers/service/service.h"

UI::UI() {
  this->service = Service();
  this->dogTypeRepository = DogTypeRepository();
}

UI::~UI() {}

void UI::printSelectModeMenu() {
  std::cout << "||==========================||" << std::endl;
  std::cout << "1. Administrator mode" << std::endl;
  std::cout << "2. User mode" << std::endl;
  std::cout << "0. Exit" << std::endl;
  std::cout << "||==========================||" << std::endl;
  std::cout << std::endl;
}

void UI::printAdminMenu() {
  std::cout << "||==========ADMIN===========||" << std::endl;
  std::cout << "1. Add dog" << std::endl;
  std::cout << "2. Update dog" << std::endl;
  std::cout << "3. Remove dog" << std::endl;
  std::cout << "4. List dogs" << std::endl;
  std::cout << "0. Exit" << std::endl;
  std::cout << "||==========================||" << std::endl;
  std::cout << std::endl;
}

void UI::printUserMenu() {
  std::cout << "||==========USER============||" << std::endl;
  std::cout << "1. Adopt dog Menu" << std::endl;
  std::cout << "2. List dogs by breed, age" << std::endl;
  std::cout << "3. List adopted dogs" << std::endl;
  std::cout << "0. Exit" << std::endl;
  std::cout << "||==========================||" << std::endl;
  std::cout << std::endl;
}

void UI::printUserAdoptMenu() {
  std::cout << "||========ADOPT MENU=========||" << std::endl;
  std::cout << "1. ADOPT" << std::endl;
  std::cout << "2. SKIP DOG" << std::endl;
  std::cout << "0. Go back" << std::endl;
  std::cout << "||==========================||" << std::endl;
  std::cout << std::endl;
}

void UI::userFilterDogs() {
  int age;
  std::string breed;

  do {
    std::cout << "Breed: ";
    std::getline(std::cin, breed);

    if (breed.length() == 0) {
      break;
    }

    if (this->dogTypeRepository.findBreed(breed) == -1) {
      std::cout << "Invalid breed!" << std::endl;
      std::cout << "Here are few breeds: ";

      int rnd_index = rand() % this->dogTypeRepository.getNumberOfBreeds();

      for (int i = std::max(0, rnd_index);
           i <
           std::min(this->dogTypeRepository.getNumberOfBreeds(), rnd_index + 5);
           i++) {
        std::cout << this->dogTypeRepository.getBreed(i) << " ";
      }
      std::cout << std::endl;
    }
  } while (this->dogTypeRepository.findBreed(breed) == -1);

  do {
    std::cout << "Age: ";
    std::cin >> age;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid age!" << std::endl;
      continue;
    }

    if (age < 0) {
      std::cout << "Invalid age!" << std::endl;
    }
  } while (age < 0);

  std::cout << std::left << std::setw(3) << "ID"
            << " | " << std::setw(20) << "Name"
            << " | " << std::setw(15) << "Breed"
            << " | " << std::setw(3) << "Age"
            << " | " << std::setw(50) << "Photograph URL" << std::endl;

  std::cout << std::string(120, '-') << std::endl;

  auto dogs = this->service.filterDogs(breed, age);

  int n = dogs.getSize();

  if (n == 0) {
    std::cout << "No dogs found!" << std::endl;
  } else {
    for (int i = 0; i < dogs.getSize(); i++) {
      Dog dog = dogs[i];
      std::cout << std::left << std::setw(3) << i + 1 << " | " << std::setw(20)
                << dog.getName() << " | " << std::setw(15) << dog.getBreed()
                << " | " << std::setw(3) << dog.getAge() << " | "
                << std::setw(50) << dog.getPhotograph() << std::endl;
    }
  }
}

void UI::userGetAdoptedDogs() {
  std::cout << std::left << std::setw(3) << "ID"
            << " | " << std::setw(20) << "Name"
            << " | " << std::setw(15) << "Breed"
            << " | " << std::setw(3) << "Age"
            << " | " << std::setw(50) << "Photograph URL" << std::endl;

  std::cout << std::string(120, '-') << std::endl;

  auto dogs = this->service.getAdoptedDogs();

  int n = dogs.getSize();

  if (n == 0) {
    std::cout << "No dogs found!" << std::endl;
  } else {
    for (int i = 0; i < dogs.getSize(); i++) {
      Dog dog = dogs[i];
      std::cout << std::left << std::setw(3) << i + 1 << " | " << std::setw(20)
                << dog.getName() << " | " << std::setw(15) << dog.getBreed()
                << " | " << std::setw(3) << dog.getAge() << " | "
                << std::setw(50) << dog.getPhotograph() << std::endl;
    }
  }
}

void UI::addDog() {
  std::string breed, name, photograph;
  int age;

  do {
    std::cout << "Breed: ";
    std::getline(std::cin, breed);

    if (this->dogTypeRepository.findBreed(breed) == -1) {
      std::cout << "Invalid breed!" << std::endl;
      std::cout << "Here are few breeds: ";

      int rnd_index = rand() % this->dogTypeRepository.getNumberOfBreeds();

      for (int i = std::max(0, rnd_index);
           i <
           std::min(this->dogTypeRepository.getNumberOfBreeds(), rnd_index + 5);
           i++) {
        std::cout << this->dogTypeRepository.getBreed(i) << " ";
      }
      std::cout << std::endl;
    }
  } while (this->dogTypeRepository.findBreed(breed) == -1);

  do {
    std::cout << "Name: ";
    std::getline(std::cin, name);

    if (name.length() == 0) {
      std::cout << "Invalid name!" << std::endl;
    }
  } while (name.length() == 0);

  do {
    std::cout << "Age: ";
    std::cin >> age;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid age!" << std::endl;
      continue;
    }

    if (age < 0) {
      std::cout << "Invalid age!" << std::endl;
    }
  } while (age < 0);

  std::cout << "Slug for the photo (leave empty for auto. slug) := "
               "https://images.dog.ceo/breeds/"
            << breed << "/";
  std::getline(std::cin, photograph);

  if (photograph.length() == 0) {
    std::pair<std::string, bool> res =
        this->dogTypeRepository.getDogImageUrl(breed);

    if (res.second == false) {
      photograph =
          "https://images.dog.ceo/breeds/hound-basset/n02088238_10473.jpg";
    } else {
      photograph = res.first;
    }
  } else {
    photograph = "https://images.dog.ceo/breeds/" + breed + "/" + photograph;
  }

  try {
    this->service.addDog(breed, name, age, photograph);

    std::cout << "Dog added successfully!" << std::endl;
  } catch (std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
  }
}

void UI::removeDog() {
  std::string breed, name;

  std::cout << "Breed: ";
  std::getline(std::cin, breed);

  std::cout << "Name: ";
  std::getline(std::cin, name);

  std::cout << "Age: ";
  int age;
  std::cin >> age;
  std::cin.ignore();

  std::cout << "Photograph slug: ";
  std::string photograph;
  std::getline(std::cin, photograph);

  std::string breed_to_lowercase = "";

  for (int i = 0; i < breed.size(); i++) {
    breed_to_lowercase += std::tolower(breed[i]);
  }

  photograph =
      "https://images.dog.ceo/breeds/" + breed_to_lowercase + "/" + photograph;

  int index = this->service.findDog(breed, name, age, photograph);

  try {
    this->service.removeDog(index);

    std::cout << "Dog removed successfully!" << std::endl;
  } catch (std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
  }
}

void UI::updateDog() {
  std::string breed, name, photograph;
  int age;

  std::cout << "Breed: ";
  std::getline(std::cin, breed);

  std::cout << "Name: ";
  std::getline(std::cin, name);

  std::cout << "Age: ";
  std::cin >> age;
  std::cin.ignore();

  std::cout << "Photograph: ";
  std::getline(std::cin, photograph);

  std::string breed_to_lowercase = "";

  for (int i = 0; i < breed.size(); i++) {
    breed_to_lowercase += std::tolower(breed[i]);
  }

  photograph =
      "https://images.dog.ceo/breeds/" + breed_to_lowercase + "/" + photograph;

  int index = this->service.findDog(breed, name, age, photograph);

  if (index == -1) {
    std::cout << "Dog not found!" << std::endl;
    return;
  }

  std::string new_breed, new_name, new_photograph;
  int new_age;

  do {
    std::cout << "New breed: ";
    std::getline(std::cin, new_breed);

    if (new_breed.length() == 0) {
      break;
    }

    if (this->dogTypeRepository.findBreed(new_breed) == -1) {
      std::cout << "Invalid breed!" << std::endl;
      std::cout << "Here are few breeds: ";

      int rnd_index = rand() % this->dogTypeRepository.getNumberOfBreeds();

      for (int i = std::max(0, rnd_index);
           i <
           std::min(this->dogTypeRepository.getNumberOfBreeds(), rnd_index + 5);
           i++) {
        std::cout << this->dogTypeRepository.getBreed(i) << " ";
      }
      std::cout << std::endl;
    }
  } while (this->dogTypeRepository.findBreed(new_breed) == -1);

  std::cout << "New name: ";
  std::getline(std::cin, new_name);

  do {
    std::cout << "New age: ";
    std::cin >> new_age;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid age!" << std::endl;
      continue;
    }

    if (new_age < 0) {
      std::cout << "Invalid age!" << std::endl;
    }
  } while (new_age < 0);

  std::cin.ignore();
  std::cout << "New photograph: ";
  std::getline(std::cin, new_photograph);

  if (new_breed.length() == 0) {
    new_breed = breed;
  }

  if (new_name.length() == 0) {
    new_name = name;
  }

  if (new_age == 0) {
    new_age = age;
  }

  if (new_photograph.length() == 0) {
    new_photograph = photograph;
  }

  try {
    // this->service.updateDog(index, breed, name, age, photograph);
    this->service.updateDog(index, new_breed, new_name, new_age,
                            new_photograph);

    std::cout << "Dog updated successfully!" << std::endl;
  } catch (std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
  }
}

void UI::listDogs() {
  if (this->service.getNumberOfDogs() == 0) {
    std::cout << "No dogs in the database!" << std::endl;
    return;
  }

  std::cout << std::left << std::setw(3) << "ID"
            << " | " << std::setw(20) << "Name"
            << " | " << std::setw(15) << "Breed"
            << " | " << std::setw(3) << "Age"
            << " | " << std::setw(50) << "Photograph URL" << std::endl;
  std::cout << std::string(120, '-') << std::endl;

  for (int i = 0; i < this->service.getNumberOfDogs(); i++) {
    Dog dog = this->service.getDog(i);
    std::cout << std::left << std::setw(3) << i + 1 << " | " << std::setw(20)
              << dog.getName() << " | " << std::setw(15) << dog.getBreed()
              << " | " << std::setw(3) << dog.getAge() << " | " << std::setw(50)
              << dog.getPhotograph() << std::endl;
  }

  std::cout << std::endl;
}

void UI::adminUI(ushort command, ushort &mode) {
  this->printAdminMenu();

  do {
    std::cout << ">> Command: ";

    std::cin >> command;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid command!" << std::endl;
      continue;
    }

    if (command != 0 && command != 1 && command != 2 && command != 3 &&
        command != 4) {
      std::cout << "Invalid command!" << std::endl;
    }
  } while (command != 0 && command != 1 && command != 2 && command != 3 &&
           command != 4);

  if (command == 0) {
    mode = 0;
    return;
  }

  if (command == 1) {
    this->addDog();
  }

  if (command == 2) {
    this->updateDog();
  }

  if (command == 3) {
    this->removeDog();
  }

  if (command == 4) {
    this->listDogs();
  }
}

void UI::userUI(ushort command, ushort &mode) {
  this->printUserMenu();

  do {
    std::cout << ">> Command: ";

    std::cin >> command;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid command!" << std::endl;
      continue;
    }

    if (command != 0 && command != 1 && command != 2 && command != 3) {
      std::cout << "Invalid command!" << std::endl;
    }
  } while (command != 0 && command != 1 && command != 2 && command != 3);

  if (command == 0) {
    mode = 0;
    return;
  }

  if (command == 1) {
    mode = 3;
    return;
  } else if (command == 2) {
    this->userFilterDogs();
  } else if (command == 3) {
    this->userGetAdoptedDogs();
  }
}

void UI::adoptUI(ushort command, ushort &mode) {
  if (this->service.getNumberOfDogs() == 0) {
    std::cout << "No dogs in the database!" << std::endl;
    mode = 2;
    return;
  }

  if (!iteratorSet) {
    dogIterator = this->service.getDogs().begin();
    iteratorSet = true;
  }

  auto end = this->service.getDogs().end();

  do {
    this->printUserAdoptMenu();

    this->service.getDogs()[dogIterator.getIndex()].print();

    std::cout << ">> Command: ";
    std::cin >> command;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid command!" << std::endl;
      continue;
    }

    if (command != 0 && command != 1 && command != 2) {
      std::cout << "Invalid command!" << std::endl;
    }
  } while (command != 0 && command != 1 && command != 2);

  if (command == 0) {
    mode = 2;
    return;
  }

  if (command == 1) {
    this->service.adoptDog(dogIterator.getIndex());
    std::cout << "Dog adopted successfully!" << std::endl;
  } else if (command == 2) {
    std::cout << "Dog skipped!" << std::endl;
    dogIterator++;
  }

  if (dogIterator == end) {
    dogIterator = this->service.getDogs().begin();
  }
}
void UI::run() {
  this->service.generateNRandomDogs(10);
  this->printSelectModeMenu();

  ushort mode, command;
  do {
    std::cout << ">> Mode: ";
    std::cin >> mode;
    std::cin.ignore();
  } while (mode != 1 && mode != 2 && mode != 0);

  if (mode == 0) {
    std::cout << "Goodbye!" << std::endl;
    return;
  }

  while (true) {
    if (mode == 0) {
      break;
    } else if (mode == 1) {
      this->adminUI(command, mode);
    } else if (mode == 2) {
      this->userUI(command, mode);
    } else if (mode == 3) {
      this->adoptUI(command, mode);
    }
  }
}