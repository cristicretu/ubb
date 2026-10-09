#include "../../headers/ui/ui.h"

#include <iomanip>
#include <iostream>

#include "../../headers/repository/csv_repository.h"
#include "../../headers/repository/db_repository.h"
#include "../../headers/repository/dog_type_repository.h"
#include "../../headers/repository/file_repository.h"
#include "../../headers/repository/html_repository.h"
#include "../../headers/service/service.h"

void UI::chooseDogRepo() {
  std::cout << "Choose the type of repository for DOGS: " << std::endl;
  std::cout << "1. TXT" << std::endl;
  std::cout << "2. SQL" << std::endl;
  std::cout << ">> Command: ";
  int dogsType;
  do {
    std::cin >> dogsType;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid command!" << std::endl;
      continue;
    }

    if (dogsType != 1 && dogsType != 2) {
      std::cout << "Invalid command!" << std::endl;
    }
  } while (dogsType != 1 && dogsType != 2);

  if (dogsType == 1) {
    this->service.setRepository(new FileRepository("../dogs.txt"));
  } else {
    this->service.setRepository(new DBRepository("../dogs.db"));
  }
}

void UI::chooseAdoptedDogRepo() {
  std::cout << "Choose the type of repository for ADOPTED DOGS: " << std::endl;
  std::cout << "1. CSV" << std::endl;
  std::cout << "2. HTML" << std::endl;
  std::cout << ">> Command: ";
  int adoptedType;
  do {
    std::cin >> adoptedType;
    std::cin.ignore();

    if (std::cin.fail()) {
      std::cin.clear();
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cout << "Invalid command!" << std::endl;
      continue;
    }

    if (adoptedType != 1 && adoptedType != 2) {
      std::cout << "Invalid command!" << std::endl;
    }
  } while (adoptedType != 1 && adoptedType != 2);

  if (adoptedType == 1) {
    this->adopedType = 1;
    this->service.setAdoptedRepository(new CSVRepository("../adopted.csv"));
  } else {
    this->adopedType = 2;
    this->service.setAdoptedRepository(new HTMLRepository("../adopted.html"));
  }
}

UI::UI() {
  this->service = Service();
  this->chooseDogRepo();
  this->chooseAdoptedDogRepo();
  this->dogTypeRepository = DogTypeRepository();
  this->iteratorSet = false;
}

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
  std::cout << "4. Open adopted file" << std::endl;
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

  /// Perform validations to make sure the breed exists
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

  // Perform validations to ensure age is valid
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

  /// Prepare the table header
  std::cout << std::left << std::setw(3) << "ID"
            << " | " << std::setw(20) << "Name"
            << " | " << std::setw(15) << "Breed"
            << " | " << std::setw(3) << "Age"
            << " | " << std::setw(50) << "Photograph URL" << std::endl;

  std::cout << std::string(120, '-') << std::endl;

  auto dogs = this->service.filterDogs(breed, age);

  int n = dogs.size();

  if (n == 0) {
    std::cout << "No dogs found!" << std::endl;
  } else {
    for (int i = 0; i < dogs.size(); i++) {
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

  int n = dogs.size();

  if (n == 0) {
    std::cout << "No dogs found!" << std::endl;
  } else {
    for (int i = 0; i < dogs.size(); i++) {
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
    mode = 3;
    return;
  } else if (command == 2) {
    this->userFilterDogs();
  } else if (command == 3) {
    this->userGetAdoptedDogs();
  } else if (command == 4) {
    std::string filename =
        this->adopedType == 1 ? "../adopted.csv" : "../adopted.html";
    this->service.openAdoptedFile(filename);
  }
}

void UI::adoptUI(ushort command, ushort &mode) {
  if (this->service.getNumberOfDogs() == 0) {
    std::cout << "No dogs in the database!" << std::endl;
    mode = 2;
    return;
  }

  for (Dog &dog : this->service.getDogs()) {
    dog.print();

    this->printUserAdoptMenu();

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

      if (command != 0 && command != 1 && command != 2) {
        std::cout << "Invalid command!" << std::endl;
      }
    } while (command != 0 && command != 1 && command != 2);

    if (command == 0) {
      mode = 2;
      return;
    }

    if (command == 1) {
      std::cout << "Dog adopted successfully!" << std::endl;

      int index = this->service.findDog(dog.getBreed(), dog.getName(),
                                        dog.getAge(), dog.getPhotograph());
      this->service.adoptDog(index);
    } else if (command == 2) {
      std::cout << "Dog skipped!" << std::endl;
    }

    if (this->service.getNumberOfDogs() == 0) {
      std::cout << "No dogs in the database!" << std::endl;
      mode = 2;
      return;
    }
  }
  // if (!this->iteratorSet) {
  //   this->it = this->service.getDogs().begin();
  //   this->iteratorSet = true;
  // }

  // auto end = this->service.getDogs().end();

  // do {
  //   this->it->print();
  //   this->printUserAdoptMenu();

  //   std::cout << ">> Command: ";
  //   std::cin >> command;
  //   std::cin.ignore();

  //   if (std::cin.fail()) {
  //     std::cin.clear();
  //     std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  //     std::cout << "Invalid command!" << std::endl;
  //     continue;
  //   }

  //   if (command != 0 && command != 1 && command != 2) {
  //     std::cout << "Invalid command!" << std::endl;
  //   }
  // } while (command != 0 && command != 1 && command != 2);

  // if (command == 0) {
  //   mode = 2;
  //   return;
  // }

  // if (command == 1) {
  //   std::cout << "Dog adopted successfully!" << std::endl;

  //   int index =
  //       this->service.findDog(this->it->getBreed(), this->it->getName(),
  //                             this->it->getAge(),
  //                             this->it->getPhotograph());
  //   // std::cout << "Index: " << index << std::endl;
  //   // this->service.adoptDog(index);
  // } else if (command == 2) {
  //   std::cout << "Dog skipped!" << std::endl;
  //   if (this->it == end) {
  //     this->it = this->service.getDogs().begin();
  //   }
  // }

  // if (this->it == this->service.getDogs().end()) {
  //   this->it = this->service.getDogs().begin();
  // } else {
  //   advance(this->it, 1);
  // }
}
void UI::run() {
  // this->service.generateNRandomDogs(50);
  // this->service.saveDogs("../dogs.txt", Repository::FileType::TXT);
  // this->service.loadDogs();
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