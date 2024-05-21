#include "../../headers/tests/tests.h"

#include <assert.h>

#include "../../headers/domain/domain.h"
#include "../../headers/repository/dog_type_repository.h"
#include "../../headers/repository/repository.h"
#include "../../headers/service/service.h"

void test_domain() {
  Dog dog1("Breed1", "Name1", 1, "Photograph1");
  Dog dog2("Breed2", "Name2", 2, "Photograph2");

  assert(dog1.getBreed() == "Breed1");
  assert(dog1.getName() == "Name1");
  assert(dog1.getAge() == 1);

  dog2.setAge(3);
  assert(dog2.getAge() == 3);

  dog1.setBreed("Breed3");
  assert(dog1.getBreed() == "Breed3");

  dog2.setName("Name3");
  assert(dog2.getName() == "Name3");

  dog1.setPhotograph("Photograph3");
  assert(dog1.getPhotograph() == "Photograph3");

  dog1.print();

  std::cout << "Domain tests passed!" << std::endl;
}
void test_service() {
  Service service;

  service.addDog("Breed1", "Name1", 1, "Photograph1");
  service.addDog("Breed2", "Name2", 2, "Photograph2");

  int index = service.findDog("Breed1", "Name1", 1, "Photograph1");
  assert(index != -1);

  assert(service.getNumberOfDogs() == 2);

  try {
    service.addDog("Breed1", "Name1", 1, "Photograph1");
  } catch (std::invalid_argument &e) {
    assert(true);
  }

  try {
    service.removeDog(12567876);
  } catch (std::invalid_argument &e) {
    assert(true);
  }

  try {
    service.updateDog(12567876, "Breed1", "Name1", 1, "Photograph1");
  } catch (std::invalid_argument &e) {
    assert(true);
  }

  service.updateDog(0, "Breed3", "Name3", 3, "Photograph3");
  assert(service.getDog(0).getBreed() == "Breed3");
  assert(service.getDog(0).getName() == "Name3");
  assert(service.getDog(0).getAge() == 3);

  service.removeDog(0);
  assert(service.getNumberOfDogs() == 1);

  auto dog1 = service.getDogs()[0];
  assert(dog1.getBreed() == "Breed2");

  service.generateNRandomDogs(5);
  assert(service.getNumberOfDogs() == 6);

  service.adoptDog(0);
  service.adoptDog(1);

  try {
    service.adoptDog(500);
  } catch (std::invalid_argument &e) {
    assert(true);
  }

  assert(service.getAdoptedDogs().size() == 2);

  dog1 = service.getAdoptedDogs()[0];
  std::string breed = dog1.getBreed();
  int age = dog1.getAge();

  auto filteredDogs = service.filterDogs("", 0);
  assert(filteredDogs.size() == 4);

  service.addDog("Breed1", "Name1", 1, "Photograph1");
  filteredDogs = service.filterDogs("Breed1", 2);
  assert(filteredDogs.size() == 1);

  service.generateNRandomDogs(10);
}
void test_repository() {
  Repository repository;

  repository.addDog(Dog("Breed1", "Name1", 1, "Photograph1"));
  auto dog1 = repository.getDogs()[0];

  assert(dog1.getBreed() == "Breed1");
  assert(dog1.getName() == "Name1");

  repository.addDog(Dog("Breed2", "Name2", 2, "Photograph2"));

  assert(repository.getNumberOfDogs() == 2);

  repository.updateDog(0, Dog("Breed3", "Name3", 3, "Photograph3"));
  assert(repository.getDog(0).getBreed() == "Breed3");
  assert(repository.getDog(0).getName() == "Name3");
  assert(repository.getDog(0).getAge() == 3);

  repository.removeDog(0);
  assert(repository.getNumberOfDogs() == 1);

  std::cout << "Repository tests passed!" << std::endl;

  DogTypeRepository dogTypeRepository;

  auto result = dogTypeRepository.getDogImageUrl("labrador");
  assert(result.first.substr(0, 4) == "http");
  assert(result.first.find("labrador") != std::string::npos);
  assert(result.first.find("\\") == std::string::npos);
  assert(result.first.find("https://") == 0);

  result = dogTypeRepository.getDogImageUrl("does not exist");
  assert(result.first == "");
  assert(result.second == false);

  assert(dogTypeRepository.getNumberOfBreeds() > 0);
  int exists = dogTypeRepository.findBreed("labrador");
  int doesNotExist = dogTypeRepository.findBreed("does not exist");
  assert(doesNotExist == -1);
  assert(exists != -1);

  auto breed = dogTypeRepository.getBreed(exists);
  assert(breed == "labrador");

  auto pereche = dogTypeRepository.getDogImageUrl("labrador");

  assert(pereche.first.substr(0, 4) == "http");

  std::cout << "DogTypeRepository tests passed!" << std::endl;
}
void test_all() {
  test_domain();
  test_service();
  test_repository();
  test_admin();

  std::cout << "All tests passed!" << std::endl;
}
void test_admin() {
  Service service;

  service.addDog("Breed1", "Name1", 1, "Photograph1");
  service.addDog("Breed2", "Name2", 2, "Photograph2");

  int index = service.findDog("Breed1", "Name1", 1, "Photograph1");
  assert(index != -1);

  assert(service.getNumberOfDogs() == 2);

  service.updateDog(0, "Breed3", "Name3", 3, "Photograph3");
  assert(service.getDog(0).getBreed() == "Breed3");

  service.removeDog(0);

  assert(service.getNumberOfDogs() == 1);

  std::cout << "Admin tests passed!" << std::endl;
}