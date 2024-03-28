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

  std::cout << "Domain tests passed!" << std::endl;
}
void test_service() {
  Service service;

  service.addDog("Breed1", "Name1", 1, "Photograph1");
  service.addDog("Breed2", "Name2", 2, "Photograph2");

  int index = service.findDog("Breed1", "Name1", 1, "Photograph1");
  assert(index != -1);

  assert(service.getNumberOfDogs() == 2);

  service.updateDog(0, "Breed3", "Name3", 3, "Photograph3");
  assert(service.getDog(0).getBreed() == "Breed3");
  assert(service.getDog(0).getName() == "Name3");
  assert(service.getDog(0).getAge() == 3);

  service.removeDog(0);
  assert(service.getNumberOfDogs() == 1);

  service.generateNRandomDogs(5);
  assert(service.getNumberOfDogs() == 6);

  std::cout << "Service tests passed!" << std::endl;
}
void test_repository() {
  Repository repository;

  repository.addDog(Dog("Breed1", "Name1", 1, "Photograph1"));
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

  int exists = dogTypeRepository.findBreed("labrador");
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
void test_user() {}