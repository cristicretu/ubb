# Assignment 04-05

## Requirements

- [x] You will be given one of the problems below to solve.
- [x] The application must be implemented in C++ and use layered architecture.
- [x] Provide tests and specifications for non-trivial functions outside the UI. Test coverage must be at least 98% for all layers, except the UI.
- [x] Have at least 10 entities in your memory repository.
- [x] Validate the input data.
- Handle the following situations:
  - [x] If an entity that already exists is added, a message is shown and the entity is not stored. You must decide what makes an entity unique.
  - [x] If the user tries to update/delete an entity that does not exist, a message will be shown and there will be no effect on the list of entities.

## Week 5

- [x] Solve the requirements related to the administrator mode.
- [x] Define the `DynamicVector` class which provides the specific operations: `add`, `remove`, `length`, etc. The array of elements must be dynamically allocated.

## Week 7

- Solve all problem requirements.
- `DynamicVector` must be a class template.

## Problem Statements

### Keep calm and adopt a pet

The _“Keep calm and adopt a pet”_ shelter needs a software application to help them find adoptive parents for the dogs they are taking care of. The application can be used in two modes: `administrator` and `user`. When the application is started, it will offer the option to select the mode.\
**Administrator mode:** The application will have a database that holds all the dogs in the shelter at a given moment. The shelter employees must be able to update the database, meaning: add a new dog, delete a dog (when the dog is adopted) and update the information of a dog. Each **Dog** has a `breed`, a `name`, an `age` and a `photograph`. The photograph is memorised as a link towards an online resource (the photograph on the presentation site of the centre). The administrators will also have the option to see all the dogs in the shelter.\
**User mode:** A user can access the application and choose one or more dogs to adopt. The application will allow the user to:

- See the dogs in the database, one by one. When the user chooses this option, the data of the first dog (breed, name, age) is displayed, along with its photograph.
- Choose to adopt the dog, in which case the dog is added to the user’s adoption list.
- Choose not to adopt the dog and to continue to the next. In this case, the information corresponding to the next dog is shown and the user is again offered the possibility to adopt it. This can continue as long as the user wants, as when arriving to the end of the list, if the user chooses next, the application will again show the first dog.
- See all the dogs of a given breed, having an age less than a given number. If no breed is provided, then all the dogs will be considered. The functionalities outlined above apply again in this case.
- See the adoption list.
