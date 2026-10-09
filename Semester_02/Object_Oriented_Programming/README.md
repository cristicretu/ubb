# Object Oriented Programming

Semester 2 · Year 1 · C, C++, Qt 6, CMake, SQLite

From C structs to C++ classes, templates, STL, exceptions, inheritance and polymorphism, then Qt GUIs, undo/redo and the Observer pattern. The main assignment is one app (A4-A10) built up over the semester: "Keep calm and adopt a pet", a dog shelter with administrator and user modes.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | First C program: first n primes, longest run of pairwise coprime neighbors |
| [Labs/A2-3](Labs/A2-3) | Pharmacy medicine stock manager in C: dynamic vector, layered architecture, filters with function pointers |
| [Labs/A4-5](Labs/A4-5) | Dog adoption app in C++ with a templated `DynamicVector`, admin and user modes, tests |
| [Labs/A6-7](Labs/A6-7) | Same app on STL, text file repository, adoption list saved as CSV or HTML, exceptions and validators, SQLite repository (bonus) |
| [Labs/A8-9](Labs/A8-9) | Same app with a Qt 6 GUI and a Qt Charts bar chart |
| [Labs/A10](Labs/A10) | Same Qt app plus undo/redo (`Action` classes on a stack) |
| [Labs/test1](Labs/test1) | Lab test 1 practice, console C++: assignment checker, bills, car manager, gene manager |
| [Labs/test2](Labs/test2) | Lab test 2 practice, inheritance and polymorphism: cars/engines, hospital departments, IoT sensors, old buildings, real estate |
| [Labs/test3](Labs/test3) | Lab test 3 practice, Qt GUIs: bills, equations, search engine, shopping list, task manager, weather; plus photos of subjects |
| [Seminars/Seminar_01](Seminars/Seminar_01) | C basics: planets with structs, small CMake project with headers |
| [Seminars/Seminar_05](Seminars/Seminar_05) | Song playlist app: repository, validators, exceptions, CSV/file playlists, undo actions |
| [Exam/research_grant](Exam/research_grant) | Practical exam solution in Qt: researchers and grant ideas, one window per researcher |
| [Exam/practice](Exam/practice) | Practical exam practice in Qt, several with the Observer pattern: art auction, drive connect, event planner, microbial world, research grant, star catalogue, volunteering |
| [Exam/subjects_practic](Exam/subjects_practic) | Photos of past practical exam subjects |
| [Exam/subjects_written](Exam/subjects_written) | Photos of past written exam subjects |

The A2-A10 folders keep the original assignment statement in their own README.

## How to run

Every project with a `CMakeLists.txt` builds the same way:

```sh
cd Labs/A10
cmake -B build && cmake --build build
cd build && ./<project name from CMakeLists.txt>
```

Most test1 and test2 folders have no build file:

```sh
cd Labs/test2/iot
g++ -std=c++17 *.cpp -o app && ./app
```

## Notes

- The Qt projects set `CMAKE_PREFIX_PATH` to `/opt/homebrew/opt/qt` (Homebrew on Apple Silicon). Change it, or pass `-DCMAKE_PREFIX_PATH=<your Qt path>`, on other systems.
- A6-7, A8-9 and A10 need SQLite3 installed (`find_package(SQLite3 REQUIRED)`).
- Most apps open their data files as `../dogs.txt`, `../tasks.txt` etc., so run the binary from a `build/` folder one level below the sources.
- `.vscode/settings.json` files in test3 contain paths from the original machine; ignore or delete them.
