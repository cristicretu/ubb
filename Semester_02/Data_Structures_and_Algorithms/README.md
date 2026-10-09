# Data Structures and Algorithms

Semester 2 · Year 1 · C++, CMake

Abstract data types and how to implement them: dynamic arrays, linked lists (also on arrays), hash tables and binary search trees, with complexity analysis. Each lab implements one ADT behind a fixed interface given by the teachers and has to pass the provided `ShortTest` and `ExtendedTest`.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Interfaces/Bag](Labs/Interfaces/Bag) | Unimplemented starter template for the Bag ADT (all methods are `TODO`) |
| [Labs/Lab_01_Bag](Labs/Lab_01_Bag) | Bag on a dynamic array of frequencies between the min and max element, with iterator |
| [Labs/Lab_02_Matrix](Labs/Lab_02_Matrix) | Sparse matrix stored as a doubly linked list of (line, column, value) nodes |
| [Labs/Lab_03_SortedMap](Labs/Lab_03_SortedMap) | Sorted map on a singly linked list on an array, with a custom ordering relation |
| [Labs/Lab_04_Set](Labs/Lab_04_Set) | Set on a hash table with open addressing and double hashing, resized at load factor 0.75 |
| [Labs/Lab_05_SortedMultiMap](Labs/Lab_05_SortedMultiMap) | Sorted multimap on a binary search tree (each node holds the values of one key); iterator uses an explicit stack |
| [Seminars](Seminars) | Notes: infix to postfix conversion and evaluating postfix expressions with a stack and a queue |
| [Exam](Exam) | Photos of past written exam subjects (complexity, trees, hash tables, Huffman coding) |

## How to run

Labs with a `CMakeLists.txt` (Lab_01, Lab_04, Lab_05):

```sh
cd Labs/Lab_01_Bag
cmake -B build && cmake --build build
./build/Bag
```

The others have no build file; compile all sources directly:

```sh
cd Labs/Lab_02_Matrix
g++ -std=c++17 *.cpp -o app && ./app
```

`App.cpp` runs the short and extended tests (asserts) and prints a final message when they all pass.

## Notes

- Do not change the parts marked `DO NOT CHANGE THIS PART` in the headers; the official tests depend on them.
- `Labs/Interfaces/Bag/app` is a macOS arm64 binary; rebuild it on other systems.
