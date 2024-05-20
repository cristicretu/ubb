cmake_minimum_required(VERSION 3.12)
project(l5)

add_executable(l5 
App.cpp
ExtendedTest.cpp
ExtendedTest.h
ShortTest.cpp
ShortTest.h
SMMIterator.cpp
SMMIterator.h
SortedMultiMap.cpp
SortedMultiMap.h)

target_link_libraries(l5 ${LIBRARIES})