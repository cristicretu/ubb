cmake_minimum_required(VERSION 3.22.4)
project(Bag)


# add_compile_options(-fprofile-arcs -ftest-coverage)
# add_link_options(-fprofile-arcs -ftest-coverage)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")


include_directories(.)

add_executable(Bag
        App.cpp
        Bag.cpp
        Bag.h
        BagIterator.cpp
        BagIterator.h
        ExtendedTest.cpp
        ExtendedTest.h
        ShortTest.cpp
        ShortTest.h)
target_compile_options(Bag PRIVATE -g -Wall )
