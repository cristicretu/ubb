cmake_minimum_required(VERSION 3.22.4)
project(Bag)

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
target_compile_options(Bag PRIVATE -g -Wall -O2)
