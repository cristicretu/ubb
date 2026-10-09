cmake_minimum_required(VERSION 3.23)
project(myproject)

set(CMAKE_CXX_STANDARD 20)

include_directories(.)

# List the source and header files manually
set(SOURCE_FILES
   "App.cpp"
   "ExtendedTest.cpp"
   "Set.cpp"
   "SetIterator.cpp"
   "ShortTest.cpp"
)

set(HEADER_FILES
   "ExtendedTest.h"
   "Set.h"
   "SetIterator.h"
   "ShortTest.h"
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
