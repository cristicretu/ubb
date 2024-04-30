cmake_minimum_required(VERSION 3.10)

project(a6-7)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(headers)

file(GLOB_RECURSE SOURCES "src/*.cpp")

file(GLOB_RECURSE HEADERS "headers/*.h")

add_executable(${PROJECT_NAME} main.cpp ${SOURCES} ${HEADERS})

find_package(SQLite3 REQUIRED)
include_directories(${SQLite3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SQLite3_LIBRARIES})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")



