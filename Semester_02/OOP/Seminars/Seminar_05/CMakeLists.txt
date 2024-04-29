cmake_minimum_required(VERSION 3.10)

project(a4-5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(headers)

file(GLOB_RECURSE SOURCES "*.cpp")

file(GLOB_RECURSE HEADERS "*.h")

# add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_executable(${PROJECT_NAME} build
CSVPlaylist.cpp
CSVPlaylist.h
FilePlaylist.cpp
FilePlaylist.h
main.cpp
PlayList.cpp
PlayList.h
Repository.cpp
Repository.h
RepositoryExceptions.cpp
RepositoryExceptions.h
Service.cpp
Service.h
Song.cpp
Song.h
SongValidator.cpp
SongValidator.h
UI.cpp
UI.h
Utils.cpp
Utils.h
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")