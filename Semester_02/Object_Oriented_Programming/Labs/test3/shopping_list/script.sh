#!/bin/bash
files=(
  "main.cpp"
  "domain.cpp"
  "domain.h"
  "service.cpp"
  "service.h"
  "repository.cpp"
  "repository.h"
  "gui.cpp"
  "gui.h"
)

for file in "${files[@]}"; do
  echo "Creating $file"
  touch "$file"
done

cmake_file="CMakeLists.txt"
echo "Creating $cmake_file"
touch "$cmake_file"

echo "Creating .gitignore"
echo "build" > .gitignore

echo "cmake_minimum_required(VERSION 3.10)" > "$cmake_file"
echo "project(untitled)" >> "$cmake_file"
echo "set(CMAKE_CXX_STANDARD 17)" >> "$cmake_file"
echo "set(CMAKE_AUTOMOC ON)" >> "$cmake_file"
echo "set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt")" >> "$cmake_file"
echo "find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)" >> "$cmake_file"
echo "add_executable(untitled main.cpp domain.cpp domain.h service.cpp service.h repository.cpp repository.h gui.cpp gui.h)" >> "$cmake_file"
echo "target_link_libraries(${PROJECT_NAME}
        Qt::Core
        Qt::Gui
        Qt::Widgets)" >> "$cmake_file"