cmake_minimum_required(VERSION 3.10)
project(untitled)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_PREFIX_PATH /opt/homebrew/opt/qt)
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)
add_executable(untitled main.cpp domain.h session.h repository.h window.h window.cpp observer.h subject.h winmap.h)
target_link_libraries(untitled
        Qt::Core
        Qt::Gui
        Qt::Widgets)
