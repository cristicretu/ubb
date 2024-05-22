cmake_minimum_required(VERSION 3.10)
project(untitled)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt")
find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        REQUIRED)

add_executable(untitled main.cpp domain.cpp domain.h service.cpp service.h repository.cpp repository.h gui.cpp gui.h)
target_link_libraries(${PROJECT_NAME}
        Qt::Core
        Qt::Gui
        Qt::Widgets
        )

