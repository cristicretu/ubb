cmake_minimum_required(VERSION 3.10)

project(a8-9)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_PREFIX_PATH "/opt/homebrew/opt/qt")


find_package(Qt6 COMPONENTS
        Core
        Gui
        Widgets
        Charts
        REQUIRED)

include_directories(headers)

file(GLOB_RECURSE SOURCES "src/*.cpp")

file(GLOB_RECURSE HEADERS "headers/*.h")

add_executable(${PROJECT_NAME} main.cpp ${SOURCES} ${HEADERS})


find_package(SQLite3 REQUIRED)
include_directories(${SQLite3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SQLite3_LIBRARIES} 
Qt::Core
Qt::Gui
Qt::Widgets
Qt::Charts
)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")



