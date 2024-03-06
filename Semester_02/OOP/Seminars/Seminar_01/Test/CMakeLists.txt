cmake_minimum_required(VERSION 3.10)

# Set your project name
project(YourProjectName)

# Automatically find all .cpp files in the current directory and add them to the SOURCES variable.
file(GLOB SOURCES "*.cpp")

# Specify the executable target to be built from the source files.
add_executable(${PROJECT_NAME} ${SOURCES})

# Optionally, set the C++ standard explicitly. Here, C++14 is used.
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
