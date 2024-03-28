cmake_minimum_required(VERSION 3.22.4)
project(a)

set(CMAKE_C_STANDARD 11)

include_directories(.)

add_executable(main
        main.c
        src/ui/UI.c
        headers/ui/ui.h
        src/domain/domain.c
        src/domain/func_operation.c
        headers/domain/domain.h
        headers/domain/func_operation.h
        src/domain/dynarray.c
        headers/domain/dynarray.h
        src/controller/service.c
        headers/controller/service.h
        src/repository/repository.c
        headers/repository/undo_repository.h
        src/repository/undo_repository.c
        headers/repository/repository.h
        test_functions.c
        test_functions.h
        
        )

target_compile_options(main PRIVATE -g -Wall -O2)
