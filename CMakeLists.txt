cmake_minimum_required(VERSION 3.10)
project(vrp)

set(CMAKE_CXX_STANDARD 20)

# Find and link OpenMP
find_package(OpenMP REQUIRED)

add_executable(vrp main.cpp
        euclidean-solver.cpp
        euclidean_solver.h
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(vrp PRIVATE OpenMP::OpenMP_CXX)
endif()
