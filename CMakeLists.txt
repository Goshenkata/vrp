cmake_minimum_required(VERSION 3.10)
project(VRPProject)

set(CMAKE_CXX_STANDARD 17)

# OpenMP support (if available)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Add source files
add_executable(vrp_solver
        main.cpp
#        VRPSolver.cpp
#        Route.cpp
#        Solution.cpp
#        DataLoader.cpp
)

# Include the current source directory
target_include_directories(vrp_solver PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
