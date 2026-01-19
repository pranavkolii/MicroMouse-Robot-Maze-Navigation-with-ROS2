cmake_minimum_required(VERSION 3.22)
project(rwa4 VERSION 1.0 LANGUAGES C CXX)

add_executable(rwa4_cpp
    src/main.cpp
    src/maze_api.cpp
    src/maze.cpp
    src/cell.cpp
    src/robot.cpp
    src/dfs_algorithm.cpp
    src/bfs_algorithm.cpp   # optional
)
target_include_directories(rwa4_cpp PRIVATE include)

# Set C++17 standard for the targets
set_property(TARGET rwa4_cpp PROPERTY CXX_STANDARD 17)
set_property(TARGET rwa4_cpp PROPERTY CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wall -Wextra -pedantic-errors)

#find valgrind executable on the system
find_program(VALGRIND_EXECUTABLE valgrind)

#only create the targets if valgrind is found
if(VALGRIND_EXECUTABLE)
    add_custom_target(memcheck)
    add_custom_command(
        TARGET memcheck 
        COMMAND ${VALGRIND_EXECUTABLE} --leak-check=full --track-origins=yes $<TARGET_FILE:rwa4_cpp> 
        COMMENT "Running Valgrind Memcheck on rwa4_cpp"
    )
    add_dependencies(memcheck rwa4_cpp)

endif()
