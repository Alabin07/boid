cmake_minimum_required(VERSION 3.16)
project(BoidSimulation C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find SDL3
find_package(SDL3 REQUIRED)

# Create the executable
add_executable(boid_sim main.c)

# Link SDL3 and math library
target_link_libraries(boid_sim PRIVATE SDL3::SDL3 m)
