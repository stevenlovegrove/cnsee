project(cnsee)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Pangolin (https://github.com/stevenlovegrove/Pangolin)
find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(cnsee src/main.cpp)
target_link_libraries(cnsee ${Pangolin_LIBRARIES})
