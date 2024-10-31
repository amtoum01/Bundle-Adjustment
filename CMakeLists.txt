cmake_minimum_required(VERSION 3.5)

project(SFM)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(my_executable src/main.cpp include/control_points.cpp include/correspondences.cpp include/ls_solver.cpp include/preprocess.cpp)
add_executable(random_exec src/random.cpp include/correspondences.cpp)

target_link_libraries(my_executable  ${OpenCV_LIBS})
target_link_libraries(random_exec  ${OpenCV_LIBS})