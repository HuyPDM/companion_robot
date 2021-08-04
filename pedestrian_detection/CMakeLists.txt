cmake_minimum_required(VERSION 3.9)

set(PROJECT_NAME TestOpenCVCmake)

project(${PROJECT_NAME})

add_compile_options(-Wall -fopenmp -O3)

set(CMAKE_CXX_STANDARD 11)

set(cmake_build_type release)

add_subdirectory(../ncnn ncnn_build)

include_directories(../ncnn/src)

find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(./src/ all_source)

MESSAGE("OpenCV_Include: " ${OpenCV_INCLUDE_DIRS})

add_executable(run main.cpp object_detection.cpp camera_tracking.cpp robot_move.cpp  ${all_source})

target_link_libraries(run ${OpenCV_LIBS} -lblas -ldlib -lJetsonGPIO -pthread ncnn stdc++fs OpenMP::OpenMP_CXX)

MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS})

