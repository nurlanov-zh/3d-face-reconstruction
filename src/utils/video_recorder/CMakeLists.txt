cmake_minimum_required(VERSION 3.1.0)

project(recorder)

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
add_library(OpenCV INTERFACE IMPORTED)
set_property(TARGET OpenCV PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property(TARGET OpenCV PROPERTY INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS})

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable(recorder recorder.cpp)
set_property(TARGET recorder PROPERTY CXX_STANDARD 11)
target_link_libraries(recorder realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES})
