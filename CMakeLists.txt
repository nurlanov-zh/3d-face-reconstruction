cmake_minimum_required(VERSION 3.10)

project(3d-face-reconstruction)
include(CTest)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen")
set(CERES_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-ceres-solver")
set(SPDLOG_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/build-spdlog")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

SET(CXX_MARCH native)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

SET(EXTRA_WARNING_FLAGS "-Wsign-compare")

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(EXTRA_WARNING_FLAGS "${EXTRA_WARNING_FLAGS} -Wno-exceptions")
endif()

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")                   # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -DNDEBUG -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                               # cmake default: "-O3 -DNDEBUG"
SET(CMAKE_CXX_FLAGS " -ftemplate-backtrace-limit=0 -Wall -Wextra ${EXTRA_WARNING_FLAGS} -march=${CXX_MARCH} ${CMAKE_CXX_FLAGS}")


set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.3.7 EXACT REQUIRED MODULE)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})  # for ceres
include_directories(${EIGEN3_INCLUDE_DIR})
  message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")

include_directories(thirdparty/Sophus)
include_directories(thirdparty/CLI11/include)
include_directories(thirdparty/spdlog/include)
include_directories(include)

find_package(Ceres 2.0 REQUIRED CONFIG HINTS "${CERES_ROOT}" NO_DEFAULT_PATH)

find_package(OpenCV REQUIRED COMPONENTS core features2d video)
message(STATUS "Found OpenCV ${OpenCV_VERSION} headers in: ${OpenCV_INCLUDE_DIRS}")
add_library(OpenCV INTERFACE IMPORTED)
set_property(TARGET OpenCV PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})
set_property(TARGET OpenCV PROPERTY INTERFACE_LINK_LIBRARIES ${OpenCV_LIBS})
if(OpenCV_VERSION VERSION_LESS 3)
  set_property(TARGET OpenCV PROPERTY INTERFACE_COMPILE_DEFINITIONS DBATK_OPENCV2)
endif()

find_package(spdlog REQUIRED CONFIG HINTS "${SPDLOG_ROOT}" NO_DEFAULT_PATH)
message(STATUS "Found spdlog")

add_subdirectory(src)
add_subdirectory(thirdparty/googletest)
