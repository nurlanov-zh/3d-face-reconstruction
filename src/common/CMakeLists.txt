cmake_minimum_required(VERSION 3.10)

project(common)

include_directories(thirdparty/Sophus)

add_library(${PROJECT_NAME} INTERFACE)

