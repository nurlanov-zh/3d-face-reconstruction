cmake_minimum_required(VERSION 3.10)

project(non_rigid_icp)

include_directories(include)

add_library(${PROJECT_NAME} 
	src/non_rigid_icp.cpp)

target_link_libraries(${PROJECT_NAME} 
					common 
					spdlog::spdlog)

target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(test)