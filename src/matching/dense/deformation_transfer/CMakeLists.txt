cmake_minimum_required(VERSION 3.10)

project(deformation_transfer)

include_directories(include)

add_library(${PROJECT_NAME} 
	src/deformation_transfer.cpp)

target_link_libraries(${PROJECT_NAME} 
						common 
						spdlog::spdlog
						${SuiteSparse_LIBRARIES})

target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(test)