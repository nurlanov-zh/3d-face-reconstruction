cmake_minimum_required(VERSION 3.10)

project(face_model)

include_directories(include)

add_library(${PROJECT_NAME} 
	src/face_model.cpp)

target_link_libraries(${PROJECT_NAME} 
					common 
					spdlog::spdlog)

target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)

#add_subdirectory(test)