cmake_minimum_required(VERSION 3.10)

project(matching)

if (MSVC)
	add_compile_definitions(_USE_MATH_DEFINES)
endif ()

include_directories(include)

add_library(${PROJECT_NAME} 
    src/sparse/sparse_aligner.cpp)

target_link_libraries(${PROJECT_NAME} 
                    common 
                    spdlog::spdlog)
target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)