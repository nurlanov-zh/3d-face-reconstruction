cmake_minimum_required(VERSION 3.10)

project(icp)

if (MSVC)
	add_compile_definitions(_USE_MATH_DEFINES _HAS_AUTO_PTR_ETC)
endif ()

include_directories(include)

add_library(${PROJECT_NAME} 
	src/icp.cpp)

target_link_libraries(${PROJECT_NAME} 
                    common 
					sparse
                    spdlog::spdlog)
target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)