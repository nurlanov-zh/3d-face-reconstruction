cmake_minimum_required(VERSION 3.10)

project(face_model)

include_directories(include)

add_library(${PROJECT_NAME} src/face_model.cpp)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)

    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX common spdlog::spdlog ceres)

endif()


target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)

#add_subdirectory(test)