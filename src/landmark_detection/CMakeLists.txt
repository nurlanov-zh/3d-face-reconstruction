cmake_minimum_required(VERSION 3.10)
project(landmark_detection)

add_library(${PROJECT_NAME} src/face_landmark_detection.cpp)
target_link_libraries(${PROJECT_NAME} dlib common spdlog::spdlog)

target_include_directories(${PROJECT_NAME}  PUBLIC ${PROJECT_SOURCE_DIR}/include)
