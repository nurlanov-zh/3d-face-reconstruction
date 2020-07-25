cmake_minimum_required(VERSION 3.10)

enable_testing()
include(GoogleTest)

add_executable(deformation_transfer_test deformation_transfer_test.cpp)

target_link_libraries(deformation_transfer_test
	gtest
	gtest_main
	common
	deformation_transfer
	spdlog::spdlog
)

gtest_discover_tests(deformation_transfer_test WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} DISCOVERY_TIMEOUT 120)
