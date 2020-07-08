cmake_minimum_required(VERSION 3.10)

enable_testing()
include(GoogleTest)

add_executable(nricp_test non_rigid_icp_test.cpp)

target_link_libraries(nricp_test
	gtest
	gtest_main
	common
	non_rigid_icp
	spdlog::spdlog
)

gtest_discover_tests(nricp_test WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} DISCOVERY_TIMEOUT 120)
