if(SPDLOG_INCLUDE_DIR)
  return()
endif()

find_path(SPDLOG_INCLUDE_DIR spdlog/spdlog.h HINT ${SPDLOG_INCLUDE_DIR_HINTS})

set(SPDLOG_INCLUDE_DIRS ${SPDLOG_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(spdlog DEFAULT_MSG SPDLOG_INCLUDE_DIR)
