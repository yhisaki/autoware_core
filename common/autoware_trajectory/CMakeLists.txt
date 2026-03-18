cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory)

# ==============================================================================
# Compiler workarounds
# ==============================================================================

# cspell: ignore Warray, Wstringop, stringop
# Disable array bounds warnings for GCC 13+ as they appear to be false positives
# NOTE: workaround https://github.com/autowarefoundation/autoware_core/pull/644
# Delete in the future
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 13)
  set_source_files_properties(src/interpolator/lane_ids_interpolator.cpp
    PROPERTIES COMPILE_FLAGS "-Wno-array-bounds -Wno-stringop-overflow"
  )
endif()

# ==============================================================================
# Core library
# ==============================================================================

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detail/logging.cpp
  src/detail/types.cpp
  src/detail/util.cpp
  src/interpolator/akima_spline.cpp
  src/interpolator/cubic_spline.cpp
  src/interpolator/lane_ids_interpolator.cpp
  src/interpolator/linear.cpp
  src/interpolator/result.cpp
  src/interpolator/spherical_linear.cpp
  src/path_point.cpp
  src/path_point_with_lane_id.cpp
  src/point.cpp
  src/pose.cpp
  src/temporal_trajectory.cpp
  src/threshold.cpp
  src/trajectory_point.cpp
  src/utils/closest.cpp
  src/utils/crossed.cpp
  src/utils/find_if.cpp
  src/utils/find_intervals.cpp
  src/utils/find_nearest.cpp
  src/utils/footprint.cpp
  src/utils/pretty_build.cpp
  src/utils/reference_path.cpp
  src/utils/shift.cpp
  src/utils/temporal/find_stop_points.cpp
  src/utils/velocity.cpp
)

# ==============================================================================
# Tests & Examples (only when BUILD_TESTING is ON)
# ==============================================================================

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  # --- Test dependencies ---
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  find_package(autoware_pyplot REQUIRED)
  find_package(autoware_motion_utils REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(autoware_test_utils REQUIRED)
  find_package(autoware_utils_geometry REQUIRED)
  find_package(range-v3 REQUIRED)

  # ---------------------------------------------------------------------------
  # Helper: register a gtest target
  # ---------------------------------------------------------------------------
  function(add_trajectory_test test_file)
    get_filename_component(_name ${test_file} NAME_WE)
    ament_auto_add_gtest(${_name} ${test_file})
    target_link_libraries(${_name} fmt::fmt Python3::Python pybind11::embed yaml-cpp)
  endfunction()

  # ---------------------------------------------------------------------------
  # Helper: register an example executable
  # ---------------------------------------------------------------------------
  function(add_trajectory_example example_file)
    get_filename_component(_name ${example_file} NAME_WE)
    ament_auto_add_executable(${_name} ${example_file})
    target_link_libraries(${_name}
      ${PROJECT_NAME}
      range-v3::range-v3
      Python3::Python
      pybind11::embed
      yaml-cpp
    )
    ament_target_dependencies(${_name}
      autoware_pyplot
      autoware_motion_utils
      ament_index_cpp
      autoware_test_utils
      autoware_utils_geometry
    )
    target_include_directories(${_name} SYSTEM PRIVATE
      ${autoware_pyplot_INCLUDE_DIRS}
      ${autoware_motion_utils_INCLUDE_DIRS}
      ${ament_index_cpp_INCLUDE_DIRS}
      ${autoware_test_utils_INCLUDE_DIRS}
      ${autoware_utils_geometry_INCLUDE_DIRS}
    )
  endfunction()

  # ---------------------------------------------------------------------------
  # Tests
  # ---------------------------------------------------------------------------
  file(GLOB _test_sources CONFIGURE_DEPENDS test/test_*.cpp)

  ament_auto_add_gtest(${PROJECT_NAME}_test ${_test_sources})
  target_link_libraries(${PROJECT_NAME}_test
    fmt::fmt
    Python3::Python
    pybind11::embed
    yaml-cpp
  )

  # ---------------------------------------------------------------------------
  # Examples (opt-in via BUILD_EXAMPLES, default ON when testing)
  # ---------------------------------------------------------------------------
  option(BUILD_EXAMPLES "Build example executables" ON)
  if(BUILD_EXAMPLES)
    file(GLOB _example_sources CONFIGURE_DEPENDS examples/example_*.cpp)
    foreach(_src IN LISTS _example_sources)
      add_trajectory_example(${_src})
    endforeach()
  endif()
endif()

# ==============================================================================
# Install
# ==============================================================================

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    test_data
)
