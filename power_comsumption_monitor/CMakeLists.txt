cmake_minimum_required(VERSION 3.8)
project(power_consumption_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Targetdd
set(POWER_CONSUMPTION_MONITOR_SRC
  src/power_consumption_monitor_core.cpp
  src/configuration_loader.cpp
  src/perf_measurement.cpp
)

# Executable
ament_auto_add_executable(${PROJECT_NAME}
  src/power_consumption_monitor_node.cpp
  ${POWER_CONSUMPTION_MONITOR_SRC}
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)