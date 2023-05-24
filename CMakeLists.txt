cmake_minimum_required(VERSION 3.8)
project(riptide_wtf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)

### Executable
add_executable(riptide_recorder src/riptide_recorder.cpp)
include_directories(${CURSES_INCLUDE_DIR})
ament_target_dependencies(riptide_recorder
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "rosbag2_cpp"
  "rosbag2_transport"
)

### Includes
target_include_directories(riptide_recorder PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

### Install
install(TARGETS riptide_recorder
        DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/riptide_wtf.py
  scripts/riptide_recorder.py
  scripts/quaternion_to_euler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()