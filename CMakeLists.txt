cmake_minimum_required(VERSION 3.8)
project(riptide_wtf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Curses REQUIRED)

### Executable
add_executable(wtf_node src/wtf_node.cpp)
include_directories(${CURSES_INCLUDE_DIR})
ament_target_dependencies(wtf_node
        rclcpp
        rosidl_default_generators
        std_srvs
        )
target_link_libraries(wtf_node "${cpp_typesupport_target}" ${CURSES_LIBRARIES})

### Includes
target_include_directories(wtf_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

### Install
install(TARGETS wtf_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()