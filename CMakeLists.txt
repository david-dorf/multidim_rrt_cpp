cmake_minimum_required(VERSION 3.8)
project(multidim_rrt_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(rrt2D src/rrt2D.cpp)
ament_target_dependencies(rrt2D rclcpp)
ament_target_dependencies(rrt2D std_msgs)
ament_target_dependencies(rrt2D nav_msgs)
ament_target_dependencies(rrt2D visualization_msgs)

target_include_directories(rrt2D PUBLIC
  ${CMAKE_SOURCE_DIR}/include/multidim_rrt_cpp
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rrt2D PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS rrt2D
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
