cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nuturtle_control REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# Define the executable target
add_executable(slam src/slam.cpp)

ament_target_dependencies(slam
  rclcpp
  sensor_msgs
  nav_msgs
  turtlelib
  tf2
  tf2_ros
  nuturtle_control
  visualization_msgs)

target_link_libraries(slam turtlelib::turtlelib "${cpp_typesupport_target}" ${ARMADILLO_LIBRARIES})

target_include_directories(slam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(slam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS slam
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()