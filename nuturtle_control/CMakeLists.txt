cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(Doxygen)

########## Begin_Citation [10] ##########

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/InitialPose.srv"
  "srv/Control.srv"
  LIBRARY_NAME ${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

########## End_Citation [10] ##########

# Executables
add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

# Ament target dependencies
ament_target_dependencies(odometry
  rclcpp
  sensor_msgs
  nav_msgs
  turtlelib
  tf2
  tf2_ros)

ament_target_dependencies(circle
  rclcpp
  geometry_msgs
  std_srvs
  turtlelib)

ament_target_dependencies(turtle_control
  rclcpp
  geometry_msgs
  sensor_msgs
  nuturtlebot_msgs)

# Target link libraries
target_link_libraries(turtle_control turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(odometry turtlelib::turtlelib "${cpp_typesupport_target}")
target_link_libraries(circle turtlelib::turtlelib "${cpp_typesupport_target}")

target_include_directories(turtle_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  launch
  srv
  config
  test
  DESTINATION share/nuturtle_control/)

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(turtle_control_test test/turtle_control_test.cpp)
  find_package(catch_ros2 REQUIRED)
  include(CTest)

  target_link_libraries(turtle_control_test
    catch_ros2::catch_ros2_with_node_main)

  ament_target_dependencies(turtle_control_test
    rclcpp std_srvs geometry_msgs nuturtlebot_msgs sensor_msgs)

  install(TARGETS
  turtle_control_test
    DESTINATION lib/${PROJECT_NAME})

  catch_ros2_add_integration_test(turtle_control_test
    LAUNCH_FILE turtle_control_test.launch.xml)

endif()

ament_package()