cmake_minimum_required(VERSION 3.8)
project(ros2_nyokkey4_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  joint_trajectory_controller
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
  urdf
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/ros2_nyokkey4_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  joint_trajectory_controller
  trajectory_msgs
  controller_interface
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(controller_interface ros2_nyokkey4_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
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

add_executable(test_plugin_loader test/test_plugin_loader.cpp)
ament_target_dependencies(test_plugin_loader
  controller_interface
  pluginlib
  rclcpp
)
install(TARGETS test_plugin_loader
  DESTINATION lib/${PROJECT_NAME})


install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# install plugin
install(
  FILES ros2_nyokkey4_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
