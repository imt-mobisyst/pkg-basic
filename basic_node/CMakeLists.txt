cmake_minimum_required(VERSION 3.8)
project(basic_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(basic_msgs REQUIRED)

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

# Install launch files.
install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Python scripts
install( DIRECTORY scripts/basic DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/color_race DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/move_simple DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/move_to DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/multiplexer DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/parasit DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/pulse_monitor DESTINATION lib/${PROJECT_NAME} )
install( PROGRAMS scripts/tf_snifer DESTINATION lib/${PROJECT_NAME} )

ament_package()
