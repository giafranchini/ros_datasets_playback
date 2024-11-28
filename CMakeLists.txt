cmake_minimum_required(VERSION 3.8)
project(ros_datasets_playback)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS 
  scripts/shell1.sh
  scripts/shell2.sh
  scripts/shell3.sh
  scripts/shell4.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  bags 
  config 
  launch
  rviz 
  scripts 
  urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()