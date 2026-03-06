cmake_minimum_required(VERSION 3.8)
project(rudder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_path(HIREDIS_INCLUDE_DIR hiredis/hiredis.h)
find_library(HIREDIS_LIBRARY hiredis)

#Add your executable
add_executable(rudder src/rudder_control.cpp)

target_include_directories(rudder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if (HIREDIS_INCLUDE_DIR AND HIREDIS_LIBRARY)
  target_include_directories(rudder PUBLIC ${HIREDIS_INCLUDE_DIR})
  target_link_libraries(rudder ${HIREDIS_LIBRARY})
else()
  message(FATAL_ERROR "hiredis not found")
endif()

#Link against dependencies
ament_target_dependencies(rudder rclcpp std_msgs)

#Install the executable
install(TARGETS
  rudder
  DESTINATION lib/${PROJECT_NAME})

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
