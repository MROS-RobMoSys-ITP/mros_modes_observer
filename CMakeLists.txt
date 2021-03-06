cmake_minimum_required(VERSION 3.5)
project(mros_modes_observer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(system_modes REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
)


add_executable(modes_observer_node src/modes_observer_node.cpp)

add_library(modes_observer_core SHARED src/modes_observer.cpp )

set(dependencies 
  rclcpp
  system_modes
  std_msgs
  rclcpp_lifecycle
  diagnostic_msgs
)
ament_target_dependencies(modes_observer_core  ${dependencies}
)
ament_target_dependencies(modes_observer_node  ${dependencies}
)

target_link_libraries(modes_observer_node modes_observer_core)

target_include_directories(modes_observer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(
  TARGETS modes_observer_core
  EXPORT modes_observer_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  TARGETS modes_observer_node
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
  )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
