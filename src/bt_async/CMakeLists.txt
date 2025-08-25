cmake_minimum_required(VERSION 3.8)
project(bt_async)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Add the async executable
add_executable(async_coffee_bt 
  src/async_coffee_bt.cpp
  src/nodes/heat_water_node.cpp
  src/nodes/grind_beans_node.cpp  
  src/nodes/brew_coffee_node.cpp
  src/nodes/say_something_node.cpp
  src/nodes/async_action_node.cpp
)
target_include_directories(async_coffee_bt PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
ament_target_dependencies(async_coffee_bt 
  rclcpp 
  behaviortree_cpp_v3
  ament_index_cpp
)

# Install executable
install(TARGETS async_coffee_bt
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt
  DESTINATION share/${PROJECT_NAME})

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
