cmake_minimum_required(VERSION 3.8)
project(bt_executer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(cnr_param REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp 
  std_msgs
  behaviortree_ros2
  btcpp_ros2_interfaces
  cnr_param
)

# =====
# BUILD
# =====

# Build the skills library (action servers)
add_executable(bt_executer_node
  src/bt_executer_node.cpp
)

ament_target_dependencies(bt_executer_node ${THIS_PACKAGE_DEPS})

# =======
# INSTALL
# =======
install(TARGETS
  bt_executer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME} )

# ====
# TEST
# ====
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
