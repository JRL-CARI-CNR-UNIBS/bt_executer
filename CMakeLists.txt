cmake_minimum_required(VERSION 3.8)
project(btcpp_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp 
  std_msgs
  behaviortree_ros2
  btcpp_ros2_interfaces
)

# =====
# BUILD
# =====

# Build the skills library (action servers)
### STATIC VERSION ###
add_executable(bt_skills_lib SHARED
  src/${PROJECT_NAME}/sleep_action.cpp
)
target_include_directories(bt_skills_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bt_skills_lib ${THIS_PACKAGE_DEPS})
######################

### PLUGIN VERSION ###
add_library(sleep_plugin SHARED
  src/${PROJECT_NAME}/sleep_action.cpp
)
target_compile_definitions(sleep_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(sleep_plugin ${THIS_PACKAGE_DEPS})
######################

add_executable(bt_skills_lib_dyn src/sleep_client.cpp)
target_compile_definitions(sleep_client_dyn PRIVATE USE_SLEEP_PLUGIN )
target_link_libraries(sleep_client_dyn sleep_plugin )
ament_target_dependencies(sleep_client_dyn ${THIS_PACKAGE_DEPS})


# Build the skills executor node (e.g. client to call actions)
### STATIC VERSION ###
add_executable(bt_skills_node
  src/bt_skills_node.cpp
)

target_include_directories(bt_skills_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bt_skills_node ${THIS_PACKAGE_DEPS})
######################

### PLUGIN VERSION ###


######################



# target_include_directories(bt_skills_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(bt_skills_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# =======
# INSTALL
# =======
install(TARGETS
  bt_skills_node
  DESTINATION lib/${PROJECT_NAME}
)


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
