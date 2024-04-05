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
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(cnr_param REQUIRED)
find_package(trajectory_loader REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(EXECUTER_DEPS
    rclcpp
    std_msgs
    behaviortree_ros2
    btcpp_ros2_interfaces
    cnr_param
    )
set(SKILL_LIBRARY_DEPS
    rclcpp
    geometry_msgs
    behaviortree_ros2
    btcpp_ros2_interfaces
    trajectory_loader
    cnr_param
    tf2
    tf2_ros
    )

# =====
# BUILD
# =====
# Build the skills library
add_library(move_to_skill SHARED
    src/skills/move_to_skill.cpp
    )
target_compile_definitions(move_to_skill PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(move_to_skill PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(move_to_skill ${SKILL_LIBRARY_DEPS})

add_library(trajectory_loader_skill SHARED
    src/skills/trajectory_loader_skill.cpp
    )
target_compile_definitions(trajectory_loader_skill PRIVATE  BT_PLUGIN_EXPORT)
target_include_directories(trajectory_loader_skill PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    )
ament_target_dependencies(trajectory_loader_skill ${SKILL_LIBRARY_DEPS})

# Build the skill executer node
add_executable(bt_executer_node
    src/bt_executer_node.cpp
    )
ament_target_dependencies(bt_executer_node ${EXECUTER_DEPS})

# =======
# INSTALL
# =======
install(TARGETS
    move_to_skill
    trajectory_loader_skill
    bt_executer_node
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY include
    DESTINATION include/)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

# ====
# TEST
# ====
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_cmake_gmock REQUIRED)
#   # ament_add_gtest(some_test <test_sources>)
#   # ament_add_gmock(some_test <test_sources>)
# endif()

ament_package()
