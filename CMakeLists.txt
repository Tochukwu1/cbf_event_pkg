cmake_minimum_required(VERSION 3.8)
project(cbf_event_pkg)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------
# Dependencies
# ---------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Added for Unicycle/Holonomic Yaw extraction and TF messaging
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED) # <-- Added for Holonomic node

# ---------------------------
# First executable: cbf_node (Standalone Math / CSV generator)
# ---------------------------
add_executable(cbf_node
  src/main.cpp
  src/two_state_system.cpp
)

target_include_directories(cbf_node PUBLIC include)
target_compile_features(cbf_node PUBLIC cxx_std_17)

ament_target_dependencies(cbf_node
  rclcpp
)

target_link_libraries(cbf_node
  Eigen3::Eigen
)

# ---------------------------
# Second executable: cbf_event_node (Unicycle ROS 2 Node)
# ---------------------------
add_executable(cbf_event_node
  src/cbf_event_node.cpp
  # Note: two_state_system.cpp is intentionally omitted here 
  # because the Unicycle node contains its own specific math.
)

target_include_directories(cbf_event_node PUBLIC include)
target_compile_features(cbf_event_node PUBLIC cxx_std_17)

ament_target_dependencies(cbf_event_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(cbf_event_node
  Eigen3::Eigen
)

# ---------------------------
# Third executable: holonomic_cbf_node (Holonomic ROS 2 Node)
# ---------------------------
add_executable(holonomic_cbf_node
  src/holonomic_cbf_node.cpp
)

target_include_directories(holonomic_cbf_node PUBLIC include)
target_compile_features(holonomic_cbf_node PUBLIC cxx_std_17)

ament_target_dependencies(holonomic_cbf_node
  rclcpp
  geometry_msgs
  tf2
  tf2_msgs
  tf2_geometry_msgs
)

target_link_libraries(holonomic_cbf_node
  Eigen3::Eigen
)

# ---------------------------
# Install Directives
# ---------------------------
install(TARGETS 
  cbf_node 
  cbf_event_node
  holonomic_cbf_node # <-- Added so launch file can find it
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------
# Testing (optional)
# ---------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()