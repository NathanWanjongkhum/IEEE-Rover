cmake_minimum_required(VERSION 3.8)
project(ieee_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ieee_rover_node publisher_node/urdf_tutorial.cpp)

ament_target_dependencies(ieee_rover_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS
  ieee_rover_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files from the 'launch' directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/config
)

# Install URDF/Xacro files
install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}/description
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install the package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()