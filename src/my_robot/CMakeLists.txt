cmake_minimum_required(VERSION 3.5)
project(my_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install Python scripts
install(
  DIRECTORY src
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(
  DIRECTORY launch worlds resource
  DESTINATION share/${PROJECT_NAME}
)

# Enable ament package
ament_package()
