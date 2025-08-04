cmake_minimum_required(VERSION 3.8)
project(simulator)

# Enable O3 optimization
add_compile_options(-O3 -Wall -Wextra -Wpedantic)

# Find packages
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Ament specific configuration
ament_package()

# Specify additional locations of header files
include_directories(include)

# Declare C++ executable
add_executable(simulation
    src/simulation.cpp
    src/simulator/map.cpp
    src/simulator/robot.cpp
    src/simulator/camera.cpp
    src/simulator/planner.cpp
    src/simulator/detection.cpp
)

# Specify library to link a library or executable target against
ament_target_dependencies(simulation
    rclcpp 
    OpenCV 
    tf2_ros
    nav_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
)

# Copy files for installation
install(TARGETS simulation
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
