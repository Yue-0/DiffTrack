cmake_minimum_required(VERSION 3.8)
project(tracker)

# Enable O3 optimization
add_compile_options(-O3 -Wall -Wextra -Wpedantic)

# Find packages
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Ament specific configuration
ament_package()

# Specify additional locations of header files
include_directories(include)

# Declare C++ executables
add_executable(tracker
    src/planning.cpp
    src/planner/map.cpp
    src/planner/mpc.cpp
    src/planner/astar.cpp
    src/predictor/bezier.cpp
)
add_executable(prediction
    src/prediction.cpp
    src/predictor/bezier.cpp
    src/predictor/predictor.cpp
)

# Specify library to link a library or executable target against
ament_target_dependencies(tracker
    rclcpp 
    OpenCV
    nav_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
)
ament_target_dependencies(prediction
    rclcpp
    nav_msgs
    geometry_msgs
)

# Copy files for installation
install(TARGETS tracker
    DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS prediction
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
