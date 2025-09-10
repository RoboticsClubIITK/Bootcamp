# cmake_minimum_required(VERSION 3.5)
# project(turtle_pub)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# # Generate service interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/DrawCircle.srv"
# )

# ament_export_dependencies(rosidl_default_runtime)

# ament_package()


cmake_minimum_required(VERSION 3.5)
project(turtle_pub)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DrawCircle.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
