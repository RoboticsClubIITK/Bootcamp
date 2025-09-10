cmake_minimum_required(VERSION 3.5)
project(turtle_pub)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # ⭐ Needed for srv generation

# Generate the service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DrawCircle.srv"
  DEPENDENCIES geometry_msgs std_srvs           # ⭐ Add dependencies if used in srv
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(
  PROGRAMS
    turtle_pub/draw_circle_server.py
    turtle_pub/draw_circle_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
