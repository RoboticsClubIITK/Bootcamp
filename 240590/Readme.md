To run the assignment, you need to

1. run the turtle and turtlesim using
 `ros2 run turtlesim turtlesim_node`
2. run the `colcon build` command in `240590` folder
3. run `ros2 run turtle_pub circle_server` in `240590/` folder
4. install ros2 topics using `sudo apt install ros-humble-ros2topic`
5. Run the command using `os2 topic pub --once /draw_circle_params geometry_msgs/msg/Vector3 "{x: 5, y: 5, z: 2.0}"`

This will run the desired output