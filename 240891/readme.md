# Assignment 1

## Instructions to Run the Code

### Terminal 1
Start the **turtlesim node**:

```bash
ros2 run turtlesim turtlesim_node
```
### Terminal 2

```bash
colcon build
source install/setup.bash
ros2 run turtle_pub client
```