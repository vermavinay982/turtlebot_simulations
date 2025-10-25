colcon build --symlink-install

ros2 run my_robot_controller turtle_controller
ros2 interface show turtlesim/msg/pose
ros2 topic info /turtle1/pose 