clear
#!/bin/bash
source /opt/ros/jade/setup.bash
export ROS_IP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')
roslaunch drone_gazebo empty_world.launch

