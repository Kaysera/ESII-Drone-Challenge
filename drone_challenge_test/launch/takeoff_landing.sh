clear

#export ROS_IP=$(ifconfig eth0 | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}')
#echo '$'ROS_IP: $ROS_IP

roslaunch drone_gazebo takeoff_landing.launch

