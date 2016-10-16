clear

#export ROS_IP=$(ifconfig eth0 | grep "Direc. inet" | awk -F: '{print $2}' | awk '{print $1}')
#echo '$'ROS_IP: $ROS_IP

roslaunch drone_gazebo takeoff_landing.launch

