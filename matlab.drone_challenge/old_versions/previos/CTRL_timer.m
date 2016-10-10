
%%
%rosshutdown
clear
clc
%rosip = '192.168.203.129';      % casa vmware server ROS Jade  / Gazebo 6.4
%rosip = '192.168.18.128';       % ESII vmware server ROS Hydro / Gazebo 1.9
rosip = '161.67.8.57';          % workstation server  ROS Jade  / Gazebo 6.4
rosinit(rosip);      
pause(1);

%%
gazebo = GazeboCommunicator();
abejorro = GazeboSpawnedModel('abejorro',gazebo);
%[abejorro_Links,abejorro_Joints] = abejorro.getComponents();

%%

ap = timer_autoPilot(abejorro, 2);
start(ap);


