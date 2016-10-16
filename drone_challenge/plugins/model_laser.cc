#include "model_laser.hh"

#include <sensor_msgs/LaserScan.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(LaserPlugin)

/////////////////////////////////////////////////
LaserPlugin::LaserPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
LaserPlugin::~LaserPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void LaserPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "LaserPlugin requires a RaySensor.\n";
    return;
  }
  
  this->world = physics::get_world(this->parentSensor->GetWorldName());

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&LaserPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
  
  // Ensure that ROS has been initialized
  if (!ros::isInitialized()) 
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized.");
    return;
  }
  
  // Configure a ROS node 
  this->rosnode_ = new ros::NodeHandle("quadcopter");
  
  // Initiates the publication topic
  this->pub_ = this->rosnode_->advertise<sensor_msgs::LaserScan>("laserdata",10);  
}

/////////////////////////////////////////////////
void LaserPlugin::OnUpdate()
{
  //std::cout << "\n\nLaserPlugin::OnUpdate()";
  //std::cout << "\nAngle Min: " << this->parentSensor->GetAngleMin() << "; Angle Max: " << this->parentSensor->GetAngleMax();
  
  // publish on the topic a message with the new laser sensor data...
  sensor_msgs::LaserScan msg;
  common::Time lasttime = this->parentSensor->GetLastMeasurementTime();
  msg.header.stamp = ros::Time(lasttime.sec, lasttime.nsec);
  printf("\nLastMeasurementTime: %u sec, %u nsec", lasttime.sec, lasttime.nsec);
  //msg.angle_min = this->parentSensor->GetAngleMin();
  //msg.angle_max = this->parentSensor->GetAngleMax();
  pub_.publish(msg);
}