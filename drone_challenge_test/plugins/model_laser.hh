#ifndef _GAZEBO_LASER_PLUGIN_HH_
#define _GAZEBO_LASER_PLUGIN_HH_

#include <string>

#include <ros/node_handle.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
  /// \brief A plugin for a laser sensor.
  class LaserPlugin : public SensorPlugin
  {
    /// \brief Constructor.  
    public: LaserPlugin();

    /// \brief Destructor.
    public: virtual ~LaserPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the laser sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the laser sensor
    private: sensors::RaySensorPtr parentSensor;
    
    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /// \brief Connection that maintains a link between the laser sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
    
    // ROS management
    private: ros::NodeHandle* rosnode_;    
    
    // Laser sensor data publisher
    private: ros::Publisher pub_;      
  };
}
#endif
