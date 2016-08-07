/*
    GazeboRayScanSensorPlugin generates messages on a ROS topic
    from the laser scan sensor on a Kuka youBot model in Gazebo.
*/

#ifndef GAZEBO_RAY_SCAN_PLUGIN_HH
#define GAZEBO_RAY_SCAN_PLUGIN_HH

#include <string>

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

#include <simple_youbot_nav/RayScanSensor.h>

namespace gazebo
{
class GazeboRayScanSensorPlugin : public SensorPlugin
{
public:
    GazeboRayScanSensorPlugin();
public:
    virtual ~GazeboRayScanSensorPlugin();

private:
    virtual void Load(sensors::SensorPtr _sensor,
                      sdf::ElementPtr _sdf);

private:
    virtual void OnUpdate();

private:
    void InitPublisher();

private:
    void getRanges(simple_youbot_nav::RayScanSensor& msg);

private:
    void publish(simple_youbot_nav::RayScanSensor& msg);

private:
    sensors::RaySensorPtr parentSensor;

private:
    event::ConnectionPtr updateConnection;

private:
    std::string topicName;

private:
    ros::NodeHandle nh;

private:
    ros::Publisher publisher;

};
}
#endif
