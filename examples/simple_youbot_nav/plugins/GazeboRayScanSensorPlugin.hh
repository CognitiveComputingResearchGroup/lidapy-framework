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

#include <sensor_msgs/LaserScan.h>

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
    void getRanges(sensor_msgs::LaserScan& msg);

private:
    void publish(sensor_msgs::LaserScan& msg);

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
