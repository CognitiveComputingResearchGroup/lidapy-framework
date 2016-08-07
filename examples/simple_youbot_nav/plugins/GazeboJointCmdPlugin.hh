/*
    GazeboJointCmdPlugin receives commands on a ROS topic and
    issues the commands to the joints in a robot model.  In
    this case, the commands are specific to the Kuka youBot.

    Currently, only wheel commands are accepted and translated
    into Gazebo model actuator commands.
*/

#ifndef GAZEBO_JOINT_CMD_PLUGIN_HH
#define GAZEBO_JOINT_CMD_PLUGIN_HH

#include <string>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <simple_youbot_nav/WheelCommand.h>

namespace gazebo
{
typedef std::map<std::string,physics::JointPtr> JointMap;
typedef std::pair<std::string,physics::JointPtr> JointPair;

typedef std::map<std::string,common::PID> PIDMap;
typedef std::pair<std::string,common::PID> PIDPair;

class GazeboJointCmdPlugin : public ModelPlugin
{
public:
    GazeboJointCmdPlugin();

public:
    virtual ~GazeboJointCmdPlugin();

protected:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
    virtual void UpdateChild();

private:
    void UpdateObjectForce(const boost::shared_ptr< simple_youbot_nav::WheelCommand const > & _msg);

private:
    void QueueThread();

private:
    // Not yet implemented
    void ProcessArmMvmt();
    void ProcessGrndMvmt();

private:
    physics::WorldPtr world_;

private:
    physics::ModelPtr model_;

private:
    physics::LinkPtr link_;

private:
    JointMap jointMap;

private:
    PIDMap posPidMap;

private:
    PIDMap velPidMap;

private:
    ros::NodeHandle* rosnode_;

private:
    ros::Subscriber sub_;

private:
    boost::mutex lock_;

private:
    std::string topic_name_;

private:
    ros::CallbackQueue queue_;

private:
    boost::thread callback_queue_thread_;

private:
    simple_youbot_nav::WheelCommand wheelCmd;

private:
    event::ConnectionPtr update_connection_;

private:
    common::Time lastTime;

private:
    physics::JointControllerPtr jointController;

private:
    physics::JointPtr flWheelJoint;
private:
    physics::JointPtr frWheelJoint;
private:
    physics::JointPtr blWheelJoint;
private:
    physics::JointPtr brWheelJoint;
private:
    physics::JointPtr flWheelTurnJoint;
private:
    physics::JointPtr frWheelTurnJoint;
private:
    physics::JointPtr blWheelTurnJoint;
private:
    physics::JointPtr brWheelTurnJoint;

private:
    common::PID flWheelPID;
private:
    common::PID frWheelPID;
private:
    common::PID blWheelPID;
private:
    common::PID brWheelPID;
private:
    common::PID flVelocityPID;
private:
    common::PID frVelocityPID;
private:
    common::PID blVelocityPID;
private:
    common::PID brVelocityPID;
private:
    double flWheelState;
private:
    double frWheelState;
private:
    double blWheelState;
private:
    double brWheelState;

private:
    double flSteeringState;
private:
    double frSteeringState;
private:
    double blSteeringState;
private:
    double brSteeringState;

private:
    double flWheelCmd;
private:
    double frWheelCmd;
private:
    double blWheelCmd;
private:
    double brWheelCmd;

};
}
#endif
