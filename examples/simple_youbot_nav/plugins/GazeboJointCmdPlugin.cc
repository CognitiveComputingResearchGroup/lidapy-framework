#include "GazeboJointCmdPlugin.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboJointCmdPlugin);

GazeboJointCmdPlugin::GazeboJointCmdPlugin()
{
}

GazeboJointCmdPlugin::~GazeboJointCmdPlugin()
{
    event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();

    delete this->rosnode_;
}

void GazeboJointCmdPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    this->model_ = _model;
    this->world_ = _model->GetWorld();

    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
        return;
    }

    this->rosnode_ = new ros::NodeHandle("");

    if (!_sdf->HasElement("topicName"))
    {
        ROS_FATAL("force plugin missing <topicName>, cannot proceed");
        return;
    }
    else
        this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

    ros::SubscribeOptions so = ros::SubscribeOptions::create<simple_youbot_nav::WheelCommand>(
                                   this->topic_name_,1,
                                   boost::bind(&GazeboJointCmdPlugin::UpdateObjectForce,this,_1),
                                   ros::VoidPtr(),
                                   &this->queue_);
    this->sub_ = this->rosnode_->subscribe(so);

    this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboJointCmdPlugin::QueueThread,this ) );


    jointController = model_->GetJointController();

    jointMap = jointController->GetJoints();

    for (JointMap::iterator iter = jointMap.begin(); iter != jointMap.end(); ++iter)
    {
        JointPair joint_pair = *iter;
    }

    posPidMap = this->jointController->GetPositionPIDs();
    velPidMap = this->jointController->GetVelocityPIDs();

    flWheelJoint = jointMap["youbot::wheel_joint_fl"];
    frWheelJoint = jointMap["youbot::wheel_joint_fr"];
    blWheelJoint = jointMap["youbot::wheel_joint_bl"];
    brWheelJoint = jointMap["youbot::wheel_joint_br"];

    flWheelTurnJoint = jointMap["youbot::caster_joint_fl"];
    frWheelTurnJoint = jointMap["youbot::caster_joint_fr"];
    blWheelTurnJoint = jointMap["youbot::caster_joint_bl"];
    brWheelTurnJoint = jointMap["youbot::caster_joint_br"];

    flWheelPID = posPidMap["youbot::wheel_joint_fl"];
    frWheelPID = posPidMap["youbot::wheel_joint_fr"];
    blWheelPID = posPidMap["youbot::wheel_joint_bl"];
    brWheelPID = posPidMap["youbot::wheel_joint_br"];

    lastTime = world_->GetSimTime();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                   boost::bind(&GazeboJointCmdPlugin::UpdateChild, this));

    ROS_INFO("GazeboJointCmdPlugin reporting for duty!");
}

void GazeboJointCmdPlugin::UpdateObjectForce(const boost::shared_ptr< simple_youbot_nav::WheelCommand const > & _msg)
{
    this->wheelCmd.angle = _msg->angle;
    this->wheelCmd.force = _msg->force;
}

void GazeboJointCmdPlugin::UpdateChild()
{
    this->lock_.lock();

    ProcessGrndMvmt();

    this->lock_.unlock();
}

void GazeboJointCmdPlugin::QueueThread()
{
    static const double timeout = 0.01;

    while (this->rosnode_->ok())
    {
        this->queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void GazeboJointCmdPlugin::ProcessGrndMvmt()
{
    double frontWheelStateCmd = 0;

    common::Time curTime = world_->GetSimTime();

    double dt = (curTime - lastTime).Double();

    if (dt > 0)
    {
        flSteeringState = flWheelTurnJoint->GetAngle(0).Radian();
        frSteeringState = frWheelTurnJoint->GetAngle(0).Radian();
        blSteeringState = blWheelTurnJoint->GetAngle(0).Radian();
        brSteeringState = brWheelTurnJoint->GetAngle(0).Radian();

        double flwsError =  flSteeringState - wheelCmd.angle;
        double flwsCmd = flWheelPID.Update(flwsError, dt);

        flWheelTurnJoint->SetForce(0, flwsCmd);

        double frwsError = frSteeringState - wheelCmd.angle;
        double frwsCmd = frWheelPID.Update(frwsError, dt);

        frWheelTurnJoint->SetForce(0, frwsCmd);

        double blwsError = blSteeringState - 0;
        double blwsCmd = blWheelPID.Update(blwsError, dt);

        blWheelTurnJoint->SetForce(0, blwsCmd);

        double brwsError = brSteeringState - 0;
        double brwsCmd = brWheelPID.Update(brwsError, dt);

        brWheelTurnJoint->SetForce(0, brwsCmd);

        flWheelJoint->SetForce(0, wheelCmd.force);
        frWheelJoint->SetForce(0, wheelCmd.force);
        blWheelJoint->SetForce(0, wheelCmd.force);
        brWheelJoint->SetForce(0, wheelCmd.force);

        lastTime = curTime;
    }
    else if (dt < 0)
    {
        lastTime = curTime;
    }
}
}
