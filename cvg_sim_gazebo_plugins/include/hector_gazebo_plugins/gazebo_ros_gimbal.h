#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GIMBAL_H

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
class GazeboRosGimbal : public ModelPlugin
{
public:
    GazeboRosGimbal()
        : ModelPlugin() {}
    virtual ~GazeboRosGimbal();

protected:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
    sdf::ElementPtr _sdf;
    std::vector<event::ConnectionPtr> _connections;

private:
    transport::NodePtr _node;

private:
    physics::ModelPtr _model;
    physics::LinkPtr _linkBase;
    physics::LinkPtr _linkPitch;

private:
    event::ConnectionPtr _update_connection;

private:
    common::PID _pidPitch;
};
} // namespace gazebo
#endif