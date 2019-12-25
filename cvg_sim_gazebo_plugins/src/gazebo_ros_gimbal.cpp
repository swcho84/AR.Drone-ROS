/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo_ros_gimbal.h"

namespace gazebo
{

GazeboRosGimbal::~GazeboRosGimbal()
{
  event::Events::DisconnectWorldUpdateBegin(_update_connection);
}

void GazeboRosGimbal::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;
  _sdf = sdf;

  _node = transport::NodePtr(new transport::Node());
  _node->Init();

  _targetPitch = 0;
  _pidPitch.Init(0.2, 0, 0, 1, -1, 50, -50);
  _linkBase = _model->GetLink("dji_m600/base_link");

  std::string strPitchLinkName = "dji_m600/gimbal_pitch_link";
  _linkPitch = _model->GetLink(strPitchLinkName);
  if (!_linkPitch)
  {
    gzerr << "GazeboRosGimbal::Load ERROR!::" << strPitchLinkName << std::endl;
  }

  _update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosGimbal::OnUpdate, this, _1));

  gzmsg << "GazeboRosGimbal::Init" << std::endl;

  _linkPitch->SetGravityMode(false);
  physics::Link_V links = _linkPitch->GetChildJointsLinks();
  for (unsigned int i = 0; i < links.size(); i++)
  {
    links[i]->SetGravityMode(false);
  }

  // ROS
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }  
}

void GazeboRosGimbal::OnUpdate(const common::UpdateInfo &_info)
{
  if (!_linkPitch)
  {
    return;
  }
  CreateCommunicationChannels();

  math::Pose baseLinkPose = _linkBase->GetWorldPose();
  ignition::math::Pose3d originAngle(
      baseLinkPose.pos.x,
      baseLinkPose.pos.y,
      baseLinkPose.pos.z-0.315,
      0.0,
      0.0,
      baseLinkPose.rot.GetYaw());
  ignition::math::Vector3d originForce(0.0, 0.0, 0.0);

  _linkPitch->SetWorldPose(originAngle);
  _linkPitch->SetForce(originForce);

  math::Pose world_pose = _linkPitch->GetWorldPose();
  math::Pose relative_pose = _linkPitch->GetRelativePose();
}

GZ_REGISTER_MODEL_PLUGIN(BottomGimbalPlugin);

} // namespace gazebo