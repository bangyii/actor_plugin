/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "actor_plugin/actor_plugin.hh"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the actor id
  if (_sdf->HasElement("actor_id"))
    this->id = _sdf->Get<int>("actor_id");
  else
    ROS_WARN("Actor ID not specified, motion will not be defined");

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

  // Create a named topic, and subscribe to it.
  // ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>("/pedsim_simulator/simulated_agents", 1, boost::bind(&ActorPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
  ros::SubscribeOptions so = ros::SubscribeOptions::create<gamma_simulator::AgentStates>("/gamma_simulator/agent_states", 1, boost::bind(&ActorPlugin::OnAgentMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread = std::thread(std::bind(&ActorPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  ignition::math::Pose3d pose = this->actor->WorldPose();

  if(agent_pose.orientation.w != 0.0)
  {
    pose.Pos().X(agent_pose.position.x);
    pose.Pos().Y(agent_pose.position.y);
    pose.Pos().Z(1.0138);

    tf2::Quaternion agent_quat;
    agent_quat.setX(agent_pose.orientation.x);
    agent_quat.setY(agent_pose.orientation.y);
    agent_quat.setZ(agent_pose.orientation.z);
    agent_quat.setW(agent_pose.orientation.w);

    tf2::Matrix3x3 mat(agent_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, 1.5707 + yaw);
  }

  // Distance traveled is used to coordinate motion with the walking animation
  double distanceTraveled = (pose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}