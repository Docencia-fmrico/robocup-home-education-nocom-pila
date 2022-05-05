// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robocup_nocom_pila/Follow_person_cml.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "robocup_nocom_pila/PIDController.hpp"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Follow_person_cml::Follow_person_cml(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config ), counter_(0), 
turn_pid_(MIN_RANG_BOX, MAX_RANG_BOX, MIN_TURN_SPEED, MAX_TURN_SPEED),
forw_pid_(MIN_FORW_DIST, MAX_FORW_DIST, MIN_FORW_SPEED, MAX_FORW_SPEED)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
Follow_person_cml::halt()
{
    ROS_INFO("Follow_person_cml halt");
}


BT::NodeStatus
Follow_person_cml::tick()
{
  ROS_INFO("Follow_person_cml tick");

  geometry_msgs::Twist msg;
  
  dist_r = getInput<float>("r_dist").value();
  std::cerr << "Distancia real: " << dist_r << std::endl; 
  dist_r = (dist_r - MIN_DISTANCE) / MAX_VEL_DISTANCE;
  centre_r = getInput<double>("r_centre").value();

  forw_pid_.set_pid(0.21, 0.06, 0.43);
  lin_vel_ = forw_pid_.get_output(dist_r) *10;
  lin_vel_ = std::clamp(lin_vel_, MIN_FORW_SPEED, MAX_FORW_SPEED);

  ang_vel_ = turn_pid_.get_output(centre_r);
  //std::cerr << "dist_r: " << dist_r << "centre_r: " << centre_r << std::endl; 
  //std::cerr << "vel x= " << lin_vel_ << " ang z= " << ang_vel_ * 3.0 << std::endl; 
  msg.linear.x = lin_vel_;
  msg.angular.z = - ang_vel_ * 3.0;

  pub_vel_.publish(msg);
  return BT::NodeStatus::RUNNING;
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Follow_person_cml>("Follow_person_cml");
}
