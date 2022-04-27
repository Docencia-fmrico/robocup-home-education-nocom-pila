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

Follow_person_cml::Follow_person_cml(const std::string& name/*, const BT::NodeConfiguration & config*/)
: BT::ActionNodeBase(name, {} /*config*/), counter_(0),
turn_pid_(MIN_RANG_BOX, MAX_RANG_BOX, MIN_TURN_SPEED, MAX_TURN_SPEED),
forw_pid_(MIN_FORW_DIST, MAX_FORW_DIST, MIN_FORW_SPEED, MAX_FORW_SPEED)
{
  // dist_sub = nh_.subscribe("/dist_person", 1, &Follow_person_cml::PerceivePersonCallback, this);
  vel_pub_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
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
  double veloc = forw_pid_.get_output(dist);
  double ang = turn_pid_.get_output(dist);  // La distancia para forw y para turn podría no ser el mismo parámetros
/*
  float object = getInput<float>("dist_r").value();
  std::cerr << object << std::endl;
*/
  geometry_msgs::Twist msg;

  ROS_INFO("vel x= ");
  std::cerr << veloc << std::endl;
/*
  if(0.5 < dist < 2.5)
  {
    if(point < 250)
    {
      msg.linear.x = ADVANCE_SPEED;
      msg.angular.z = TURNING_SPEED;
    }
    if( 250 <= point <= 350)
    {
      msg.linear.x = ADVANCE_SPEED;
      //msg.angular.z = TURNING_SPEED;
    }
    if(point > 350)
    {
      msg.linear.x = ADVANCE_SPEED;
      msg.angular.z = -TURNING_SPEED;
    }
  } Ajustar linear y angular segun el PID (aun por implementar)
*/  

  vel_pub_.publish(msg);

  return BT::NodeStatus::RUNNING;
};

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Follow_person_cml>("Follow_person_cml");
}
