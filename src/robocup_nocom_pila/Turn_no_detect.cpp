// Copyright 2022 Nocom-pila
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

#include "robocup_nocom_pila/Turn_no_detect.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Turn_no_detect::Turn_no_detect(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0), nh("~")
{
  nh.getParam("TURNING_SPEED", TURNING_SPEED);
  nh.getParam("TURNING_TIME", TURNING_TIME);
  pub_vel_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
}

void
Turn_no_detect::halt()
{
  ROS_INFO("Turn_no_detect halt");
}

BT::NodeStatus
Turn_no_detect::tick()
{
  ROS_INFO("Turn_no_detect tick");

  person = getInput<int>("r_person").value();
  if ( person >= 8)
  {
    return BT::NodeStatus::SUCCESS;
  }

  state = getInput<int>("r_state").value();
  // std::cerr << state << std::endl;

  if (state == 0)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else if (state == 1)
  {
    geometry_msgs::Twist cmd;
    cmd.angular.z = TURNING_SPEED;
    if ( time == 0)
    {
      turn_ts_ = ros::Time::now();
      time = 1;
    }

    if ((ros::Time::now()-turn_ts_).toSec() > TURNING_TIME )
    {
      ROS_INFO("FINISED");
      time = 0;
      setOutput<int>("w_state", 0);
      if (person == 7)
      {
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }

    pub_vel_.publish(cmd);
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Turn_no_detect>("Turn_no_detect");
}
