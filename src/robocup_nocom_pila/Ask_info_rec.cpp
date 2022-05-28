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

#include "robocup_nocom_pila/Ask_info_rec.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Ask_info::Ask_info(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  age_sub = nh.subscribe("/sound/age", 1, &Ask_info::ageCallback, this);
  name_sub = nh.subscribe("/sound/name", 1, &Ask_info::nameCallback, this);
}

void
Ask_info::halt()
{
  ROS_INFO("Ask_info halt");
}

void Ask_info::nameCallback(const std_msgs::String::ConstPtr& msg)
{
  name = msg->data;
  setOutput<std::string>("w_name", name);
}


void Ask_info::ageCallback(const std_msgs::String::ConstPtr& msg)
{
  age = msg->data;
  setOutput<std::string>("w_age", age);
}

void Ask_info::drinkCallback(const std_msgs::String::ConstPtr& msg)
{
  age = msg->data;
  setOutput<std::string>("w_drink", age);
}

void
Ask_info::get_name()
{
  ROS_INFO("Ask_info get name");
  sleep(0.1);
  forwarder.speak("What is your name?");
  sleep(0.5);
  forwarder.listen();
}

void
Ask_info::get_age()
{
  ROS_INFO("Ask_info get age");
  sleep(0.1);
  forwarder.speak("How old are you?");
  sleep(0.5);
  forwarder.listen();
}

void
Ask_info::get_drink()
{
  ROS_INFO("Ask_info get favourite drink");
  sleep(0.1);
  forwarder.speak("And what's your favourite drink?");
  sleep(0.5);
  forwarder.listen();
}

BT::NodeStatus
Ask_info::tick()
{
  ROS_INFO("Ask_info tick");

  chair = getInput<int>("r_chair").value();
  if (chair >= 7)
  {
    return BT::NodeStatus::SUCCESS;
  }
  // std::cerr << name << "\t" << age << std::endl;

  if (name != "")
  {
    num = 1;
  }
  if (name != "" && age != "")
  {
    num = 2;
  }

  if (repeticiones == 0)
  {
    switch (num)
    {
      case 0:
        get_name();
        repeticiones = 1;
        return BT::NodeStatus::RUNNING;
      break;
      case 1:
        get_age();
        repeticiones = 1;
        return BT::NodeStatus::RUNNING;
      break;
      case 2:
        num = 0;
        name = "";
        age = "";
        return BT::NodeStatus::SUCCESS;
      break;
    }
  }
  else
  {
    repeticiones = 0;
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Ask_info>("Ask_info");
}