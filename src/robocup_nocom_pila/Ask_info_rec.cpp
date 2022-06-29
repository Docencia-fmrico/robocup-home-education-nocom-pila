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

Ask_info_rec::Ask_info_rec(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  age_sub = nh.subscribe("/sound/age", 1, &Ask_info_rec::ageCallback, this);
  name_sub = nh.subscribe("/sound/name", 1, &Ask_info_rec::nameCallback, this);
  drink_sub = nh.subscribe("/sound/drink", 1, &Ask_info_rec::drinkCallback, this);
}

void
Ask_info_rec::halt()
{
  ROS_INFO("Ask_info_rec halt");
}

void Ask_info_rec::nameCallback(const std_msgs::String::ConstPtr& msg)
{
  name = msg->data;
  setOutput<std::string>("w_name", name);
}


void Ask_info_rec::ageCallback(const std_msgs::String::ConstPtr& msg)
{
  age = msg->data;
  setOutput<std::string>("w_age", age);
}

void Ask_info_rec::drinkCallback(const std_msgs::String::ConstPtr& msg)
{
  drink = msg->data;
  setOutput<std::string>("w_drink", drink);
}

void
Ask_info_rec::get_name()
{
  ROS_INFO("Ask_info_rec get name");
  sleep(0.1);
  forwarder.speak("What is your name?");
  sleep(0.5);
  forwarder.listen();
}

void
Ask_info_rec::get_age()
{
  ROS_INFO("Ask_info_rec get age");
  sleep(0.1);
  forwarder.speak("How old are you?");
  sleep(0.5);
  forwarder.listen();
}

void
Ask_info_rec::get_drink()
{
  ROS_INFO("Ask_info_rec get favourite drink");
  sleep(0.1);
  forwarder.speak("And what's your favourite drink?");
  sleep(0.5);
  forwarder.listen();
}

BT::NodeStatus
Ask_info_rec::tick()
{
  ROS_INFO("Ask_info_rec tick");

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
  if (name != "" && age != "" && drink != "")
  {
    num = 3;
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
        get_drink();
        repeticiones = 1;
        return BT::NodeStatus::RUNNING;
      break;
      case 3:
        forwarder.speak("Follow me uwu");
        num = 0;
        name = "";
        age = "";
        drink = "";
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
  factory.registerNodeType<robocup_nocom_pila::Ask_info_rec>("Ask_info_rec");
}
