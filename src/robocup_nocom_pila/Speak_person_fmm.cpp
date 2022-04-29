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

#include "robocup_nocom_pila/Speak_person_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"


namespace robocup_nocom_pila
{

Speak_person_fmm::Speak_person_fmm(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
   color_sub = nh.subscribe("/sound/color", 1, &Speak_person_fmm::colorCallback, this);
   name_sub = nh.subscribe("/sound/name", 1, &Speak_person_fmm::nameCallback, this);
}

void
Speak_person_fmm::halt()
{
  ROS_INFO("Speak_person_fmm halt");
}

void Speak_person_fmm::nameCallback(const std_msgs::String::ConstPtr& msg)
{
  name = msg->data;
  setOutput<std::string>("w_name", name);
}


void Speak_person_fmm::colorCallback(const std_msgs::String::ConstPtr& msg)
{
  color = msg->data;
  setOutput<std::string>("w_color", color);
}

void
Speak_person_fmm::get_name()
{
  ROS_INFO("Speak_person_fmm get name");
  sleep(1);
  forwarder.speak("What is your name?");
  sleep(1);
  forwarder.listen();
  sleep(1);
}

void
Speak_person_fmm::get_color()
{
  ROS_INFO("Speak_person_fmm get color");
  sleep(1); 
  forwarder.speak("Which is your t-shirt color?");
  sleep(1);
  forwarder.listen();
  sleep(1);
}

BT::NodeStatus
Speak_person_fmm::tick()
{
  ROS_INFO("Speak_person_fmm tick");
  
  std::cerr << name << "\t" << color << std::endl;

  if(name != "")
  {
    num = 1;
  }
  if(name != "" && color != "")
  {
    num = 2;
  }

  if (repeticiones == 0)
  {
    switch(num)
    {
      case 0:
        get_name();
        repeticiones = 1;
        return BT::NodeStatus::RUNNING;
      break;
      case 1:
        get_color();
        repeticiones = 1;
        return BT::NodeStatus::RUNNING;
      break;
      case 2:
        num = 0; 
        name = "";
        color = "";
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
  factory.registerNodeType<robocup_nocom_pila::Speak_person_fmm>("Speak_person_fmm");
}
