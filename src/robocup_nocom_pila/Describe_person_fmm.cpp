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

#include "robocup_nocom_pila/Describe_person_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Describe_person_fmm::Describe_person_fmm(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  // dist_sub = nh_.subscribe("/dist_person", 1, &Describe_person_fmm::PerceivePersonCallback, this);
}

void
Describe_person_fmm::halt()
{
    ROS_INFO("Describe_person_fmm halt");
}

BT::NodeStatus
Describe_person_fmm::tick()
{
    ROS_INFO("Describe_person_fmm tick");
    

    name_r = getInput<std::string>("r_name").value();
    color_r = getInput<std::string>("r_color").value();
    object_r = getInput<std::string>("r_object").value();

    sleep(1);

    switch(num)
      {
        case 0:
          if(name_r != "")
            num = 1;
            std:: cerr << name_r << std::endl;
            return BT::NodeStatus::RUNNING;
        break;
        case 1:
          if(color_r != "")
            num = 2;
            std:: cerr << "\t" << color_r << std::endl;
            return BT::NodeStatus::RUNNING;
        break;
        case 2:
          if(object_r != "")
            num = 3;
            std:: cerr << "\t" << object_r << std::endl;
            return BT::NodeStatus::RUNNING;
        break;
        case 3:
          num = 0; 
          forwarder.speak(name_r + " tishirt is " + color_r + " and have a " + object_r);
          sleep(1);
          return BT::NodeStatus::SUCCESS;
        break;
      }
      return BT::NodeStatus::RUNNING;

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Describe_person_fmm>("Describe_person_fmm");
}
}