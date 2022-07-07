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

#include "robocup_nocom_pila/Describe_person_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

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
  person = getInput<int>("r_person").value();
  if ( person >= 7)
  {
    return BT::NodeStatus::SUCCESS;
  }

  switch (person)
  {
    case 2:
      n_person = " 1 ";
    break;
    case 3:
      n_person = " 2 ";
    break;
    case 4:
      n_person = " 3 ";
    break;
    case 5:
      n_person = " 4 ";
    break;
    case 6:
      n_person = " 5 ";
    break;
    case 7:
      n_person = " 6 ";
    break;
  }

  name_r = getInput<std::string>("r_name").value();
  color_r = getInput<std::string>("r_color").value();
  object_r = getInput<std::string>("r_object").value();

  sleep(0.1);

  // forwarder.speak("The person number " + n_person + " is " + name_r +
  //  " wear a " + color_r + " tishirt and have a " + object_r);
  forwarder.speak("The person number " + n_person + " is " + name_r + " and have a " + object_r);  // sin color

  sleep(4);

  if (count >= 3)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    count = count + 1;
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Describe_person_fmm>("Describe_person_fmm");
}
