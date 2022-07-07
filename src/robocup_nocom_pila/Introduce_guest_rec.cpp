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

#include "robocup_nocom_pila/Introduce_guest_rec.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Introduce_guest_rec::Introduce_guest_rec(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  // dist_sub = nh_.subscribe("/dist_chair", 1, &Introduce_guest_rec::PerceivePersonCallback, this);
}

void
Introduce_guest_rec::halt()
{
    ROS_INFO("Introduce_guest_rec halt");
}

BT::NodeStatus
Introduce_guest_rec::tick()
{
  ROS_INFO("Introduce_guest_rec tick");
  chair = getInput<int>("r_chair").value();
  if ( chair >= 7)
  {
    return BT::NodeStatus::SUCCESS;
  }

  switch (chair)
  {
    case 2:
      n_chair = " 1 ";
    break;
    case 3:
      n_chair = " 2 ";
    break;
    case 4:
      n_chair = " 3 ";
    break;
    case 5:
      n_chair = " 4 ";
    break;
    case 6:
      n_chair = " 5 ";
    break;
    case 7:
      n_chair = " 6 ";
    break;
  }

  name_r = getInput<std::string>("r_name").value();
  age_r = getInput<std::string>("r_age").value();
  drink_r = getInput<std::string>("r_drink").value();

  ROS_WARN("The guest:");
  std::cerr << "The guest is in chair " << n_chair << std::endl;
  std::cerr << "Name: " << name_r << std::endl;
  std::cerr << "Age: " << age_r << std::endl;
  std::cerr << "Drink: " << drink_r << std::endl;

  sleep(0.1);

  forwarder.speak(name_r + "is " + age_r + "years old, is in the chair number "
   + n_chair + " and it's favorite drink is " + drink_r);

  sleep(4);

  if (count >= 2)
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
  factory.registerNodeType<robocup_nocom_pila::Introduce_guest_rec>("Introduce_guest_rec");
}
