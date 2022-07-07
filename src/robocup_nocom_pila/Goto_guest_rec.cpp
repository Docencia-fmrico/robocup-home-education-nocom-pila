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

#include "robocup_nocom_pila/Goto_guest_rec.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Goto_guest_rec::Goto_guest_rec(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0), nh("~")
{
  nh.getParam("gpx", gx);
  nh.getParam("gpy", gy);
  nh.getParam("gpz", gz);
  nh.getParam("gox", ox);
  nh.getParam("goy", oy);
  nh.getParam("goz", oz);
  nh.getParam("gow", ow);
}

void
Goto_guest_rec::halt()
{
    ROS_INFO("Goto_guest_rec halt");
}

BT::NodeStatus
Goto_guest_rec::tick()
{
  ROS_INFO("Goto_guest_rec tick");

  if (action)
  {
    my_node.doWork(gx, gy, gz, ox, oy, oz, ow, 200);
    action = false;
  }

  if (my_node.checkstatus())
  {
    action = true;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::RUNNING;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Goto_guest_rec>("Goto_guest_rec");
}
