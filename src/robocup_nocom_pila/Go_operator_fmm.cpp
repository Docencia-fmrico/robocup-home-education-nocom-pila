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

#include "robocup_nocom_pila/Go_operator_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"


#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"


namespace robocup_nocom_pila
{

Go_operator_fmm::Go_operator_fmm(const std::string& name)
: BT::ActionNodeBase(name, {}), counter_(0), nh("~")
{
  nh.getParam("opx", px);
  nh.getParam("opy", py);
  nh.getParam("opz", pz);
  nh.getParam("oox", ox);
  nh.getParam("ooy", oy);
  nh.getParam("ooz", oz);
  nh.getParam("oow", ow);
}

void
Go_operator_fmm::halt()
{
  ROS_INFO("Go_operator_fmm halt");
}

BT::NodeStatus
Go_operator_fmm::tick()
{
  ROS_INFO("Go_operator_fmm tick");

  if (action)
  {
    my_node.doWork(px, py, pz, ox, oy, oz, ow, 200);
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
  factory.registerNodeType<robocup_nocom_pila::Go_operator_fmm>("Go_operator_fmm");
}
