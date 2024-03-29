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

#include "robocup_nocom_pila/Go_arena_nf.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Go_arena_nf::Go_arena_nf(const std::string& name,  const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0),  nh("~")
{
}

void
Go_arena_nf::halt()
{
    ROS_INFO("Go_arena_nf halt");
}

BT::NodeStatus
Go_arena_nf::tick()
{
  ROS_INFO("Go_arena_nf tick");
/*
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
  */

  xvector = getInput<std::vector<float>>("posx").value();
  yvector = getInput<std::vector<float>>("posy").value();

  while (xvector[target] ==0 ) { target--; }
  move = true;
  if (move)
  {
    px = xvector[target];
    py = yvector[target];
    my_node.doWork(px, py, pz, ox, oy, oz, ow, 200);
    move = false;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Go_arena_nf>("Go_arena_nf");
}
