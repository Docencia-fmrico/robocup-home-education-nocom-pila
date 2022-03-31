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

#include "robocup_nocom_pila/Bag_taked.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Bag_taked::Bag_taked(const std::string& name/*, const BT::NodeConfiguration & config*/)
: BT::ActionNodeBase(name, {} /*config*/), counter_(0)
{
  // dist_sub = nh_.subscribe("/dist_person", 1, &Bag_taked::PerceivePersonCallback, this);
}

void
Bag_taked::halt()
{
    ROS_INFO("Bag_taked halt");
}

BT::NodeStatus
Bag_taked::tick()
{
    ROS_INFO("Bag_taked tick");
/*
    setOutput<std::string>("person", "hola");

    std::cerr << dist << std::endl;
*/
    return BT::NodeStatus::SUCCESS;
}



}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Bag_taked>("Bag_taked");
}
