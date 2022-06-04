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

#include "robocup_nocom_pila/Goto_chair_rec.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Goto_chair_rec::Goto_chair_rec(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0), nh("~")
{
}

void
Goto_chair_rec::halt()
{
  ROS_INFO("Goto_chair_rec halt");
}

BT::NodeStatus
Goto_chair_rec::tick()
{
  ROS_INFO("Goto_chair_rec tick");
  chair = getInput<int>("r_chair").value();
  age = getInput<int>("r_age").value();
  // std::cerr << chair << std::endl;
  if (age >= 55)
  {
    nh.getParam("cx5", cx);
    nh.getParam("cy5", cy);
    nh.getParam("cz5", cz);
    nh.getParam("ox5", ox);
    nh.getParam("oy5", oy);
    nh.getParam("oz5", oz);
    nh.getParam("ow5", ow);
  }
  else
  {
    switch (chair)
    {
      case 1:
        nh.getParam("cx1", cx);
        nh.getParam("cy1", cy);
        nh.getParam("cz1", cz);
        nh.getParam("ox1", ox);
        nh.getParam("oy1", oy);
        nh.getParam("oz1", oz);
        nh.getParam("ow1", ow);
      break;
      case 2:
        nh.getParam("cx2", cx);
        nh.getParam("cy2", cy);
        nh.getParam("cz2", cz);
        nh.getParam("ox2", ox);
        nh.getParam("oy2", oy);
        nh.getParam("oz2", oz);
        nh.getParam("ow2", ow);
      break;
      case 3:
        nh.getParam("cx3", cx);
        nh.getParam("cy3", cy);
        nh.getParam("cz3", cz);
        nh.getParam("ox3", ox);
        nh.getParam("oy3", oy);
        nh.getParam("oz3", oz);
        nh.getParam("ow3", ow);
      break;
      case 4:
        nh.getParam("cx4", cx);
        nh.getParam("cy4", cy);
        nh.getParam("cz4", cz);
        nh.getParam("ox4", ox);
        nh.getParam("oy4", oy);
        nh.getParam("oz4", oz);
        nh.getParam("ow4", ow);
      break;
      case 5:
        nh.getParam("cx5", cx);
        nh.getParam("cy5", cy);
        nh.getParam("cz5", cz);
        nh.getParam("ox5", ox);
        nh.getParam("oy5", oy);
        nh.getParam("oz5", oz);
        nh.getParam("ow5", ow);
      break;
      case 6:
        nh.getParam("cx6", cx);
        nh.getParam("cy6", cy);
        nh.getParam("cz6", cz);
        nh.getParam("ox6", ox);
        nh.getParam("oy6", oy);
        nh.getParam("oz6", oz);
        nh.getParam("ow6", ow);
      break;
    }

  }
/*
  std::cerr << cx << std::endl;
  std::cerr << cy << std::endl;
  std::cerr << cz << std::endl;
  std::cerr << ox << std::endl;
  std::cerr << oy << std::endl;
  std::cerr << oz << std::endl;
  std::cerr << ow << std::endl;
*/
  if (action)
  {
    my_node.doWork(cx, cy, cz, ox, oy, oz, ow, 200);
    action = false;
  }
  if (my_node.aborted())
  {
    action = true;
    return BT::NodeStatus::RUNNING;
  }
  if (my_node.checkstatus())
  {
    action = true;
    chair++;  //-- Cambia a la siguiente silla
    setOutput<int>("w_chair", chair);
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
  factory.registerNodeType<robocup_nocom_pila::Goto_chair_rec>("Goto_chair_rec");
}
