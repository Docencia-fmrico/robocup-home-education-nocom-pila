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

#include "robocup_nocom_pila/Go_person_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Go_person_fmm::Go_person_fmm(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0), nh("~")
{
}

void
Go_person_fmm::halt()
{
  ROS_INFO("Go_person_fmm halt");
}

BT::NodeStatus
Go_person_fmm::tick()
{
  ROS_INFO("Go_person_fmm tick");
  person = getInput<int>("r_person").value();
  // std::cerr << person << std::endl;

  switch (person)
  {
    case 1:
      nh.getParam("px1", px);
      nh.getParam("py1", py);
      nh.getParam("pz1", pz);
      nh.getParam("ox1", ox);
      nh.getParam("oy1", oy);
      nh.getParam("oz1", oz);
      nh.getParam("ow1", ow);
    break;
    case 2:
      nh.getParam("px2", px);
      nh.getParam("py2", py);
      nh.getParam("pz2", pz);
      nh.getParam("ox2", ox);
      nh.getParam("oy2", oy);
      nh.getParam("oz2", oz);
      nh.getParam("ow2", ow);
    break;
    case 3:
      nh.getParam("px3", px);
      nh.getParam("py3", py);
      nh.getParam("pz3", pz);
      nh.getParam("ox3", ox);
      nh.getParam("oy3", oy);
      nh.getParam("oz3", oz);
      nh.getParam("ow3", ow);
    break;
    case 4:
      nh.getParam("px4", px);
      nh.getParam("py4", py);
      nh.getParam("pz4", pz);
      nh.getParam("ox4", ox);
      nh.getParam("oy4", oy);
      nh.getParam("oz4", oz);
      nh.getParam("ow4", ow);
    break;
    case 5:
      nh.getParam("px5", px);
      nh.getParam("py5", py);
      nh.getParam("pz5", pz);
      nh.getParam("ox5", ox);
      nh.getParam("oy5", oy);
      nh.getParam("oz5", oz);
      nh.getParam("ow5", ow);
    break;
    case 6:
      nh.getParam("px6", px);
      nh.getParam("py6", py);
      nh.getParam("pz6", pz);
      nh.getParam("ox6", ox);
      nh.getParam("oy6", oy);
      nh.getParam("oz6", oz);
      nh.getParam("ow6", ow);
    break;
    
  }

  std::cerr << px << std::endl;
  std::cerr << py << std::endl;
  std::cerr << pz << std::endl;
  std::cerr << ox << std::endl;
  std::cerr << oy << std::endl;
  std::cerr << oz << std::endl;
  std::cerr << ow << std::endl;

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
  factory.registerNodeType<robocup_nocom_pila::Go_person_fmm>("Go_person_fmm");
}
