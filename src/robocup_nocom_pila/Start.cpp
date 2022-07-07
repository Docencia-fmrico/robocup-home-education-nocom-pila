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

#include "robocup_nocom_pila/Start.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Start::Start(const std::string& name)
: BT::ActionNodeBase(name, {}), counter_(0)
{
    init_sub = nh.subscribe("/sound/start", 1, &Start::startCallback, this);
}

void
Start::halt()
{
    ROS_INFO("Start halt");
}

void
Start::get_init()
{
  ROS_INFO("Start get init");
  forwarder.listen();
}

void
Start::startCallback(const std_msgs::String::ConstPtr& msg)
{
    init = msg->data;
}

BT::NodeStatus
Start::tick()
{
    ROS_INFO("Start tick");
    // std::cerr << init <<  std::endl;

    if (init != "")
    {
        num = 1;
    }

    if (repeticiones == 0)
    {
        switch (num)
        {
        case 0:
            get_init();
            repeticiones = 1;
            return BT::NodeStatus::RUNNING;
        break;
        case 1:
            num = 0;
            init = "";
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
  factory.registerNodeType<robocup_nocom_pila::Start>("Start");
}
