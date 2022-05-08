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

#ifndef ROBOCUP_NOCOM_PILA_GO_HOME_H
#define ROBOCUP_NOCOM_PILA_GO_HOME_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include "<sensor_msgs/Image.h>"

#include "../src/nav.cpp"

#include <string>

namespace robocup_nocom_pila
{

class Go_arena : public BT::ActionNodeBase
{
public:
    explicit Go_arena(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::vector<float>>("posx"), BT::InputPort<std::vector<float>>("posy") };
    }

private:
    MyNode my_node;

    ros::NodeHandle nh;
    float px;
    float py;
    const float pz = 0;
    const float ox = 0;
    const float oy = 0;
    const float oz = 0;
    const float ow = 1;

    int action = true;
    float dist;
    bool finish = false;

    int counter_;
    int target = 0;

    std::vector<float> xvector = std::vector<float>(20, 0);
    std::vector<float> yvector = std::vector<float>(20, 0);
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_GO_HOME_H
