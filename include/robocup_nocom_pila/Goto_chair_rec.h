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

#ifndef ROBOCUP_NOCOM_PILA_GOTO_CHAIR_REC_H
#define ROBOCUP_NOCOM_PILA_GOTO_CHAIR_REC_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"

#include "../src/nav.cpp"

#include <string>

namespace robocup_nocom_pila
{

class Goto_chair_rec : public BT::ActionNodeBase
{
public:
    explicit Goto_chair_rec(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<int>("r_chair"),
            BT::OutputPort<int>("w_chair"),
            BT::InputPort<int>("r_age")
        };
    }

private:
    MyNode my_node;

    ros::NodeHandle nh;
    float cx;
    float cy;
    float cz;
    float ox;
    float oy;
    float oz;
    float ow;

    int chair, age;

    int action = true;
    int counter_;
    float dist;
    bool finish = false;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_GOTO_CHAIR_REC_H
