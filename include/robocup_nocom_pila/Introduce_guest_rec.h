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

#ifndef ROBOCUP_NOCOM_PILA_INTRODUCE_GUEST_REC_H
#define ROBOCUP_NOCOM_PILA_INTRODUCE_GUEST_REC_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/String.h"
#include <string>
#include "../src/chatbot.cpp"

namespace robocup_nocom_pila
{

class Introduce_guest_rec : public BT::ActionNodeBase
{
public:
    explicit Introduce_guest_rec(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<std::string>("r_name"),
            BT::InputPort<std::string>("r_age"),
            BT::InputPort<std::string>("r_drink"),
            BT::InputPort<int>("r_chair")
            };
    }

private:
    int chair;
    int counter_;
    int count = 1;
    int num = 0;
    std::string age_r;
    std::string name_r;
    std::string drink_r;
    std::string n_chair;
    gb_dialog::ExampleDF forwarder;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_INTRODUCE_GUEST_REC_H
