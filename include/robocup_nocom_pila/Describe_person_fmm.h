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

#ifndef ROBOCUP_NOCOM_PILA_DESCRIBE_PERSON_FMM_H
#define ROBOCUP_NOCOM_PILA_DESCRIBE_PERSON_FMM_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include <string>
#include "../src/chatbot.cpp"

namespace robocup_nocom_pila
{

class Describe_person_fmm : public BT::ActionNodeBase
{
public:
    explicit Describe_person_fmm(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>("r_name"), BT::InputPort<std::string>("r_color"), BT::InputPort<std::string>("r_object"), BT::InputPort<int>("r_person")};
    }

private:
    int person; 
    int counter_;
    int count = 1;
    int num = 0;
    std::string color_r;
    std::string name_r;
    std::string object_r;
    gb_dialog::ExampleDF forwarder;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_DESCRIBE_PERSON_FMM_H
