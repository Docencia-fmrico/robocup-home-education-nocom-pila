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

#ifndef ROBOCUP_NOCOM_PILA_ASK_INFO_H
#define ROBOCUP_NOCOM_PILA_ASK_INFO_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include "../src/chatbot.cpp"

#include <string>
#include "std_msgs/String.h"

namespace robocup_nocom_pila
{

class Ask_info_rec : public BT::ActionNodeBase
{
public:
    explicit Ask_info_rec(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    void get_name();

    void get_age();

    void get_drink();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::OutputPort<std::string>("w_name"),
            BT::OutputPort<std::string>("w_age"),
            BT::OutputPort<std::string>("w_drink"),
            BT::InputPort<int>("r_chair")
        };
    }

    void nameCallback(const std_msgs::String::ConstPtr& msg);
    void ageCallback(const std_msgs::String::ConstPtr& msg);
    void drinkCallback(const std_msgs::String::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber name_sub;  // subscriptor al chatbot.
    ros::Subscriber age_sub;  // subscriptor al chatbot.
    gb_dialog::ExampleDF forwarder;

    int chair;
    float dist;
    int num = 0;
    int repeticiones = 0;
    int counter_;
    std::string age;
    std::string name;
    std::string drink;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_ASK_INFO_H
