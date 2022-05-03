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

#ifndef ROBOCUP_NOCOM_PILA_START_H
#define ROBOCUP_NOCOM_PILA_START_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "../src/chatbot.cpp"

#include <string>
#include "std_msgs/String.h"

namespace robocup_nocom_pila
{

class Start : public BT::ActionNodeBase
{
public:
    explicit Start(const std::string& name);

    void halt();
    void get_init();
    BT::NodeStatus tick();
    void startCallback(const std_msgs::String::ConstPtr& msg);



private:
    ros::NodeHandle nh;
    gb_dialog::ExampleDF forwarder;
    ros::Subscriber init_sub;
    int repeticiones = 0;
    int num = 0;
    int counter_;
    std::string init;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_START_H
