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

#ifndef ROBOCUP_NOCOM_PILA_GO_HOME_FMM_H
#define ROBOCUP_NOCOM_PILA_GO_HOME_FMM_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>

#include "../src/fsm_nav.cpp"

#include <string>

namespace robocup_nocom_pila
{

class Go_home_fmm : public BT::ActionNodeBase
{
public:
    explicit Go_home_fmm(const std::string& name/*, const BT::NodeConfiguration& config*/);

    void halt();

    BT::NodeStatus tick();
/*
    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<float>("dist_w")};
    }
*/
private:
    MyNode my_node;

    ros::NodeHandle nh;
    float px;
    float py;
    float pz;
    float ox;
    float oy;
    float oz;
    float ow;

    int action = true;
    float dist;
    bool finish = false;
    
    int counter_;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_GO_HOME_FMM_H
