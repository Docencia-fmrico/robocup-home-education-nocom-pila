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

#ifndef ROBOCUP_NOCOM_PILA_TURN_NO_DETECT_H
#define ROBOCUP_NOCOM_PILA_TURN_NO_DETECT_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>

#include <string>

namespace robocup_nocom_pila
{

class Turn_no_detect : public BT::ActionNodeBase
{
public:
    explicit Turn_no_detect(const std::string& name, const BT::NodeConfiguration & config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<int>("r_state")};
    }

private:

    ros::NodeHandle nh;
    double TURNING_TIME;
    double TURNING_SPEED;
    ros::Time turn_ts_;

    ros::Publisher pub_vel_;
    int time = 0;
    int state = 0;
    int counter_;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_TURN_NO_DETECT_H
