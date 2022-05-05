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

#ifndef ROBOCUP_NOCOM_PILA_FOLLOW_PERSON_CML_H
#define ROBOCUP_NOCOM_PILA_FOLLOW_PERSON_CML_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"

#include "robocup_nocom_pila/PIDController.hpp"

#include <string>


namespace robocup_nocom_pila
{

class Follow_person_cml : public BT::ActionNodeBase
{
public:
    explicit Follow_person_cml(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<float>("r_dist"), BT::InputPort<int>("r_centre"), BT::OutputPort<int>("counter"), BT::InputPort<int>("counter")};
    }

private:

    const double MIN_TURN_SPEED = 0.0;
    const double MAX_TURN_SPEED = 0.7;
    const double MIN_RANG_BOX = -1.0;
    const double MAX_RANG_BOX = 1.0;

    const double MIN_FORW_SPEED = 0.0;
    const double MAX_FORW_SPEED = 0.45;
    const double MIN_FORW_DIST = 0.0;
    const double MAX_FORW_DIST = 1.0;

    const double MIN_BACKW_DIST = 0.1;
    const double MAX_BACKW_DIST = 1;

    const float MIN_TURNING_SPEED = 0.05;
    const float MAX_TURNING_SPEED = 0.45;

    ros::NodeHandle n_;
    ros::Publisher pub_vel_;

    float dist_r;
    double centre_r;
    bool is_person = false;
    PIDController turn_pid_, forw_pid_;
    int point;
    int counter_ = 0;
    double ang_vel_, lin_vel_;
    int bbx_counter_ = 0;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_FOLLOW_PERSON_CML_H
