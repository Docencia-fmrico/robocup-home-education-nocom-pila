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

#include "robocup_nocom_pila/PIDController.hpp"

#include <string>
#include "std_msgs/String.h"

namespace robocup_nocom_pila
{

class Follow_person_cml : public BT::ActionNodeBase
{
public:
    explicit Follow_person_cml(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();
    
    void EndCallback(const std_msgs::String::ConstPtr& msg);

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::InputPort<float>("r_dist"),
            BT::InputPort<int>("r_centre"),
            BT::OutputPort<int>("counter"),
            BT::InputPort<int>("counter")
            };
    }



private:
    const double MIN_TURN_SPEED = 0.0;
    const double MAX_TURN_SPEED = 0.7;
    const double MIN_RANG_BOX = -1.0;
    const double MAX_RANG_BOX = 1.0;

    const double MIN_FORW_SPEED = 0.0;
    const double MAX_FORW_SPEED = 0.25;
    const double MIN_FORW_DIST = 0.0;
    const double MAX_FORW_DIST = 1.0;

    const int MIN_DISTANCE = 0.9;
    const int MAX_VEL_DISTANCE = 3.0;    // Distances greater than this one will make the robot go max velocity

    const int SECURITY_DIST = 0.5;      // Meters from last person's scan to avoid new person

    ros::NodeHandle n_;
    ros::Publisher pub_vel_;
    ros::Subscriber sub_stop;

    float dist_r, last_dist = 0;

    double centre_r;
    bool is_person = false;  
    bool arrived = false;
    PIDController turn_pid_, forw_pid_;
    int counter_ = 0;
    double ang_vel_, lin_vel_;
    int bbx_counter_ = 0;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_FOLLOW_PERSON_CML_H
