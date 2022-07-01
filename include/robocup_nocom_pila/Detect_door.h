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

#ifndef ROBOCUP_NOCOM_PILA_DETECT_DOOR__H
#define ROBOCUP_NOCOM_PILA_DETECT_DOOR__H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/LaserScan.h"

#include <string>

namespace robocup_nocom_pila
{

class Detect_door : public BT::ActionNodeBase
{
public:
    explicit Detect_door(const std::string& name, const BT::NodeConfiguration& config);

    void halt();
    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("r_door"), BT::OutputPort<int>("w_door")};
    }

private:
    ros::NodeHandle nh;
    static const int TURNING_RIGHT = 3;
    const float DISTANCE_ = 0.7;  // minim distance the kobuki change state
    
    int max_array_;  // contains the max size of the array
    int max_sweep_left;  // max left angle kobuki uses to detect
    int min_sweep_right;  // min left angle kobuki uses to detect (center)
    int max_sweep_right;  // max rigth angle kobuki uses to detect
    int min_sweep_left;  // min rigth angle kobuki uses to detect (center)
    int laser_detected_;  // true when the laser detect something (on any side)
    bool door = true;  // true when the bumper detect something on the left side. False on the rigth side.
    int n_door;
    float dist;
    ros::Subscriber sub_laser_;
    int counter_;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_DETECT_DOOR__H
