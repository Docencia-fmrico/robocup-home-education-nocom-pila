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

#ifndef ROBOCUP_NOCOM_PILA_DETECT_BAG_CML_H
#define ROBOCUP_NOCOM_PILA_DETECT_BAG_CML_H

#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "std_msgs/String.h"
#include "../src/chatbot.cpp"


#include <string>

namespace robocup_nocom_pila
{

class Detect_bag_cml : public BT::ActionNodeBase
{
public:
    explicit Detect_bag_cml(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void 
    ImageCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);
    
    void 
    OrderCallback(const std_msgs::String::ConstPtr& msg);
    void
    get_order();

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("r_order")};
    }

private:
    /*const float ADVANCE_SPEED = 0.1;
    const float TURNING_SPEED = 0.35;

    
    ros::Publisher vel_pub_;
    ros::Subscriber dist_point_person;
    ros::Subscriber px_point_person;

    float dist;
    int point;*/
    int px_max;
    int px_min;
    ros::Subscriber bagDarknet;
    ros::Subscriber orderDiag;
    ros::NodeHandle nh;
    int counter_;
    int num = 0;
    int repeticiones = 0;
    std::string order;
    gb_dialog::ExampleDF forwarder;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_DETECT_BAG_CML_H
