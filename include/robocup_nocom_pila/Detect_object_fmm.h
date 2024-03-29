// Copyright 2022 Nocom-pila
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

#ifndef ROBOCUP_NOCOM_PILA_DETECT_OBJECT_FMM_H
#define ROBOCUP_NOCOM_PILA_DETECT_OBJECT_FMM_H

#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <string>

namespace robocup_nocom_pila
{

class Detect_object_fmm : public BT::ActionNodeBase
{
public:
    explicit Detect_object_fmm(const std::string& name, const BT::NodeConfiguration& config);

    void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

    void halt();

    BT::NodeStatus tick();

    void DetectObjectBBXCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& obj);
    void DetectObjectImageCallback(const sensor_msgs::ImageConstPtr& image);

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<std::string>("w_object"), BT::InputPort<int>("r_person")};
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber objects_bbx;
    ros::Subscriber objects_image;

    cv_bridge::CvImagePtr img_ptr_depth;

    int person;
    bool is_object = false;
    float dist = 10;
    int repeticiones = 0;
    int px = 0;
    int py = 0;
    int counter_;
    std::string object = "";
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_DETECT_OBJECT_FMM_H
