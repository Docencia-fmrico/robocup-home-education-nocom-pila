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

#ifndef ROBOCUP_NOCOM_PILA_DETECT_PERSON_CML_H
#define ROBOCUP_NOCOM_PILA_DETECT_PERSON_CML_H

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

class Detect_person_cml : public BT::ActionNodeBase
{
public:
    explicit Detect_person_cml(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    void DetectPersonBBXCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& obj);
    void DetectPersonImageCallback(const sensor_msgs::ImageConstPtr& image);

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::OutputPort<float>("w_dist"),
            BT::OutputPort<double>("w_centre"),
            BT::OutputPort<int>("counter"),
            BT::InputPort<int>("counter")
        };
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber objects_bbx;
    ros::Subscriber objects_image;

    cv_bridge::CvImagePtr img_ptr_depth;

    float dist_w = 0;
    double centre_w = 0;
    int px = 0;
    int py = 0;
    int area = 0;
    bool is_person = false;
    int cont = 0;
    int bbx_counter_ = 0;
    int counter_;
};

}  // namespace robocup_nocom_pila

#endif  // ROBOCUP_NOCOM_PILA_DETECT_PERSON_CML_H
